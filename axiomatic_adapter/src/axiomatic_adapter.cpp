// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "axiomatic_adapter/axiomatic_adapter.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/system/error_code.hpp>

#include "axiomatic_adapter/axiomatic_frame_parser.hpp"

namespace polymath
{
namespace can
{

class AxiomaticAdapter::AxiomaticAdapterImpl
{
public:
  AxiomaticAdapterImpl(
    const std::string & ip_address,
    const std::string & port,
    const std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> && receive_callback_function,
    const std::function<void(AxiomaticAdapter::socket_error_string_t error)> && error_callback_function,
    const std::chrono::milliseconds & receive_timeout_ms,
    bool verbose)
  : tcp_io_context_()
  , work_guard_(boost::asio::make_work_guard(tcp_io_context_))
  , strand_(boost::asio::make_strand(tcp_io_context_))
  , tcp_socket_(tcp_io_context_)
  , ip_address_(ip_address)
  , port_(port)
  , receive_callback_(receive_callback_function)
  , error_callback_(error_callback_function)
  , receive_timeout_ms_(receive_timeout_ms)
  {
    parser_.set_verbose(verbose);
    // One worker thread runs the io_context for the adapter's lifetime. All
    // socket operations are posted onto strand_, so they execute serialized
    // on this thread even when called concurrently from the bridge thread.
    io_thread_ = std::thread([this]() {
      try {
        tcp_io_context_.run();
      } catch (const std::exception & e) {
        std::cerr << "[Axiomatic] io_context thread crashed: " << e.what() << std::endl;
      }
    });
  }

  ~AxiomaticAdapterImpl()
  {
    joinReceptionThread();
    closeSocket();

    // Allow the io_context worker to exit.
    work_guard_.reset();
    tcp_io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
  }

  bool openSocket()
  {
    try {
      boost::asio::ip::tcp::resolver resolver(tcp_io_context_);
      auto endpoints = resolver.resolve(ip_address_, port_);

      std::promise<boost::system::error_code> promise;
      auto future = promise.get_future();

      // Post the connect + timeout onto the strand so they execute on the io
      // thread. Both handlers race; the first one to fire sets the result.
      auto done = std::make_shared<std::atomic<bool>>(false);
      auto timer = std::make_shared<boost::asio::steady_timer>(strand_);
      timer->expires_after(TCP_IP_CONNECTION_TIMEOUT_MS);

      boost::asio::post(strand_, [this, endpoints, &promise, done, timer]() {
        boost::asio::async_connect(
          tcp_socket_, endpoints,
          boost::asio::bind_executor(
            strand_,
            [&promise, done, timer](const boost::system::error_code & ec,
                                    const boost::asio::ip::tcp::endpoint &) {
              if (done->exchange(true)) return;
              boost::system::error_code ignored;
              timer->cancel(ignored);
              promise.set_value(ec);
            }));

        timer->async_wait(boost::asio::bind_executor(
          strand_, [this, &promise, done](const boost::system::error_code & ec) {
            if (ec) return;
            if (done->exchange(true)) return;
            boost::system::error_code ignored;
            tcp_socket_.cancel(ignored);
            promise.set_value(boost::asio::error::timed_out);
          }));
      });

      auto connect_ec = future.get();
      if (connect_ec) {
        std::cerr << "Connection failed: " << connect_ec.message() << std::endl;
        socket_state_ = TCPSocketState::ERROR;
        return false;
      }

      // Disable Nagle: flash protocols depend on small request frames hitting
      // the wire immediately, not being buffered up to ~40 ms waiting for an
      // ACK or batch.
      boost::system::error_code nd_ec;
      // tcp_socket_.set_option(boost::asio::ip::tcp::no_delay(true), nd_ec);
      if (nd_ec) {
        std::cerr << "[Axiomatic] Failed to set TCP_NODELAY: " << nd_ec.message() << std::endl;
        // Non-fatal — connection is still usable, just slower.
      }

      socket_state_ = TCPSocketState::OPEN;
      return true;
    } catch (std::exception & e) {
      std::cerr << "Connection failed (exception): " << e.what() << std::endl;
      socket_state_ = TCPSocketState::ERROR;
      return false;
    }
  }

  bool closeSocket()
  {
    if (socket_state_ == TCPSocketState::CLOSED) {
      return true;
    }

    std::promise<bool> promise;
    auto future = promise.get_future();
    boost::asio::post(strand_, [this, &promise]() {
      boost::system::error_code ec;
      tcp_socket_.close(ec);
      if (ec) {
        std::cerr << "[ERROR] Failed to close TCP socket: " << ec.message() << std::endl;
        promise.set_value(false);
      } else {
        promise.set_value(true);
      }
    });

    bool ok = future.get();
    if (ok) {
      socket_state_ = TCPSocketState::CLOSED;
    }
    return ok;
  }

  bool startReceptionThread()
  {
    if (socket_state_ == TCPSocketState::CLOSED) {
      return false;
    }
    if (reception_active_.exchange(true)) {
      return true;  // already running
    }
    stop_thread_requested_ = false;
    boost::asio::post(strand_, [this]() { scheduleAsyncReceive(); });
    return true;
  }

  bool joinReceptionThread(
    const std::chrono::milliseconds & timeout = AxiomaticAdapter::JOIN_RECEPTION_TIMEOUT_MS)
  {
    if (!reception_active_.load()) {
      return true;
    }
    stop_thread_requested_ = true;

    // Cancel any in-flight async_receive on the strand so the handler runs
    // immediately with operation_aborted, observes the stop flag, and exits
    // the chain.
    boost::asio::post(strand_, [this]() {
      if (tcp_socket_.is_open()) {
        boost::system::error_code ignored;
        tcp_socket_.cancel(ignored);
      }
    });

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (reception_active_.load()) {
      if (std::chrono::steady_clock::now() > deadline) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame)
  {
    // Parser drains first: a single TCP read may have produced multiple
    // CAN frames, and we want every subsequent call to return one without
    // touching the socket.
    if (auto frame = drainParser()) {
      can_frame = *frame;
      return std::nullopt;
    }

    if (reception_active_.load()) {
      // Sync receive() and the async reception loop both pull from the same
      // socket — running them simultaneously would interleave bytes between
      // two parsers. Pick one mode per adapter instance.
      return std::optional<AxiomaticAdapter::socket_error_string_t>(
        "Sync receive() is not allowed while the reception thread is active");
    }

    auto rx_buf = std::make_shared<std::vector<uint8_t>>(1024, 0);
    std::promise<std::optional<AxiomaticAdapter::socket_error_string_t>> promise;
    auto future = promise.get_future();

    auto done = std::make_shared<std::atomic<bool>>(false);
    auto timer = std::make_shared<boost::asio::steady_timer>(strand_);
    timer->expires_after(receive_timeout_ms_);

    boost::asio::post(strand_, [this, rx_buf, &promise, done, timer]() {
      tcp_socket_.async_receive(
        boost::asio::buffer(*rx_buf),
        boost::asio::bind_executor(
          strand_,
          [this, rx_buf, &promise, done, timer](
            const boost::system::error_code & ec, std::size_t bytes_transferred) {
            if (done->exchange(true)) return;
            boost::system::error_code ignored;
            timer->cancel(ignored);
            if (ec == boost::asio::error::operation_aborted) {
              promise.set_value(std::make_optional<AxiomaticAdapter::socket_error_string_t>(
                "Receive operation timed out"));
              return;
            }
            if (ec) {
              promise.set_value(std::make_optional<AxiomaticAdapter::socket_error_string_t>(
                "Receive operation failed: " + ec.message()));
              return;
            }
            parser_.append(rx_buf->data(), bytes_transferred);
            promise.set_value(std::nullopt);
          }));

      timer->async_wait(boost::asio::bind_executor(
        strand_, [this, &promise, done](const boost::system::error_code & ec) {
          if (ec) return;
          if (done->exchange(true)) return;
          boost::system::error_code ignored;
          tcp_socket_.cancel(ignored);
          promise.set_value(std::make_optional<AxiomaticAdapter::socket_error_string_t>(
            "Receive operation timed out"));
        }));
    });

    auto err = future.get();
    if (err) return err;

    if (auto frame = drainParser()) {
      can_frame = *frame;
      return std::nullopt;
    }
    // Bytes arrived but not yet a complete frame; reception loop ignores this.
    return std::optional<AxiomaticAdapter::socket_error_string_t>(INCOMPLETE_FRAME_SENTINEL);
  }

  std::optional<const polymath::socketcan::CanFrame> receive()
  {
    polymath::socketcan::CanFrame can_frame = polymath::socketcan::CanFrame();
    auto result = receive(can_frame);
    return !result ? std::optional<const polymath::socketcan::CanFrame>(can_frame) : std::nullopt;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> send(const polymath::socketcan::CanFrame & frame)
  {
    auto frame_data = frame.get_data();
    auto frame_data_length = frame.get_len();
    size_t control_timestamp_byte_length = 3;

    size_t frame_id_byte_length;
    bool is_extended = false;
    if (frame.get_id_type() == polymath::socketcan::IdType::EXTENDED) {
      frame_id_byte_length = 4;
      is_extended = true;
    } else {
      frame_id_byte_length = 2;
    }

    size_t message_length = frame_data_length + control_timestamp_byte_length + frame_id_byte_length;
    unsigned char control_byte = (1 << 6);
    control_byte |= (is_extended ? (1 << 4) : 0);
    control_byte |= (frame_data_length & 0x0F);

    // Build the protocol envelope: 6-byte SYNC_PREFIX, Message ID = 1
    // (deprecated CAN Stream), Message Version 0, Message Data Length, then
    // the body (control byte + timestamp + CAN ID + data).
    auto full_message = std::make_shared<std::vector<uint8_t>>();
    full_message->insert(
      full_message->end(), AxiomaticFrameParser::SYNC_PREFIX.begin(),
      AxiomaticFrameParser::SYNC_PREFIX.end());
    full_message->push_back(
      static_cast<uint8_t>(AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED & 0xFF));
    full_message->push_back(
      static_cast<uint8_t>((AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED >> 8) & 0xFF));
    full_message->push_back(0x00);  // Message Version
    full_message->push_back(static_cast<uint8_t>(message_length & 0xFF));
    full_message->push_back(static_cast<uint8_t>((message_length >> 8) & 0xFF));
    full_message->push_back(control_byte);
    full_message->push_back(192);
    full_message->push_back(70);

    auto can_id = frame.get_id();
    for (size_t i = 0; i < frame_id_byte_length; ++i) {
      full_message->push_back(static_cast<uint8_t>((can_id >> (i * 8)) & 0xFF));
    }
    full_message->insert(
      full_message->end(), frame_data.begin(), frame_data.begin() + frame_data_length);

    std::promise<std::optional<AxiomaticAdapter::socket_error_string_t>> promise;
    auto future = promise.get_future();

    boost::asio::post(strand_, [this, full_message, &promise]() {
      boost::asio::async_write(
        tcp_socket_, boost::asio::buffer(*full_message),
        boost::asio::bind_executor(
          strand_,
          [full_message, &promise](
            const boost::system::error_code & ec, std::size_t /*bytes*/) {
            if (ec) {
              promise.set_value(std::make_optional<AxiomaticAdapter::socket_error_string_t>(
                std::string("TCP Send Failed: ") + ec.message()));
            } else {
              promise.set_value(std::nullopt);
            }
          }));
    });

    return future.get();
  }

  TCPSocketState get_socket_state()
  {
    return socket_state_;
  }

  bool is_thread_running()
  {
    return reception_active_.load();
  }

private:
  static constexpr std::chrono::milliseconds TCP_IP_CONNECTION_TIMEOUT_MS{3000};

  // Sentinel error string returned by receive() when bytes arrived but a full
  // CAN frame is not yet assembled. The reception loop checks this by value
  // and continues without firing the user's error callback.
  static inline const std::string INCOMPLETE_FRAME_SENTINEL{"__axiomatic_incomplete_frame__"};

  std::optional<polymath::socketcan::CanFrame> drainParser()
  {
    // The parser is touched from the io thread (append from async_receive
    // handler) and from sync receive() callers (tryParseFrame). Serialize
    // here so the std::deque inside the parser is never accessed concurrently.
    std::lock_guard<std::mutex> lock(parser_mutex_);
    return parser_.tryParseFrame();
  }

  void scheduleAsyncReceive()
  {
    if (stop_thread_requested_.load()) {
      reception_active_ = false;
      return;
    }
    auto rx_buf = std::make_shared<std::vector<uint8_t>>(1024, 0);
    tcp_socket_.async_receive(
      boost::asio::buffer(*rx_buf),
      boost::asio::bind_executor(
        strand_,
        [this, rx_buf](const boost::system::error_code & ec, std::size_t bytes_transferred) {
          if (stop_thread_requested_.load()) {
            reception_active_ = false;
            return;
          }
          if (ec == boost::asio::error::operation_aborted) {
            // Likely a cancel from joinReceptionThread or closeSocket — stop.
            reception_active_ = false;
            return;
          }
          if (ec) {
            error_callback_("Receive operation failed: " + ec.message());
          } else {
            {
              std::lock_guard<std::mutex> lock(parser_mutex_);
              parser_.append(rx_buf->data(), bytes_transferred);
            }
            while (auto frame = drainParser()) {
              receive_callback_(std::make_unique<polymath::socketcan::CanFrame>(*frame));
            }
          }
          scheduleAsyncReceive();
        }));
  }

  boost::asio::io_context tcp_io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  boost::asio::strand<boost::asio::io_context::executor_type> strand_;
  boost::asio::ip::tcp::socket tcp_socket_;
  TCPSocketState socket_state_{TCPSocketState::CLOSED};

  std::thread io_thread_;
  std::atomic<bool> reception_active_{false};
  std::atomic<bool> stop_thread_requested_{false};

  AxiomaticFrameParser parser_;
  std::mutex parser_mutex_;

  // from construction
  std::string ip_address_;
  std::string port_;
  std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> receive_callback_;
  std::function<void(AxiomaticAdapter::socket_error_string_t error)> error_callback_;
  std::chrono::milliseconds receive_timeout_ms_;
};

AxiomaticAdapter::AxiomaticAdapter(
  const std::string & ip_address,
  const std::string & port,
  const std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> && receive_callback_function,
  const std::function<void(AxiomaticAdapter::socket_error_string_t error)> && error_callback_function,
  const std::chrono::milliseconds & receive_timeout_ms,
  bool verbose)
: pimpl_(std::make_unique<AxiomaticAdapterImpl>(
    ip_address, port, std::move(receive_callback_function), std::move(error_callback_function),
    receive_timeout_ms, verbose))
{}

AxiomaticAdapter::~AxiomaticAdapter()
{
  closeSocket();
}

bool AxiomaticAdapter::openSocket()
{
  return pimpl_->openSocket();
}

bool AxiomaticAdapter::closeSocket()
{
  return pimpl_->closeSocket();
}

bool AxiomaticAdapter::startReceptionThread()
{
  return pimpl_->startReceptionThread();
}

bool AxiomaticAdapter::joinReceptionThread(const std::chrono::milliseconds & timeout_s)
{
  return pimpl_->joinReceptionThread(timeout_s);
}

std::optional<AxiomaticAdapter::socket_error_string_t> AxiomaticAdapter::receive(
  polymath::socketcan::CanFrame & can_frame)
{
  return pimpl_->receive(can_frame);
}

std::optional<const polymath::socketcan::CanFrame> AxiomaticAdapter::receive()
{
  return pimpl_->receive();
}

std::optional<AxiomaticAdapter::socket_error_string_t> AxiomaticAdapter::send(
  const polymath::socketcan::CanFrame & frame)
{
  return pimpl_->send(frame);
}

std::optional<AxiomaticAdapter::socket_error_string_t> AxiomaticAdapter::send(const can_frame & frame)
{
  return send(polymath::socketcan::CanFrame(frame));
}

TCPSocketState AxiomaticAdapter::get_socket_state()
{
  return pimpl_->get_socket_state();
}

bool AxiomaticAdapter::is_thread_running()
{
  return pimpl_->is_thread_running();
}

}  // namespace can
}  // namespace polymath
