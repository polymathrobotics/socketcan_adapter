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

#include <atomic>
#include <deque>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/system/error_code.hpp>

namespace polymath::can
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
    bool tcp_nodelay,
    std::unique_ptr<const AxiomaticModel> model)
  : tcp_io_context_()
  , tcp_socket_(tcp_io_context_)
  , ip_address_(ip_address)
  , port_(port)
  , receive_callback_(receive_callback_function)
  , error_callback_(error_callback_function)
  , receive_timeout_ms_(receive_timeout_ms)
  , rx_buffer_(RECEIVE_BUFFER_SIZE, 0)
  , tcp_nodelay_(tcp_nodelay)
  , model_(std::move(model))
  {
    if (!model_) {
      throw std::invalid_argument("AxiomaticAdapter requires a model");
    }
  }

  ~AxiomaticAdapterImpl()
  {
    joinReceptionThread();
    closeSocket();
  }

  bool openSocket()
  {
    try {
      boost::asio::ip::tcp::resolver resolver(tcp_io_context_);
      auto endpoints = resolver.resolve(ip_address_, port_);

      boost::asio::steady_timer timer(tcp_io_context_);
      timer.expires_after(TCP_IP_CONNECTION_TIMEOUT_MS);

      TCPSocketConnectionState connection_state{false, boost::asio::error::would_block};
      std::mutex connection_state_mutex;

      // asynchronously attempt to connect
      boost::asio::async_connect(
        tcp_socket_, endpoints, [&](const boost::system::error_code & error, const boost::asio::ip::tcp::endpoint &) {
          std::lock_guard<std::mutex> guard(connection_state_mutex);
          connection_state.error_code = error;
          connection_state.connected = !error;
          // cancel timeout if connected successfully
          timer.cancel();
        });

      // set up a timer to cancel the operation if it exceeds the timeout
      timer.async_wait([&](const boost::system::error_code & error) {
        if (!error) {
          std::lock_guard<std::mutex> guard(connection_state_mutex);
          if (!connection_state.connected) {
            connection_state.error_code = boost::asio::error::timed_out;
            tcp_socket_.cancel();
          }
        }
      });

      // run the I/O context to handle events
      tcp_io_context_.restart();
      tcp_io_context_.run();

      // capture the error message and connection state
      boost::system::error_code captured_error;
      bool is_connected;
      {
        std::lock_guard<std::mutex> guard(connection_state_mutex);
        captured_error = connection_state.error_code;
        is_connected = connection_state.connected;
      }

      if (captured_error || !is_connected) {
        std::cerr << "Connection failed: " << captured_error.message() << std::endl;
        socket_state_ = TCPSocketState::ERROR;
        return false;
      }
      // optionally disable Nagle's algorithm so each CAN frame becomes its
      // own TCP segment instead of being coalesced
      if (tcp_nodelay_) {
        boost::system::error_code nd_ec;
        tcp_socket_.set_option(boost::asio::ip::tcp::no_delay(true), nd_ec);
        if (nd_ec) {
          std::cerr << "[Axiomatic] Failed to set TCP_NODELAY: " << nd_ec.message() << std::endl;
        } else {
          std::cout << "[Axiomatic] TCP_NODELAY enabled (Nagle's algorithm disabled)" << std::endl;
        }
      } else {
        std::cout << "[Axiomatic] TCP_NODELAY disabled (Nagle's algorithm active — kernel may coalesce small writes)"
                  << std::endl;
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
    if (socket_state_ != TCPSocketState::CLOSED) {
      boost::system::error_code error_code;
      tcp_socket_.close(error_code);

      if (error_code) {
        std::cerr << "[ERROR] Failed to close TCP socket: " << error_code.message() << std::endl;
        return false;
      } else {
        socket_state_ = TCPSocketState::CLOSED;
        return true;
      }
    }
    return true;
  }

  bool startReceptionThread()
  {
    if (socket_state_ == TCPSocketState::CLOSED) {
      return false;
    }

    stop_thread_requested_ = false;
    thread_running_ = true;

    tcp_receive_thread_ = std::thread([this]() {
      while (!stop_thread_requested_) {
        polymath::socketcan::CanFrame frame = polymath::socketcan::CanFrame();
        std::optional<AxiomaticAdapter::socket_error_string_t> error = receive(frame);

        if (!error) {
          receive_callback_(std::make_unique<polymath::socketcan::CanFrame>(frame));
        } else {
          error_callback_(*error);
        }
      }

      thread_running_ = false;
    });

    return true;
  }

  bool joinReceptionThread(const std::chrono::milliseconds & timeout_s = AxiomaticAdapter::JOIN_RECEPTION_TIMEOUT_MS)
  {
    stop_thread_requested_ = true;

    if (tcp_receive_thread_.joinable()) {
      // use std::async to wait asynchronously for the thread to stop
      std::future<void> join_future = std::async(std::launch::async, [this] { tcp_receive_thread_.join(); });

      // wait for the thread to stop within the timeout period
      return join_future.wait_for(timeout_s) == std::future_status::ready;
    }

    return false;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame)
  {
    // A previous TCP read may have decoded several CAN frames out of
    // packed protocol messages — deliver those one at a time before doing
    // another network read, so packed frames don't get silently dropped
    if (!pending_frames_.empty()) {
      can_frame = pending_frames_.front();
      pending_frames_.pop_front();
      return std::nullopt;
    }

    size_t bytes_received = 0;
    std::atomic<bool> data_received(false);
    boost::system::error_code error_code;

    // set up the timer for timeout
    boost::asio::steady_timer timer(tcp_io_context_);
    timer.expires_after(receive_timeout_ms_);

    // start async receive operation
    tcp_socket_.async_receive(
      boost::asio::buffer(rx_buffer_.data(), RECEIVE_BUFFER_SIZE),
      [&](const boost::system::error_code & error, std::size_t bytes_transferred) {
        error_code = error;
        if (!error) {
          bytes_received = bytes_transferred;
          data_received = true;
        }
        timer.cancel();
      });

    // set up the timer to handle timeout cancellation
    timer.async_wait([&](const boost::system::error_code & error) {
      if (!error && !data_received.load()) {
        error_code = boost::asio::error::timed_out;
        // cancel the ongoing async receive operation on timeout (does not close socket)
        tcp_socket_.cancel();
      }
    });

    // run the I/O operations concurrently (this allows for new async operations in the future)
    tcp_io_context_.restart();
    tcp_io_context_.run();

    // check for timeout or other errors
    if (error_code == boost::asio::error::timed_out) {
      return std::optional<AxiomaticAdapter::socket_error_string_t>("Receive operation timed out");
    } else if (error_code) {
      return std::optional<AxiomaticAdapter::socket_error_string_t>(
        "Receive operation failed: " + error_code.message());
    }

    // NOTE: given the axiomatic documentation claims a deliberate 256-byte design + the large buffer size used in
    // rx_buffer_, a mid-tcp-message split is never supposed to occur. In testing this drop has never happened.
    // it is possible that with non-standard very low MTU's or in future revisions, this assumption no longer holds
    protocol::DecodeResult decoded = model_->decode(rx_buffer_.data(), bytes_received);
    for (const auto & diagnostic : decoded.diagnostics) {
      std::cerr << "[Axiomatic parser] " << diagnostic << std::endl;
    }
    if (decoded.frames.empty()) {
      return std::make_optional<AxiomaticAdapter::socket_error_string_t>(
        decoded.diagnostics.empty() ? "No CAN frames in received protocol traffic." : decoded.diagnostics.back());
    }

    pending_frames_.insert(pending_frames_.end(), decoded.frames.begin(), decoded.frames.end());
    can_frame = pending_frames_.front();
    pending_frames_.pop_front();
    return std::nullopt;
  }

  std::optional<const polymath::socketcan::CanFrame> receive()
  {
    polymath::socketcan::CanFrame can_frame = polymath::socketcan::CanFrame();
    auto result = receive(can_frame);
    return !result ? std::optional<const polymath::socketcan::CanFrame>(can_frame) : std::nullopt;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> send(const polymath::socketcan::CanFrame & frame)
  {
    const std::vector<uint8_t> full_message = model_->encode(frame);

    try {
      boost::asio::write(tcp_socket_, boost::asio::buffer(full_message.data(), full_message.size()));
    } catch (const std::exception & e) {
      return std::optional<AxiomaticAdapter::socket_error_string_t>(std::string("TCP Send Failed: ") + e.what());
    }
    return std::nullopt;
  }

  TCPSocketState get_socket_state()
  {
    return socket_state_;
  }

  bool is_thread_running()
  {
    return thread_running_;
  }

private:
  static constexpr std::chrono::milliseconds TCP_IP_CONNECTION_TIMEOUT_MS{3000};

  // receive buffer size for each async_receive call. Larger than the
  // protocol's per-message cap (256 bytes) by a wide margin
  static constexpr size_t RECEIVE_BUFFER_SIZE = 65536;

  /// @brief socket connection state as a struct for the mutex during TCP Open Socket to update the variables together
  struct TCPSocketConnectionState
  {
    bool connected{false};
    boost::system::error_code error_code{boost::asio::error::would_block};
  };

  boost::asio::io_context tcp_io_context_;
  boost::asio::ip::tcp::socket tcp_socket_;
  TCPSocketState socket_state_{TCPSocketState::CLOSED};

  std::thread tcp_receive_thread_;
  std::atomic<bool> thread_running_;
  std::atomic<bool> stop_thread_requested_;

  // CAN frames decoded from one TCP read but not yet delivered
  // through receive(). Drained one at a time, ahead of the next TCP read.
  std::deque<polymath::socketcan::CanFrame> pending_frames_;

  // from construction
  std::string ip_address_;
  std::string port_;
  std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> receive_callback_;
  std::function<void(AxiomaticAdapter::socket_error_string_t error)> error_callback_;
  std::chrono::milliseconds receive_timeout_ms_;

  // receive buffer for async_receive — allocated once at construction and reused across every receive() call
  std::vector<uint8_t> rx_buffer_;

  // when true, disable Nagle's algorithm on the TCP socket after connect
  bool tcp_nodelay_;

  std::unique_ptr<const AxiomaticModel> model_;
};

AxiomaticAdapter::AxiomaticAdapter(
  const std::string & ip_address,
  const std::string & port,
  const std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> && receive_callback_function,
  const std::function<void(AxiomaticAdapter::socket_error_string_t error)> && error_callback_function,
  const std::chrono::milliseconds & receive_timeout_ms,
  bool tcp_nodelay,
  std::unique_ptr<const AxiomaticModel> model)
: pimpl_(std::make_unique<AxiomaticAdapterImpl>(
    ip_address,
    port,
    std::move(receive_callback_function),
    std::move(error_callback_function),
    receive_timeout_ms,
    tcp_nodelay,
    std::move(model)))
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

}  // namespace polymath::can
