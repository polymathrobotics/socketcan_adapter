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
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
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
  , tcp_socket_(tcp_io_context_)
  , ip_address_(ip_address)
  , port_(port)
  , receive_callback_(receive_callback_function)
  , error_callback_(error_callback_function)
  , receive_timeout_ms_(receive_timeout_ms)
  {
    parser_.set_verbose(verbose);
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

      // Asynchronously attempt to connect
      boost::asio::async_connect(
        tcp_socket_, endpoints, [&](const boost::system::error_code & error, const boost::asio::ip::tcp::endpoint &) {
          std::lock_guard<std::mutex> guard(connection_state_mutex);
          connection_state.error_code = error;
          connection_state.connected = !error;
          // Cancel timeout if connected successfully
          timer.cancel();
        });

      // Set up a timer to cancel the operation if it exceeds the timeout
      timer.async_wait([&](const boost::system::error_code & error) {
        if (!error) {
          std::lock_guard<std::mutex> guard(connection_state_mutex);
          if (!connection_state.connected) {
            connection_state.error_code = boost::asio::error::timed_out;
            tcp_socket_.cancel();
          }
        }
      });

      // Run the I/O context to handle events
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
        } else if (*error != INCOMPLETE_FRAME_SENTINEL) {
          error_callback_(*error);
        }
        // INCOMPLETE_FRAME_SENTINEL: the read returned bytes but not yet a full
        // CAN frame. Loop again to read more; do not surface as a user error.
      }

      thread_running_ = false;
    });

    return true;
  }

  bool joinReceptionThread(const std::chrono::milliseconds & timeout_s = AxiomaticAdapter::JOIN_RECEPTION_TIMEOUT_MS)
  {
    stop_thread_requested_ = true;

    if (tcp_receive_thread_.joinable()) {
      // Use std::async to wait asynchronously for the thread to stop
      std::future<void> join_future = std::async(std::launch::async, [this] { tcp_receive_thread_.join(); });

      // Wait for the thread to stop within the timeout period
      return join_future.wait_for(timeout_s) == std::future_status::ready;
    }

    return false;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame)
  {
    // First drain anything left over in the parser from a previous TCP read.
    // This is what makes coalesced frames work: one read may yield N frames,
    // and the reception loop calls us back-to-back to deliver each one.
    if (auto frame = parser_.tryParseFrame()) {
      can_frame = *frame;
      return std::nullopt;
    }

    std::vector<uint8_t> data(1024, 0);
    std::atomic<bool> data_received(false);
    boost::system::error_code error_code;

    // Set up the timer for timeout
    boost::asio::steady_timer timer(tcp_io_context_);
    timer.expires_after(receive_timeout_ms_);

    // Start async receive operation
    tcp_socket_.async_receive(
      boost::asio::buffer(data), [&](const boost::system::error_code & error, std::size_t bytes_transferred) {
        error_code = error;
        if (!error) {
          data.resize(bytes_transferred);
          data_received = true;
        }
        timer.cancel();
      });

    // Set up the timer to handle timeout cancellation
    timer.async_wait([&](const boost::system::error_code & error) {
      if (!error && !data_received.load()) {
        error_code = boost::asio::error::timed_out;
        // Cancel the ongoing async receive operation on timeout (does not close socket)
        tcp_socket_.cancel();
      }
    });

    // Run the I/O operations concurrently (this allows for new async operations in the future)
    tcp_io_context_.restart();
    tcp_io_context_.run();

    // Check for timeout or other errors
    if (error_code == boost::asio::error::timed_out) {
      return std::optional<AxiomaticAdapter::socket_error_string_t>("Receive operation timed out");
    } else if (error_code) {
      return std::optional<AxiomaticAdapter::socket_error_string_t>(
        "Receive operation failed: " + error_code.message());
    }

    parser_.append(data.data(), data.size());

    if (auto frame = parser_.tryParseFrame()) {
      can_frame = *frame;
      return std::nullopt;
    }

    // Bytes arrived but not yet a complete frame. The reception loop treats
    // this sentinel as a no-op and will call receive() again immediately.
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

    // Determine the CAN frame ID length (extended or standard)
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
    std::vector<uint8_t> full_message;
    full_message.insert(
      full_message.end(), AxiomaticFrameParser::SYNC_PREFIX.begin(),
      AxiomaticFrameParser::SYNC_PREFIX.end());
    full_message.push_back(
      static_cast<uint8_t>(AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED & 0xFF));
    full_message.push_back(
      static_cast<uint8_t>((AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED >> 8) & 0xFF));
    full_message.push_back(0x00);  // Message Version
    full_message.push_back(static_cast<uint8_t>(message_length & 0xFF));
    full_message.push_back(static_cast<uint8_t>((message_length >> 8) & 0xFF));
    full_message.push_back(control_byte);
    full_message.push_back(192);
    full_message.push_back(70);

    // insert the can frame id
    auto can_id = frame.get_id();
    for (size_t i = 0; i < frame_id_byte_length; ++i) {
      full_message.push_back(static_cast<uint8_t>((can_id >> (i * 8)) & 0xFF));
    }
    // insert the can frame data — only the DLC-many bytes so that the declared
    // message_length matches the actual payload on the wire.
    full_message.insert(
      full_message.end(), frame_data.begin(), frame_data.begin() + frame_data_length);

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

  // Sentinel error string returned by receive() when bytes arrived but a full
  // CAN frame is not yet assembled. The reception loop checks this by value
  // and continues without firing the user's error callback.
  static inline const std::string INCOMPLETE_FRAME_SENTINEL{"__axiomatic_incomplete_frame__"};

  /// @brief Socket connection state as a struct for the mutex during TCP Open Socket to update the variables together
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

  AxiomaticFrameParser parser_;

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
