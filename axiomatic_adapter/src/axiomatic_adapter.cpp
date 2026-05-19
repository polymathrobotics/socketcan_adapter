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
#include <condition_variable>
#include <deque>
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
    const std::chrono::milliseconds & receive_timeout_ms)
  : tcp_io_context_()
  , tcp_socket_(tcp_io_context_)
  , ip_address_(ip_address)
  , port_(port)
  , receive_callback_(receive_callback_function)
  , error_callback_(error_callback_function)
  , receive_timeout_ms_(receive_timeout_ms)
  {}

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

      // Disable Nagle's algorithm. With the async send worker, the calling
      // thread is never blocked on write() latency, and we'd rather have small
      // frequent TCP segments than bursty coalescing — the device's delayed-ACK
      // behaviour otherwise produces tens-to-hundreds of milliseconds of
      // segment stalls under sustained CAN traffic. Failure is non-fatal.
      {
        boost::system::error_code nd_ec;
        tcp_socket_.set_option(boost::asio::ip::tcp::no_delay(true), nd_ec);
        if (nd_ec) {
          std::cerr << "[Axiomatic] Failed to set TCP_NODELAY: " << nd_ec.message() << std::endl;
        }
      }

      socket_state_ = TCPSocketState::OPEN;
      startSendWorker();
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
      stopSendWorker();

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
        } else if (*error == AxiomaticAdapter::NON_FRAME_PROTOCOL_MESSAGE) {
          // Heartbeat / status response / other non-CAN-Stream protocol traffic
          // was consumed but yielded no CAN frame to deliver. Not a real error.
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
      // Use std::async to wait asynchronously for the thread to stop
      std::future<void> join_future = std::async(std::launch::async, [this] { tcp_receive_thread_.join(); });

      // Wait for the thread to stop within the timeout period
      return join_future.wait_for(timeout_s) == std::future_status::ready;
    }

    return false;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame)
  {
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

    // --- Process the received data ---
    //
    // Walk the buffer looking for an Axiomatic protocol message we can decode
    // into a CAN frame. Any non-CAN-Stream messages we encounter at the front
    // (heartbeats, status responses, CAN FD stream, unknown IDs) are skipped
    // using their declared Message Data Length so a CAN frame that follows in
    // the same TCP read isn't dropped. If the buffer contains only non-frame
    // protocol traffic, return the NON_FRAME_PROTOCOL_MESSAGE sentinel — the
    // reception thread treats this as "no frame this round" rather than a
    // socket error.
    size_t pos = 0;
    while (pos + HEADER_BYTES <= data.size()) {
      if (!std::equal(AXIOMATIC_SYNC_PREFIX.begin(), AXIOMATIC_SYNC_PREFIX.end(), data.begin() + pos)) {
        return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Not a valid Axiomatic message.");
      }

      uint16_t msg_id = static_cast<uint16_t>(data[pos + 6]) | (static_cast<uint16_t>(data[pos + 7]) << 8);
      uint16_t decl_data_len = static_cast<uint16_t>(data[pos + 9]) | (static_cast<uint16_t>(data[pos + 10]) << 8);
      size_t msg_total = static_cast<size_t>(HEADER_BYTES) + decl_data_len;

      if (msg_id != MSG_ID_CAN_STREAM_DEPRECATED) {
        // Consume this non-CAN-Stream message and look for the next one.
        // Declared length matches on-wire footprint for heartbeats / status
        // responses per the protocol spec (verified by hardware capture for
        // heartbeats specifically). If we'd overrun the buffer, advance to
        // the end so the outer loop exits cleanly.
        pos += (msg_total <= data.size() - pos) ? msg_total : (data.size() - pos);
        continue;
      }

      // CAN Stream (Message ID 1). Decode using the Control Byte at byte 11.
      uint8_t control_byte = data[pos + 11];
      size_t timestamp_size = (control_byte & 0x60) >> 5;
      bool is_can_extended = (control_byte & 0x10) >> 4;
      size_t can_length = control_byte & 0x0F;

      size_t can_id_start = pos + 12 + timestamp_size;
      size_t min_id_size = is_can_extended ? 4 : 2;
      if (data.size() < can_id_start + min_id_size) {
        return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Data too short for CAN ID.");
      }

      uint32_t can_id = 0;
      size_t can_data_start = 0;
      if (!is_can_extended) {
        can_id = static_cast<uint16_t>(data[can_id_start] | (data[can_id_start + 1] << 8));
        can_data_start = can_id_start + 2;
      } else {
        can_id = static_cast<uint32_t>(
          data[can_id_start] | (data[can_id_start + 1] << 8) | (data[can_id_start + 2] << 16) |
          (data[can_id_start + 3] << 24));
        can_data_start = can_id_start + 4;
        can_frame.set_id_as_extended();
      }

      if (data.size() < can_data_start + can_length) {
        return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Data too short for CAN payload.");
      }

      std::array<uint8_t, 8> can_data = {0};
      std::copy_n(data.begin() + can_data_start, can_length, can_data.begin());

      can_frame.set_can_id(can_id);
      can_frame.set_len(can_length);
      can_frame.set_data(can_data);

      return std::nullopt;
    }

    // We exhausted the buffer without ever finding a CAN Stream message.
    // Either pos < HEADER_BYTES of remaining bytes (insufficient to parse a
    // header) or we walked past everything skipping non-CAN-Stream messages.
    if (data.size() < HEADER_BYTES) {
      return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Data too short for header and control byte.");
    }
    return std::make_optional<AxiomaticAdapter::socket_error_string_t>(AxiomaticAdapter::NON_FRAME_PROTOCOL_MESSAGE);
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

    // initialize the full message with the header, control bytes, timestamp bytes.
    // Header layout: 6-byte sync prefix + 2-byte Message ID (LSB first) + 1-byte
    // Message Version + 2-byte Message Data Length (LSB first) = 11 bytes total.
    std::vector<uint8_t> full_message;
    full_message.insert(full_message.end(), AXIOMATIC_SYNC_PREFIX.begin(), AXIOMATIC_SYNC_PREFIX.end());
    full_message.push_back(static_cast<uint8_t>(MSG_ID_CAN_STREAM_DEPRECATED & 0xFF));
    full_message.push_back(static_cast<uint8_t>((MSG_ID_CAN_STREAM_DEPRECATED >> 8) & 0xFF));
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
    // insert the can frame data
    full_message.insert(full_message.end(), frame_data.begin(), frame_data.end());

    // Hand off to the send worker. Returns immediately so the calling thread —
    // typically the SocketCAN reception callback — is never blocked on a slow
    // or back-pressured TCP write. Actual write errors are reported through
    // error_callback_ from the worker thread.
    return enqueueSend(std::move(full_message));
  }

  TCPSocketState get_socket_state()
  {
    return socket_state_;
  }

  bool is_thread_running()
  {
    return thread_running_;
  }

  /// @brief Number of outbound messages dropped because the send queue was full.
  /// Reset to zero when the send worker starts.
  uint64_t get_send_queue_drops()
  {
    return send_queue_drops_.load();
  }

private:
  // ------ Async send worker --------------------------------------------------
  //
  // send() copies the prebuilt protocol message bytes onto a bounded queue and
  // returns immediately. A dedicated worker thread pops messages off the queue
  // and does the actual blocking boost::asio::write() against the socket.
  //
  // This decouples the SocketCAN reception callback (which calls send()) from
  // TCP back-pressure — under load the calling thread no longer waits on the
  // kernel send buffer, so frames don't pile up in the SocketCAN kernel queue
  // and get silently dropped. The cost is bounded user-space memory for the
  // outbound queue and best-effort delivery semantics: actual write errors
  // surface via error_callback_, not via send()'s return value.

  std::optional<AxiomaticAdapter::socket_error_string_t> enqueueSend(std::vector<uint8_t> bytes)
  {
    if (socket_state_ != TCPSocketState::OPEN) {
      return std::make_optional<AxiomaticAdapter::socket_error_string_t>(
        "axiomatic: send called while socket not open");
    }
    {
      std::lock_guard<std::mutex> lock(send_queue_mutex_);
      if (send_queue_.size() >= SEND_QUEUE_MAX) {
        // Drop the oldest queued frame. Matches the kernel SocketCAN buffer's
        // overflow semantics in spirit — under sustained back-pressure we keep
        // the freshest data and shed the stalest.
        send_queue_.pop_front();
        send_queue_drops_.fetch_add(1, std::memory_order_relaxed);
      }
      send_queue_.push_back(std::move(bytes));
    }
    send_queue_cv_.notify_one();
    return std::nullopt;
  }

  void startSendWorker()
  {
    send_worker_stop_.store(false);
    send_queue_drops_.store(0);
    send_worker_thread_ = std::thread([this]() { sendWorkerLoop(); });
  }

  void stopSendWorker()
  {
    {
      std::lock_guard<std::mutex> lock(send_queue_mutex_);
      send_worker_stop_.store(true);
    }
    send_queue_cv_.notify_all();
    if (send_worker_thread_.joinable()) {
      send_worker_thread_.join();
    }
    // Drop anything that didn't make it onto the wire so a future openSocket()
    // doesn't replay stale frames.
    std::lock_guard<std::mutex> lock(send_queue_mutex_);
    send_queue_.clear();
  }

  void sendWorkerLoop()
  {
    while (true) {
      std::vector<uint8_t> message;
      {
        std::unique_lock<std::mutex> lock(send_queue_mutex_);
        send_queue_cv_.wait(lock, [this]() { return send_worker_stop_.load() || !send_queue_.empty(); });
        if (send_worker_stop_.load() && send_queue_.empty()) {
          return;
        }
        message = std::move(send_queue_.front());
        send_queue_.pop_front();
      }
      try {
        boost::asio::write(tcp_socket_, boost::asio::buffer(message.data(), message.size()));
      } catch (const std::exception & e) {
        if (error_callback_) {
          error_callback_(std::string("TCP Send Failed: ") + e.what());
        }
      }
    }
  }

private:
  // 6-byte sync prefix common to every Axiomatic protocol message: "AXIO" + Protocol ID 14010
  // (= 0x36BA, little-endian on the wire as 0xBA 0x36). Message ID lives at bytes 6-7 of the
  // header and is checked separately so we can distinguish CAN Stream from Heartbeat / Status
  // Response / CAN FD Stream / unknown.
  static constexpr std::array<uint8_t, 6> AXIOMATIC_SYNC_PREFIX = {'A', 'X', 'I', 'O', 0xBA, 0x36};

  // Message IDs from the Axiomatic protocol spec (v6, April 2025).
  static constexpr uint16_t MSG_ID_CAN_STREAM_DEPRECATED = 1;
  static constexpr uint16_t MSG_ID_STATUS_RESPONSE = 3;
  static constexpr uint16_t MSG_ID_HEARTBEAT = 4;
  static constexpr uint16_t MSG_ID_CAN_FD_STREAM = 5;

  // Minimum header bytes needed before we can read Message Data Length (bytes 9-10).
  static constexpr size_t HEADER_BYTES = 11;

  static constexpr std::chrono::milliseconds TCP_IP_CONNECTION_TIMEOUT_MS{3000};

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

  // Async send worker state.
  static constexpr size_t SEND_QUEUE_MAX = 1024;
  std::mutex send_queue_mutex_;
  std::condition_variable send_queue_cv_;
  std::deque<std::vector<uint8_t>> send_queue_;
  std::thread send_worker_thread_;
  std::atomic<bool> send_worker_stop_{false};
  std::atomic<uint64_t> send_queue_drops_{0};

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
  const std::chrono::milliseconds & receive_timeout_ms)
: pimpl_(std::make_unique<AxiomaticAdapterImpl>(
    ip_address, port, std::move(receive_callback_function), std::move(error_callback_function), receive_timeout_ms))
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
