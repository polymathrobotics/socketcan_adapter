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
      // Disable Nagle's algorithm. This removes batching TCP packets for low latency comms and
      // keeps one CAN frame per TCP message
      {
        boost::system::error_code nd_ec;
        tcp_socket_.set_option(boost::asio::ip::tcp::no_delay(true), nd_ec);
        if (nd_ec) {
          std::cerr << "[Axiomatic] Failed to set TCP_NODELAY: " << nd_ec.message() << std::endl;
        }
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
      // Use std::async to wait asynchronously for the thread to stop
      std::future<void> join_future = std::async(std::launch::async, [this] { tcp_receive_thread_.join(); });

      // Wait for the thread to stop within the timeout period
      return join_future.wait_for(timeout_s) == std::future_status::ready;
    }

    return false;
  }

  std::optional<AxiomaticAdapter::socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame)
  {
    // A previous TCP read may have decoded several CAN frames out of a single
    // packed CAN Stream message — deliver those one at a time before doing
    // another network read, so packed frames don't get silently dropped.
    if (!pending_frames_.empty()) {
      can_frame = pending_frames_.front();
      pending_frames_.pop_front();
      return std::nullopt;
    }

    std::vector<uint8_t> data(RECEIVE_BUFFER_SIZE, 0);
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
    // Walk the buffer starting at position 0, dispatching every Axiomatic
    // protocol message we find by its Message ID:
    //   - CAN Stream (ID 1): extract every CAN frame from its body into
    //     pending_frames_. This handles both frames packed inside one message
    //     and multiple CAN Stream messages coalesced into one TCP read.
    //   - Heartbeat / Status Response / CAN FD Stream / unknown ID: skip past
    //     the message using its declared Message Data Length. The frames are
    //     not surfaced to the caller, but trailing CAN frames in the same
    //     read are still recovered.
    //
    // We compare only the first 6 bytes of the header constant ("AXIO" +
    // 0xBA 0x36) so the sync match doesn't require Message ID = 1 like the
    // previous strict 7-byte check did. That earlier behavior would reject
    // an entire TCP read whose first protocol message happened to be a
    // heartbeat — including any CAN frames that followed it in the same
    // buffer. Loss of those trailing CAN frames was the most likely cause
    // of single-frame drops during UDS flashes (heartbeats land at the
    // front of a TCP read once per second on average, and a flash takes
    // long enough to make a coincidence with a critical response likely).
    if (data.size() < 11) {
      std::cerr << "[Axiomatic parser] DROP: received " << data.size()
                << " bytes, too short to contain a complete protocol header" << std::endl;
      return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Data too short for header.");
    }

    size_t scan_pos = 0;
    while (scan_pos + 11 <= data.size()) {
      if (!std::equal(
            AXIOMATIC_CAN_MESSAGE_HEADER.begin(), AXIOMATIC_CAN_MESSAGE_HEADER.begin() + 6, data.begin() + scan_pos))
      {
        std::cerr << "[Axiomatic parser] DROP: sync prefix mismatch at offset " << scan_pos << " of " << data.size()
                  << "-byte TCP read; first 6 bytes there: " << std::hex;
        for (size_t i = 0; i < 6 && scan_pos + i < data.size(); ++i) {
          std::cerr << ' ' << static_cast<int>(data[scan_pos + i]);
        }
        std::cerr << std::dec << " (stopping scan; remaining " << (data.size() - scan_pos)
                  << " bytes ignored — possible truncated message or partial TCP read)" << std::endl;
        break;
      }
      const uint16_t msg_id =
        static_cast<uint16_t>(data[scan_pos + 6]) | (static_cast<uint16_t>(data[scan_pos + 7]) << 8);
      const size_t decl_len = static_cast<size_t>(data[scan_pos + 9]) | (static_cast<size_t>(data[scan_pos + 10]) << 8);
      const size_t body_end = std::min<size_t>(scan_pos + 11 + decl_len, data.size());
      if (msg_id == 1) {  // CAN Stream (deprecated, but what V5.05 firmware emits)
        decodePackedCanFramesInto(data, scan_pos + 11, body_end);
      } else {
        std::cerr << "[Axiomatic parser] SKIP: non-CAN-Stream message (Message ID " << msg_id << ", " << decl_len
                  << "-byte body) at offset " << scan_pos << " — heartbeat/status/FD/unknown; not delivered to caller"
                  << std::endl;
      }
      scan_pos += 11 + decl_len;
    }

    if (pending_frames_.empty()) {
      // Either the buffer started with non-Axiomatic bytes (scan_pos still 0),
      // or it contained only non-CAN-Stream protocol traffic (heartbeats etc.).
      // Neither case is a real error; the default error_callback_ swallows it.
      if (scan_pos == 0) {
        return std::make_optional<AxiomaticAdapter::socket_error_string_t>("Not a valid Axiomatic message.");
      }
      return std::make_optional<AxiomaticAdapter::socket_error_string_t>("No CAN frames in received protocol traffic.");
    }

    can_frame = pending_frames_.front();
    pending_frames_.pop_front();
    return std::nullopt;
  }

  // walks the body of a single CAN Stream message and pushes every CAN frame onto pending_frames_
  void decodePackedCanFramesInto(const std::vector<uint8_t> & data, size_t body_start, size_t body_end)
  {
    size_t walker = body_start;
    while (walker + 1 <= body_end) {
      const uint8_t cb = data[walker];
      if ((cb & CONTROL_BYTE_NOTIFICATION_FRAME_FLAG) != 0) {
        std::cerr << "[Axiomatic parser] SKIP: notification frame (CB=0x" << std::hex << static_cast<int>(cb)
                  << std::dec << ") at offset " << walker << " — not delivered to caller" << std::endl;
        walker += NOTIFICATION_FRAME_TOTAL_BYTES;
        continue;
      }
      const size_t ts_size =
        TIMESTAMP_LENGTH_BYTES_TABLE[(cb & CONTROL_BYTE_TIMESTAMP_LENGTH_MASK) >> CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT];
      const bool ext_id = (cb & CONTROL_BYTE_EXTENDED_ID_FLAG) != 0;
      const size_t id_size = ext_id ? 4 : 2;
      const size_t dlc = cb & CONTROL_BYTE_CAN_DATA_LENGTH_MASK;
      const size_t frame_bytes = 1 + ts_size + id_size + dlc;
      if (walker + frame_bytes > body_end) {
        // truncated final frame — abandon rather than misdecode.
        std::cerr << "[Axiomatic parser] DROP: truncated CAN frame at offset " << walker << " (CB=0x" << std::hex
                  << static_cast<int>(cb) << std::dec << " declares " << frame_bytes << " bytes but only "
                  << (body_end - walker) << " bytes remain in message body) — frame and remainder dropped" << std::endl;
        break;
      }
      const size_t id_offset = walker + 1 + ts_size;
      uint32_t cid = 0;
      for (size_t i = 0; i < id_size; ++i) {
        cid |= static_cast<uint32_t>(data[id_offset + i]) << (8 * i);
      }
      std::array<uint8_t, 8> dbytes = {0};
      std::copy_n(data.begin() + id_offset + id_size, dlc, dbytes.begin());

      polymath::socketcan::CanFrame extra;
      extra.set_can_id(cid);
      extra.set_len(static_cast<unsigned char>(dlc));
      extra.set_data(dbytes);
      if (ext_id) {
        extra.set_id_as_extended();
      }
      pending_frames_.push_back(extra);
      walker += frame_bytes;
    }
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
    unsigned char control_byte = CONTROL_BYTE_TIMESTAMP_2_BYTE_VALUE;
    control_byte |= (is_extended ? CONTROL_BYTE_EXTENDED_ID_FLAG : 0);
    control_byte |= (frame_data_length & CONTROL_BYTE_CAN_DATA_LENGTH_MASK);

    // initialize the full message with the header, control bytes, timestamp bytes
    std::vector<uint8_t> full_message;
    full_message.insert(full_message.end(), AXIOMATIC_CAN_MESSAGE_HEADER.begin(), AXIOMATIC_CAN_MESSAGE_HEADER.end());
    full_message.push_back(0x00);
    full_message.push_back(0x00);
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
  static constexpr std::array<uint8_t, 7> AXIOMATIC_CAN_MESSAGE_HEADER = {'A', 'X', 'I', 'O', 0xBA, 0x36, 0x01};
  static constexpr std::chrono::milliseconds TCP_IP_CONNECTION_TIMEOUT_MS{3000};

  // Receive buffer size for each async_receive call. Larger than the
  // protocol's per-message cap (256 bytes) by a wide margin so that bursts
  // of protocol messages coalesced by the kernel into a single TCP read fit
  // comfortably without truncating any message mid-body. 64 KiB is below
  // the typical Linux TCP receive-buffer default (~85 KiB), so we drain
  // everything the kernel had buffered in a single call.
  static constexpr size_t RECEIVE_BUFFER_SIZE = 65536;

  // Control Byte (CB) field layout — first byte of every CAN/Notification
  // Frame inside a CAN Stream message body. Per Axiomatic Communication
  // Protocol spec v6, Section "Control Byte":
  //   bit  7   : C_Bit    — 0 = CAN Frame, 1 = Notification Frame
  //   bits 6:5 : TS_Bit   — Time Stamp length code (see TIMESTAMP_LENGTH_BYTES_TABLE)
  //   bit  4   : EID_Bit  — 0 = standard 11-bit ID, 1 = extended 29-bit ID
  //   bits 3:0 : L_Bit    — CAN Data Length (DLC), 0..8 valid
  static constexpr uint8_t CONTROL_BYTE_NOTIFICATION_FRAME_FLAG = 0x80;
  static constexpr uint8_t CONTROL_BYTE_TIMESTAMP_LENGTH_MASK = 0x60;
  static constexpr int CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT = 5;
  static constexpr uint8_t CONTROL_BYTE_EXTENDED_ID_FLAG = 0x10;
  static constexpr uint8_t CONTROL_BYTE_CAN_DATA_LENGTH_MASK = 0x0F;

  // TS_Bit code → number of timestamp bytes that follow CB. Per Table 4 in
  // the spec: 00→0 bytes, 01→1, 10→2, 11→4. Note this mapping is non-linear
  // at index 3 (which is why a lookup table is needed instead of using the
  // raw 2-bit value as the byte count directly).
  static constexpr size_t TIMESTAMP_LENGTH_BYTES_TABLE[4] = {0, 1, 2, 4};

  // TS_Bit code that send() writes into the Control Byte. We always include
  // a 2-byte timestamp slot on outbound; firmware tolerates whatever bytes
  // are there (spec says outbound TS is ignored by the converter).
  static constexpr uint8_t CONTROL_BYTE_TIMESTAMP_2_BYTE_VALUE = static_cast<uint8_t>(2)
                                                                 << CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT;

  // Notification Frame fixed size: 1-byte NIDB + 4-byte NDB1..NDB4.
  static constexpr size_t NOTIFICATION_FRAME_TOTAL_BYTES = 5;

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

  // CAN frames decoded from a packed CAN Stream message but not yet delivered
  // through receive(). Drained one at a time, ahead of the next TCP read.
  std::deque<polymath::socketcan::CanFrame> pending_frames_;

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
