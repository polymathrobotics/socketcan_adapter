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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_ADAPTER_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_ADAPTER_HPP_

#include <linux/can.h>
#include <poll.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "axiomatic_adapter/axiomatic_model.hpp"
#include "axiomatic_adapter/models/ax140900.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::can
{

/// @brief State of TCP socket, error, open or closed
enum class TCPSocketState
{
  ERROR = -1,
  OPEN = 0,
  CLOSED = 1,
};

/// @class polymath::can::AxiomaticAdapter
/// @brief Creates and manages a tcp connection and simplifies the interface.
/// Generally does not throw, but returns booleans to tell you success
class AxiomaticAdapter : public std::enable_shared_from_this<AxiomaticAdapter>
{
public:
  /// @brief Mapped to std lib, but should be remapped to Polymath Safety compatible versions
  using socket_error_string_t = std::string;

  static constexpr std::chrono::milliseconds DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS{100};
  static constexpr std::chrono::milliseconds JOIN_RECEPTION_TIMEOUT_MS{100};

  /// @brief AxiomaticAdapter Class Init
  /// @param ip_address Axiomatic Device IP address to connect
  /// @param port Axiomatic Device Port to connect to
  /// @param receive_callback_function called for each CAN frame received from the converter
  /// @param error_callback_function called for socket errors and unrecoverable parser conditions
  /// @param receive_timeout_ms receive timeout in milliseconds
  /// @param tcp_nodelay when true (default), sets TCP_NODELAY on the socket after connect to
  ///                    disable Nagle's algorithm. This eliminates the device's delayed-ACK +
  ///                    Nagle interaction that produces 30-450 ms inter-segment stalls under
  ///                    sustained CAN traffic, at the cost of higher packet-per-second rate
  ///                    on the network (each small CAN frame becomes its own TCP segment with
  ///                    ~40 bytes of IP+TCP header overhead — meaning more interrupts/syscalls
  ///                    on both sides, more switch/NIC PPS load, and worse bytes-on-wire
  ///                    efficiency for bulk transfers). Better for real-time, but can, in constrained
  ///                    resource environments, cause issues
  /// @param model converter model, e.g. std::make_unique<ax142100a::Ax142100a>(); must not be null
  AxiomaticAdapter(
    const std::string & ip_address,
    const std::string & port,
    const std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> && receive_callback_function =
      [](std::unique_ptr<const polymath::socketcan::CanFrame> /*frame*/) { /*do nothing*/ },
    const std::function<void(socket_error_string_t error)> && error_callback_function =
      [](socket_error_string_t /*error*/) { /*do nothing*/ },
    const std::chrono::milliseconds & receive_timeout_ms = AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
    bool tcp_nodelay = true,
    std::unique_ptr<const AxiomaticModel> model = std::make_unique<ax140900::Ax140900>());

  /// @brief Destructor for AxiomaticAdapter
  virtual ~AxiomaticAdapter();

  /// @brief Open TCP Socket
  /// @return bool success for opening socket
  bool openSocket();

  /// @brief Close TCP Socket
  /// @return bool success for closing socket
  bool closeSocket();

  /// @brief Receive with a reference to a CanFrame to fill
  /// @param frame OUTPUT CanFrame to fill
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> receive(polymath::socketcan::CanFrame & can_frame);

  /// @brief Receive returns the received CanFrame
  /// @return optional CanFrame
  /// nullopt is returned if no frame received, acts like null
  std::optional<const polymath::socketcan::CanFrame> receive();

  /// @brief Start a reception thread (calls callback)
  /// @return success on started
  bool startReceptionThread();

  /// @brief Set the callback for raw (serial) data payloads.
  /// Called on the thread that read them: the reception thread, or the caller of receive().
  /// Unset drops raw data.
  /// @return false while the reception thread is running
  bool setOnRawDataCallback(std::function<void(std::vector<uint8_t> data)> && callback_function);

  /// @brief Stop and join reception thread
  /// @param timeout_s INPUT timeout in seconds, <=0 means no timeout
  /// @return success on closed and joined thread
  bool joinReceptionThread(const std::chrono::milliseconds & timeout_s = AxiomaticAdapter::JOIN_RECEPTION_TIMEOUT_MS);

  /// @brief Transmit a can frame via socket
  /// @param frame INPUT const reference to the frame
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const polymath::socketcan::CanFrame & frame);

  /// @brief Transmit a can frame via socket
  /// @param frame Linux CAN frame to send
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const can_frame & frame);

  /// @brief Transmit raw (serial) data via socket
  /// @param data payload, at most the model's raw data limit
  /// @return optional error string, set when the model has no raw data channel or the write fails
  std::optional<socket_error_string_t> sendRawData(const std::vector<uint8_t> & data);

  /// @brief Whether the model carries raw (serial) data
  bool supportsRawData() const;

  /// @brief Get state of socket
  /// @return TCPSocketState data type detailing OPEN or CLOSED
  TCPSocketState get_socket_state();

  /// @brief Checks if the receive thread is running
  /// @return True if the thread is running, false otherwise
  bool is_thread_running();

private:
  /// @brief use Implemention (pimpl) to avoid including boost/asio.hpp in header + linking in CMake
  class AxiomaticAdapterImpl;
  std::unique_ptr<AxiomaticAdapterImpl> pimpl_;
};
}  // namespace polymath::can

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_ADAPTER_HPP_
