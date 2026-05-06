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

#ifndef SOCKETCAN_ADAPTER__I_CAN_BACKEND_HPP_
#define SOCKETCAN_ADAPTER__I_CAN_BACKEND_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath::socketcan
{

/// @brief State of the underlying transport (socket, TCP/UDP connection, etc).
/// SocketState was previously the SocketcanAdapter-only enum; lifting it here
/// so any backend (Linux SocketCAN, Axiomatic Ethernet/CAN, future variants)
/// reports its open/closed/error state through one type.
enum class SocketState;  // forward — defined in socketcan_adapter.hpp

/// @brief FilterMode for how to add filters
enum class FilterMode;   // forward — defined in socketcan_adapter.hpp

/// @class polymath::socketcan::ICanBackend
/// @brief Cross-cutting interface implemented by every CAN transport backend.
///
/// Carved out of SocketcanAdapter's public surface in 2026-05 to allow
/// non-SocketCAN backends (e.g. the Axiomatic AX140970 Dual CAN FD over
/// Ethernet converter) to plug into the same ROS2 bridge node without forking
/// the runtime path. Linux-specific knobs (CAN_RAW_FILTER vector, ERR mask,
/// JOIN flag) stay on the concrete SocketcanAdapter — they have no analog in
/// the Axiomatic protocol and forcing every backend to no-op them muddies
/// the contract.
///
/// All implementations must:
///   * be safe to construct without opening any sockets (open happens in
///     openSocket()),
///   * tolerate openSocket()/closeSocket() being called multiple times,
///   * deliver received frames to the on-receive callback set by
///     setOnReceiveCallback() once startReceptionThread() has been called.
class ICanBackend
{
public:
  /// @brief Mapped to std lib, but should be remapped to Polymath Safety compatible versions
  using socket_error_string_t = std::string;

  virtual ~ICanBackend() = default;

  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------

  /// @brief Open the underlying transport. Returns true on success.
  virtual bool openSocket() = 0;

  /// @brief Close the underlying transport. Returns true on success.
  virtual bool closeSocket() = 0;

  /// @brief Get the current transport state.
  virtual SocketState get_socket_state() = 0;

  // ---------------------------------------------------------------------------
  // Send / receive
  // ---------------------------------------------------------------------------

  /// @brief Transmit a frame.
  /// @param frame INPUT shared_ptr to a frame to send.
  /// @return optional error string if any.
  virtual std::optional<socket_error_string_t> send(
    const std::shared_ptr<const CanFrame> frame) = 0;

  /// @brief Transmit a frame.
  virtual std::optional<socket_error_string_t> send(const CanFrame & frame) = 0;

  /// @brief Block-receive a frame into `frame`. Subject to the receive timeout
  /// configured via the constructor / set_receive_timeout().
  /// @return optional error string if any.
  virtual std::optional<socket_error_string_t> receive(CanFrame & frame) = 0;

  // ---------------------------------------------------------------------------
  // Reception thread + callbacks
  // ---------------------------------------------------------------------------

  virtual bool startReceptionThread() = 0;

  virtual bool joinReceptionThread(
    const std::chrono::duration<float> & timeout_s) = 0;

  virtual bool is_thread_running() = 0;

  virtual bool setOnReceiveCallback(
    std::function<void(std::unique_ptr<const CanFrame> frame)> && callback) = 0;

  virtual bool setOnErrorCallback(
    std::function<void(socket_error_string_t error)> && callback) = 0;
};

}  // namespace polymath::socketcan

#endif  // SOCKETCAN_ADAPTER__I_CAN_BACKEND_HPP_
