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

#ifndef SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_
#define SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_

#include <linux/can.h>
#include <poll.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath::socketcan
{

/// @brief State of socket, error, open or closed
enum class SocketState
{
  ERROR = -1,
  OPEN = 0,
  CLOSED = 1,
};

/// @brief FilterMode for how to add filters
enum class FilterMode
{
  /// TODO: Add additional filtermodes for "add" and "remove"
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/6
  OVERWRITE = 0,
};

constexpr std::chrono::duration<float> SOCKET_RECEIVE_TIMEOUT_S = std::chrono::duration<float>(0.1);
constexpr std::chrono::duration<float> JOIN_RECEPTION_TIMEOUT_S = std::chrono::duration<float>(0.1);
constexpr int32_t CLOSED_SOCKET_VALUE = -1;
constexpr nfds_t NUM_SOCKETS_IN_ADAPTER = 1;

/// @brief J1939 CAN ID bit layout constants
constexpr uint32_t J1939_PGN_SHIFT = 8;
constexpr uint32_t J1939_PF_SHIFT = 16;
constexpr uint32_t J1939_PF_MASK = 0xFF;
constexpr uint32_t J1939_PDU2_THRESHOLD = 0xF0;
/// @brief Masks priority (3 bits) and source address (8 bits), matches full PGN (18 bits)
constexpr uint32_t J1939_PGN_FULL_MASK = 0x03FFFF00;
/// @brief Masks priority, source address, and PDU Specific (destination addr in PDU1)
constexpr uint32_t J1939_PGN_PDU1_MASK = 0x03FF0000;

/// @brief Convert a J1939 PGN to a CAN filter suitable for use with setFilters()
/// Handles PDU1 (PF < 0xF0) vs PDU2 (PF >= 0xF0) masking automatically:
/// - PDU2: PS is Group Extension, part of the PGN — all 18 PGN bits are matched
/// - PDU1: PS is destination address, not part of the PGN — only 10 bits are matched
/// @param pgn The PGN value (18-bit, e.g. 0xFEF1)
/// @return can_filter with appropriate can_id and can_mask set
static inline struct can_filter j1939PgnToFilter(const uint32_t pgn)
{
  const uint8_t pf = static_cast<uint8_t>((pgn >> J1939_PGN_SHIFT) & J1939_PF_MASK);

  struct can_filter filter {};
  filter.can_id = (pgn << J1939_PGN_SHIFT) | CAN_EFF_FLAG;

  if (J1939_PDU2_THRESHOLD <= pf) {
    filter.can_mask = J1939_PGN_FULL_MASK | CAN_EFF_FLAG;
  } else {
    filter.can_mask = J1939_PGN_PDU1_MASK | CAN_EFF_FLAG;
  }

  return filter;
}

/// @class polymath::socketcan::SocketcanAdapter
/// @brief Creates and manages a socketcan instance and simplifies the interface.
/// Generally does not throw, but returns booleans to tell you success
class SocketcanAdapter : public std::enable_shared_from_this<SocketcanAdapter>
{
public:
  /// @brief Mapped to std lib, but should be remapped to Polymath Safety compatible versions
  using socket_error_string_t = std::string;
  using filter_vector_t = std::vector<struct can_filter>;

  /// @brief SocketcanAdapter Class Init
  /// @param interface_name Interface for the socket to initialize on
  SocketcanAdapter(
    const std::string & interface_name,
    const std::chrono::duration<float> & receive_timeout_s = SOCKET_RECEIVE_TIMEOUT_S);

  /// @brief Destructor for SocketcanAdapter
  virtual ~SocketcanAdapter();

  /// @brief Open Socket
  /// @return bool successfully opened socket
  bool openSocket();

  /// @brief Close Socket
  /// @return bool successfully closed socket
  bool closeSocket();

  /// @brief Set a number of filters, vectorized
  /// @param filters reference to a vector of can filters to set for the socket
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> setFilters(
    const filter_vector_t & filters, FilterMode mode = FilterMode::OVERWRITE);

  /// Shared ptr to a vector is technically more efficient than a vector of shared_ptrs
  /// TODO: Vectors are harder to justify in MISRA, so might want to use Array
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/7
  /// @brief Set a number of filters, vectorized
  /// @param filters INPUT shared ptr to a vector of can filters to set for the socket
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> setFilters(
    const std::shared_ptr<filter_vector_t> filters, FilterMode mode = FilterMode::OVERWRITE);

  /// @brief Set the error mask
  /// @param error_mask INPUT error maskfor the socket to pass through
  /// @return optional string containing error information
  std::optional<socket_error_string_t> setErrorMaskOverwrite(const can_err_mask_t & error_mask);

  /// @brief Set the socket to Join Filters and do a logical AND instead of OR
  /// @param error_mask INPUT whether to set JOINED FILTERS
  /// @return optional string containing error information
  std::optional<socket_error_string_t> setJoinOverwrite(const bool & join);

  /// @brief Receive with a reference to a CanFrame to fill
  /// @param frame OUTPUT CanFrame to fill
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> receive(CanFrame & can_frame);

  /// TODO: Switch to unique ptr
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  /// @brief Receive with a reference to a CanFrame to fill
  /// @param frame OUTPUT CanFrame to fill via shared_ptr
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> receive(std::shared_ptr<CanFrame> frame);

  /// @brief Receive returns the received CanFrame
  /// @return optional CanFrame that was received by reference
  /// nullopt is returned if no canframe received, acts like null
  std::optional<const CanFrame> receive();

  /// @brief Start a reception thread (calls callback)
  /// @return success on started
  bool startReceptionThread();

  /// @brief Stop and join reception thread
  /// @param timeout_s INPUT timeout in seconds, <=0 means no timeout
  /// @return success on closed and joined thread
  bool joinReceptionThread(const std::chrono::duration<float> & timeout_s = JOIN_RECEPTION_TIMEOUT_S);

  /// @brief Set receive callback function if thread is used
  /// @param callback_function INPUT To be called on receipt of a can frame
  /// @return success on receive callback set
  bool setOnReceiveCallback(std::function<void(std::unique_ptr<const CanFrame> frame)> && callback_function);

  /// @brief Set receive callback function if thread is used
  /// @param callback_function INPUT To be called on receipt of a can frame
  /// @return success on error callback set
  bool setOnErrorCallback(std::function<void(socket_error_string_t error)> && callback_function);

  /// @brief Transmit a can frame via socket
  /// @param frame INPUT const reference to the frame
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const CanFrame & frame);

  /// @brief Transmit a can frame via socket
  /// @param frame INPUT shared_ptr to frame. Convert non-const to const by doing
  /// ```c++
  /// std::shared_ptr<CanFrame> frame = std::make_shared<CanFrame>();
  /// // do work on frame then lock the frame as const before sending
  /// std::shared_ptr<const CanFrame> frame = frame
  /// ```
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const std::shared_ptr<const CanFrame> frame);

  /// @brief Transmit a can frame via socket
  /// @param frame Linux CAN frame to send
  /// @return optional error string filled with an error message if any
  std::optional<socket_error_string_t> send(const can_frame & frame);

  /// @brief Set receive timeout
  /// @param reecive_timeout_s std::chrono::duration<float> sets receive timeout in seconds
  void set_receive_timeout(const std::chrono::duration<float> & recive_timeout_s);

  /// @brief Get state of socket
  /// @return SocketState data type detailing OPEN or CLOSED
  SocketState get_socket_state();

  /// @brief Get interface
  /// @return Can interface
  std::string get_interface();

  /// @brief Checks if the receive thread is running
  /// @return True if the thread is running, false otherwise
  bool is_thread_running();

private:
  /// @brief Wraps C level socket operations to set can_filter frames
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendFilters();

  /// @brief Wraps C level socket operations to set error mask frames
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendErrorMask();

  /// @brief Wraps C level socket operations to set whether to join filters
  /// @return error string if failure, nullopt otherwise
  std::optional<socket_error_string_t> sendJoin();

  std::string interface_name_;
  std::chrono::duration<float> receive_timeout_s_;

  int32_t socket_file_descriptor_;

  std::function<void(std::unique_ptr<const CanFrame> frame)> receive_callback_unique_ptr_;
  std::function<void(socket_error_string_t error)> receive_error_callback_;
  std::atomic<bool> thread_running_;
  std::atomic<bool> stop_thread_requested_;
  SocketState socket_state_;

  std::thread can_receive_thread_;

  /// @brief Store the filter list in case we want to be able to add, subtract in addition to overriding
  filter_vector_t filter_list_;
  can_err_mask_t error_mask_;
  bool join_;
};

}  // namespace polymath::socketcan

#endif  // SOCKETCAN_ADAPTER__SOCKETCAN_ADAPTER_HPP_
