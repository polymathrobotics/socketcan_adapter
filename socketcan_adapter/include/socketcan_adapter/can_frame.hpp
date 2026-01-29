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

#ifndef SOCKETCAN_ADAPTER__CAN_FRAME_HPP_
#define SOCKETCAN_ADAPTER__CAN_FRAME_HPP_

#include <linux/can.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace polymath::socketcan
{

/// @brief Can Frame Types to help passthrough into the CAN Frame
enum class FrameType : uint8_t
{
  DATA = 0U,
  ERROR,
  REMOTE
};  // enum class FrameType

/// @brief Can Frame ID Type, Standard or Extended
enum class IdType : uint8_t
{
  STANDARD = 0U,
  EXTENDED
};  // enum class IdType

///
/// @class polymath::socketcan::CanFrame
/// @brief CanFrame is a wrapper around the linux/can.h struct can_frame that adds some
/// additional functionalility
///
class CanFrame
{
public:
  /// @brief Construct Empty CanFrame
  CanFrame();

  /// @brief Construct CanFrame directly from a can_frame structure
  /// @param frame
  explicit CanFrame(const struct can_frame & frame);

  /// @brief Initialize CanFrame with raw_id and data defined in std::array
  /// @param raw_id Can ID already generated including error and data information
  /// @param data std::array<unsigned char, CAN_MAX_DLC> enforces C++ std::array for it's core API
  /// @param timestamp uint64_t utc timestamp for the can frame to store
  CanFrame(
    const canid_t raw_id,
    const std::array<unsigned char, CAN_MAX_DLC> & data,
    const uint64_t & bus_timestamp,
    uint8_t len = CAN_MAX_DLC);

  /// @brief Initialize CanFrame with a partial ID and data defined in std::array
  /// @param raw_id Can ID already generated including error and data information
  /// @param data std::array<unsigned char, CAN_MAX_DLC> enforces C++ std::array for it's core API
  /// @param timestamp uint64_t utc timestamp for the can frame to store
  /// @param frame_type DATA, ERROR or REMOTE
  /// @param frame_id_type EXTENDED or Standard
  CanFrame(
    const canid_t id,
    const std::array<unsigned char, CAN_MAX_DLC> & data,
    const uint64_t & bus_timestamp,
    FrameType & frame_type,
    IdType & frame_id_type,
    uint8_t len = CAN_MAX_DLC);

  /// @brief ~CanFrame is a default destructor for now
  ~CanFrame() = default;

  /// @brief Set the internal frame to some predefined value
  /// @param frame INPUT c type can_frame structure
  /// @return Whether the frame was successfully set
  bool set_frame(const struct can_frame & frame);

  /// @brief Set the CAN ID (does not set EFF, ERR, RTR)
  /// @param id INPUT can_id
  void set_can_id(const canid_t & id);

  /// @brief Set bit for extended ID
  void set_id_as_extended();

  /// @brief Set bit for standard ID
  void set_id_as_standard();

  /// @brief Set bit for error ID (overwrites remote)
  void set_type_error();

  /// @brief Set bit for remote ID (overwrites error id)
  void set_type_remote();

  /// @brief Set bit to specify data frame (overwrites remote and error id)
  void set_type_data();

  /// @brief Set can dlc if we don't want to send the entire array
  void set_len(unsigned char len);

  /// @brief Set data if we have an array available
  /// @param data INPUT raw data to pass through the socket
  void set_data(const std::array<unsigned char, CAN_MAX_DLC> & data);

  /// @brief Set bus timestamp
  /// @param timestamp INPUT time as uint64_t from the bus
  void set_bus_timestamp(const uint64_t & timestamp);

  /// @brief Set receive timestamp
  /// @param timestamp INPUT time as uint64_t from the bus
  void set_receive_timestamp(const uint64_t & timestamp);

  /// @brief Set timestamp
  /// @param timestamp INPUT time as std::chrono::system_clock::time_point from the bus
  void set_bus_timestamp(const std::chrono::system_clock::time_point & timestamp);

  /// @brief Set timestamp
  /// @param timestamp INPUT time as std::chrono::steady_clock::time_point from the adapter
  void set_receive_timestamp(const std::chrono::steady_clock::time_point & timestamp);

  /// @brief Get bus time
  /// @return returns std::chrono::system_clock::time_point for when the frame was received on the bus
  std::chrono::system_clock::time_point get_bus_time() const;

  /// @brief Get receive time
  /// @return returns std::chrono::steady_clock::time_point for when the frame was received by the adapter
  std::chrono::steady_clock::time_point get_receive_time() const;

  /// @brief Get frame type
  /// @return returns polymath::socketcan::FrameType
  FrameType get_frame_type() const;

  /// @brief Get frame id type
  /// @return returns polymath::socketcan::IdType
  IdType get_id_type() const;

  /// @brief Get frame id
  /// @return canid_t frame id
  canid_t get_id() const;

  /// @brief Get length set for the can_frame
  /// @return unsigned char defining the length
  unsigned char get_len() const;

  /// @brief Get a subset of can errors as a string
  /// @return std::string detailing error parsed from an error frame
  std::string get_error() const;

  /// @brief Get a part of the frame id that is most relevant to us
  /// @param bit_length INPUT number of bits to return
  /// @param bit_offset INPUT offset of the bits to return
  /// @return Masked and shifted can id
  canid_t get_masked_id(uint8_t bit_length, uint8_t bit_offset) const;

  /// @brief Get a copy of raw frame
  /// @return frame saved inside returned as a copy
  struct can_frame get_frame() const;

  /// @brief Get the raw data
  /// @return std::array<usnigned char, CAN_MAX_DLC> raw data
  const std::array<unsigned char, CAN_MAX_DLC> get_data() const;

  /// TODO: Do we need data packing and unpacking, or do we leave that somewhere else?
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/5

private:
  struct can_frame frame_{};
  std::chrono::steady_clock::time_point receive_time_{};
  std::chrono::system_clock::time_point bus_time_{};
};

}  // namespace polymath::socketcan

#endif  // SOCKETCAN_ADAPTER__CAN_FRAME_HPP_
