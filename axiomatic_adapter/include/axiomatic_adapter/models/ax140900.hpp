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

#ifndef AXIOMATIC_ADAPTER__MODELS__AX140900_HPP_
#define AXIOMATIC_ADAPTER__MODELS__AX140900_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

/// @brief AX140900 CAN/Ethernet Converter, "Ethernet to CAN Converter Communication Protocol" v6.
/// https://www.axiomatic.com/product/canethernet-converter-ax140900/
/// https://www.axiomatic.com/wp-content/uploads/Ethernet-to-CAN-Converter-Communication-Protocol.pdf
namespace polymath
{
namespace can
{
namespace ax140900
{

/// @brief Protocol ID, 0xBA 0x36 on the wire
static constexpr uint16_t PROTOCOL_ID = 0x36BA;

/// @brief Message IDs (spec v6, table 3)
enum class MessageId : uint16_t
{
  Undefined = 0,
  CanStream = 1,
  StatusRequest = 2,
  StatusResponse = 3,
  Heartbeat = 4,
  CanFdStream = 5,
};

/// @brief Encode one CAN frame as a single-frame CAN Stream message with a fixed 2-byte time stamp
std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame);

/// @brief Decode every CAN frame packed in the CAN Stream messages of a buffer.
/// Notification frames and other Message IDs are skipped.
/// @param data buffer start
/// @param size buffer length in bytes
protocol::DecodeResult decode(const uint8_t * data, size_t size);

}  // namespace ax140900
}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__MODELS__AX140900_HPP_
