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

#ifndef AXIOMATIC_ADAPTER__MODELS__AX142100A_HPP_
#define AXIOMATIC_ADAPTER__MODELS__AX142100A_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

/// @brief AX142100A RS232-RS232-RS422-ENET-CAN converter, UMAX142100A section 4.2.
/// https://www.axiomatic.com/product/protocol-converter-ethernet-rs-422-2-rs-232-can-sae-j1939-ax142100a/
/// https://www.axiomatic.com/wp-content/uploads/UMAX142100A.pdf
namespace polymath
{
namespace can
{
namespace ax142100a
{

/// @brief Protocol ID 20008, 0x28 0x4E on the wire
static constexpr uint16_t PROTOCOL_ID = 0x4E28;

/// @brief Message IDs (UMAX142100A section 4.2)
enum class MessageId : uint16_t
{
  Undefined = 0,
  ForwardedData = 1,
};

/// @brief Encode one CAN frame as a Forwarded Data message
std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame);

/// @brief Decode every CAN frame in the Forwarded Data messages of a buffer.
/// Raw data payloads and other Message IDs are skipped.
/// @param data buffer start
/// @param size buffer length in bytes
protocol::DecodeResult decode(const uint8_t * data, size_t size);

}  // namespace ax142100a
}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__MODELS__AX142100A_HPP_
