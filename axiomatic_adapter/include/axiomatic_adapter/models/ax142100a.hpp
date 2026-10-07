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

#include <cstdint>
#include <optional>
#include <vector>

#include "axiomatic_adapter/axiomatic_model.hpp"
#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

/// @brief AX142100A RS232-RS232-RS422-ENET-CAN converter, UMAX142100A section 4.2.
/// https://www.axiomatic.com/product/protocol-converter-ethernet-rs-422-2-rs-232-can-sae-j1939-ax142100a/
/// https://www.axiomatic.com/wp-content/uploads/UMAX142100A.pdf
namespace polymath::can::ax142100a
{

/// @brief Protocol ID 20008, 0x28 0x4E on the wire
static constexpr uint16_t PROTOCOL_ID = 0x4E28;

/// @brief Largest raw data payload one message can carry; the status byte takes one byte of the 16-bit body length
/// TODO: (David Tarazi) Confirm the device's own limit on hardware; the UMAX142100A example client uses a 256-byte buffer.
static constexpr size_t MAX_RAW_DATA_BYTES = 0xFFFF - 1;

/// @brief Message IDs (UMAX142100A section 4.2)
enum class MessageId : uint16_t
{
  Undefined = 0,
  ForwardedData = 1,
};

/// @class polymath::can::ax142100a::Ax142100a
/// @brief Encodes and decodes Forwarded Data messages carrying a CAN frame or raw data.
/// The device's data routing configuration selects which serial port raw data is forwarded to and from.
class Ax142100a : public AxiomaticModel
{
public:
  std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame) const override;

  /// @throws std::length_error when data exceeds MAX_RAW_DATA_BYTES
  std::optional<std::vector<uint8_t>> encodeRawData(const std::vector<uint8_t> & data) const override;

protected:
  uint16_t protocolId() const override;
  bool decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const override;
};

}  // namespace polymath::can::ax142100a

#endif  // AXIOMATIC_ADAPTER__MODELS__AX142100A_HPP_
