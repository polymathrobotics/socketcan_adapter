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

#include <cstdint>
#include <vector>

#include "axiomatic_adapter/axiomatic_codec.hpp"
#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

/// @brief AX140900 CAN/Ethernet Converter, "Ethernet to CAN Converter Communication Protocol" v6.
/// https://www.axiomatic.com/product/canethernet-converter-ax140900/
/// https://www.axiomatic.com/wp-content/uploads/Ethernet-to-CAN-Converter-Communication-Protocol.pdf
namespace polymath::can::ax140900
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

/// @class polymath::can::ax140900::Codec
/// @brief Encodes a single-frame CAN Stream message with a fixed 2-byte time stamp.
/// Decodes CAN Stream messages; notification frames are skipped.
class Codec : public AxiomaticCodec
{
public:
  std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame) const override;

protected:
  uint16_t protocolId() const override;
  bool decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const override;
};

}  // namespace polymath::can::ax140900

#endif  // AXIOMATIC_ADAPTER__MODELS__AX140900_HPP_
