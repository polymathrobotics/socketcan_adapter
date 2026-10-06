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

#include "axiomatic_adapter/models/ax142100a.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace polymath::can::ax142100a
{

namespace
{
constexpr uint8_t MESSAGE_VERSION = 0;

/// TODO: (David) Confirm the 11-bit ID width on hardware; UMAX142100A only shows a 29-bit example.
constexpr size_t STANDARD_CAN_ID_BYTES = 2;
constexpr size_t EXTENDED_CAN_ID_BYTES = 4;
constexpr size_t STATUS_BYTE_BYTES = 1;

// Status byte, first byte of the Forwarded Data payload:
//   bit  6   : raw data — the rest of the payload is unformatted serial/Ethernet data
//   bit  4   : 0 = standard 11-bit ID, 1 = extended 29-bit ID
//   bits 3:0 : CAN data length
constexpr uint8_t STATUS_BYTE_RAW_DATA_FLAG = 0x40;
constexpr uint8_t STATUS_BYTE_EXTENDED_ID_FLAG = 0x10;
constexpr uint8_t STATUS_BYTE_CAN_DATA_LENGTH_MASK = 0x0F;

/// Appends every CAN frame in one Forwarded Data message body to result
void decodeForwardedData(const uint8_t * body, size_t body_size, protocol::DecodeResult & result)
{
  size_t walker = 0;
  while (walker + STATUS_BYTE_BYTES <= body_size) {
    const uint8_t status_byte = body[walker];
    if (0 != (status_byte & STATUS_BYTE_RAW_DATA_FLAG)) {
      result.diagnostics.push_back(
        "SKIP: raw data payload of " + std::to_string(body_size - walker) + " bytes at body offset " +
        std::to_string(walker));
      return;
    }
    const bool extended_id = 0 != (status_byte & STATUS_BYTE_EXTENDED_ID_FLAG);
    const size_t id_size = extended_id ? EXTENDED_CAN_ID_BYTES : STANDARD_CAN_ID_BYTES;
    const size_t can_data_length = status_byte & STATUS_BYTE_CAN_DATA_LENGTH_MASK;
    if (can_data_length > CAN_MAX_DLC) {
      result.diagnostics.push_back(
        "DROP: CAN data length " + std::to_string(can_data_length) + " at body offset " + std::to_string(walker) +
        " exceeds " + std::to_string(CAN_MAX_DLC) + "; remainder of body dropped");
      return;
    }
    const size_t frame_bytes = STATUS_BYTE_BYTES + id_size + can_data_length;
    if (walker + frame_bytes > body_size) {
      result.diagnostics.push_back(
        "DROP: truncated CAN frame at body offset " + std::to_string(walker) + " (declares " +
        std::to_string(frame_bytes) + " bytes but only " + std::to_string(body_size - walker) + " remain)");
      return;
    }
    const uint8_t * id_start = body + walker + STATUS_BYTE_BYTES;
    std::array<unsigned char, CAN_MAX_DLC> data_bytes = {0};
    std::copy_n(id_start + id_size, can_data_length, data_bytes.begin());

    polymath::socketcan::CanFrame frame;
    frame.set_can_id(protocol::readLittleEndian(id_start, id_size));
    frame.set_len(static_cast<unsigned char>(can_data_length));
    frame.set_data(data_bytes);
    if (extended_id) {
      frame.set_id_as_extended();
    }
    result.frames.push_back(frame);
    walker += frame_bytes;
  }
}
}  // namespace

std::vector<uint8_t> Codec::encode(const polymath::socketcan::CanFrame & frame) const
{
  const bool extended_id = polymath::socketcan::IdType::EXTENDED == frame.get_id_type();
  const size_t data_length = std::min<size_t>(frame.get_len(), CAN_MAX_DLC);
  const auto data = frame.get_data();

  std::vector<uint8_t> body;
  body.push_back(static_cast<uint8_t>(
    (extended_id ? STATUS_BYTE_EXTENDED_ID_FLAG : 0) | (data_length & STATUS_BYTE_CAN_DATA_LENGTH_MASK)));
  protocol::appendLittleEndian(body, frame.get_id(), extended_id ? EXTENDED_CAN_ID_BYTES : STANDARD_CAN_ID_BYTES);
  body.insert(body.end(), data.begin(), data.begin() + data_length);

  return protocol::encodeMessage(PROTOCOL_ID, static_cast<uint16_t>(MessageId::ForwardedData), MESSAGE_VERSION, body);
}

uint16_t Codec::protocolId() const
{
  return PROTOCOL_ID;
}

bool Codec::decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const
{
  if (static_cast<uint16_t>(MessageId::ForwardedData) != message.message_id) {
    return false;
  }
  decodeForwardedData(message.body, message.body_size, result);
  return true;
}

}  // namespace polymath::can::ax142100a
