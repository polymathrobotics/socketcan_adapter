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

#include "axiomatic_adapter/models/ax140900.hpp"

#include <algorithm>
#include <array>
#include <sstream>
#include <string>
#include <vector>

namespace polymath::can::ax140900
{

namespace
{
constexpr uint8_t MESSAGE_VERSION = 0;

constexpr size_t STANDARD_CAN_ID_BYTES = 2;
constexpr size_t EXTENDED_CAN_ID_BYTES = 4;
constexpr size_t CONTROL_BYTE_BYTES = 1;

// Control Byte (CB), first byte of every CAN or Notification frame in a CAN Stream body:
//   bit  7   : C_Bit   — 0 = CAN Frame, 1 = Notification Frame
//   bits 6:5 : TS_Bit  — Time Stamp length code, see TIMESTAMP_LENGTH_BYTES_TABLE
//   bit  4   : EID_Bit — 0 = standard 11-bit ID, 1 = extended 29-bit ID
//   bits 3:0 : L_Bit   — CAN Data Length, 0..8
constexpr uint8_t CONTROL_BYTE_NOTIFICATION_FRAME_FLAG = 0x80;
constexpr uint8_t CONTROL_BYTE_TIMESTAMP_LENGTH_MASK = 0x60;
constexpr int CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT = 5;
constexpr uint8_t CONTROL_BYTE_EXTENDED_ID_FLAG = 0x10;
constexpr uint8_t CONTROL_BYTE_CAN_DATA_LENGTH_MASK = 0x0F;

// TS_Bit code → time stamp bytes following CB (spec table 4); code 3 is 4 bytes, not 3
constexpr std::array<size_t, 4> TIMESTAMP_LENGTH_BYTES_TABLE = {0, 1, 2, 4};

// encode() always writes a 2-byte time stamp of 0x46C0
constexpr uint8_t SEND_TIMESTAMP_CODE = 2;
constexpr uint16_t SEND_TIMESTAMP = 0x46C0;
constexpr size_t SEND_TIMESTAMP_BYTES = 2;

// Notification Frame: 1-byte NIDB + 4-byte NDB1..NDB4
constexpr size_t NOTIFICATION_FRAME_TOTAL_BYTES = 5;

std::string hexByte(uint8_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << static_cast<int>(value);
  return stream.str();
}

/// Appends every CAN frame in one CAN Stream message body to result
void decodeCanStream(const uint8_t * body, size_t body_size, protocol::DecodeResult & result)
{
  size_t walker = 0;
  while (walker + CONTROL_BYTE_BYTES <= body_size) {
    const uint8_t control_byte = body[walker];
    if (0 != (control_byte & CONTROL_BYTE_NOTIFICATION_FRAME_FLAG)) {
      result.diagnostics.push_back(
        "SKIP: notification frame (CB=" + hexByte(control_byte) + ") at body offset " + std::to_string(walker));
      walker += NOTIFICATION_FRAME_TOTAL_BYTES;
      continue;
    }
    const size_t timestamp_size = TIMESTAMP_LENGTH_BYTES_TABLE
      [(control_byte & CONTROL_BYTE_TIMESTAMP_LENGTH_MASK) >> CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT];
    const bool extended_id = 0 != (control_byte & CONTROL_BYTE_EXTENDED_ID_FLAG);
    const size_t id_size = extended_id ? EXTENDED_CAN_ID_BYTES : STANDARD_CAN_ID_BYTES;
    const size_t can_data_length = control_byte & CONTROL_BYTE_CAN_DATA_LENGTH_MASK;
    const size_t frame_bytes = CONTROL_BYTE_BYTES + timestamp_size + id_size + can_data_length;
    if (walker + frame_bytes > body_size) {
      result.diagnostics.push_back(
        "DROP: truncated CAN frame at body offset " + std::to_string(walker) + " (CB=" + hexByte(control_byte) +
        " declares " + std::to_string(frame_bytes) + " bytes but only " + std::to_string(body_size - walker) +
        " remain)");
      return;
    }
    const uint8_t * id_start = body + walker + CONTROL_BYTE_BYTES + timestamp_size;
    std::array<unsigned char, CAN_MAX_DLC> data_bytes = {0};
    /// TODO: (David Tarazi) Drop frames whose CAN data length exceeds CAN_MAX_DLC; up to 15 overflows data_bytes.
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

std::vector<uint8_t> Ax140900::encode(const polymath::socketcan::CanFrame & frame) const
{
  const bool extended_id = polymath::socketcan::IdType::EXTENDED == frame.get_id_type();
  const size_t data_length = frame.get_len();
  const auto data = frame.get_data();

  std::vector<uint8_t> body;
  body.push_back(static_cast<uint8_t>(
    (SEND_TIMESTAMP_CODE << CONTROL_BYTE_TIMESTAMP_LENGTH_SHIFT) | (extended_id ? CONTROL_BYTE_EXTENDED_ID_FLAG : 0) |
    (data_length & CONTROL_BYTE_CAN_DATA_LENGTH_MASK)));
  protocol::appendLittleEndian(body, SEND_TIMESTAMP, SEND_TIMESTAMP_BYTES);
  protocol::appendLittleEndian(body, frame.get_id(), extended_id ? EXTENDED_CAN_ID_BYTES : STANDARD_CAN_ID_BYTES);
  const size_t declared_body_size = body.size() + data_length;
  body.insert(body.end(), data.begin(), data.end());

  std::vector<uint8_t> message =
    protocol::encodeMessage(PROTOCOL_ID, static_cast<uint16_t>(MessageId::CanStream), MESSAGE_VERSION, body);
  /// TODO: (David Tarazi) Send only the frame's data length in bytes; the header declares that many, but all
  /// CAN_MAX_DLC data bytes are sent.
  message[protocol::MESSAGE_DATA_LENGTH_OFFSET] = static_cast<uint8_t>(declared_body_size & 0xFF);
  message[protocol::MESSAGE_DATA_LENGTH_OFFSET + 1] = static_cast<uint8_t>((declared_body_size >> 8) & 0xFF);
  return message;
}

uint16_t Ax140900::protocolId() const
{
  return PROTOCOL_ID;
}

bool Ax140900::decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const
{
  if (static_cast<uint16_t>(MessageId::CanStream) != message.message_id) {
    return false;
  }
  decodeCanStream(message.body, message.body_size, result);
  return true;
}

}  // namespace polymath::can::ax140900
