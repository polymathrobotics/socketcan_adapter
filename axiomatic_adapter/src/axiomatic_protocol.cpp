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

#include "axiomatic_adapter/axiomatic_protocol.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace polymath
{
namespace can
{
namespace protocol
{

namespace
{
constexpr int BITS_PER_BYTE = 8;
constexpr size_t PROTOCOL_ID_BYTES = 2;
constexpr size_t MESSAGE_DATA_LENGTH_BYTES = 2;
constexpr size_t MESSAGE_ID_BYTES = 2;
}  // namespace

uint32_t readLittleEndian(const uint8_t * data, size_t num_bytes)
{
  uint32_t value = 0;
  for (size_t i = 0; i < num_bytes; ++i) {
    value |= static_cast<uint32_t>(data[i]) << (BITS_PER_BYTE * i);
  }
  return value;
}

void appendLittleEndian(std::vector<uint8_t> & out, uint32_t value, size_t num_bytes)
{
  for (size_t i = 0; i < num_bytes; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (BITS_PER_BYTE * i)) & 0xFF));
  }
}

std::vector<uint8_t> encodeMessage(
  uint16_t protocol_id, uint16_t message_id, uint8_t message_version, const std::vector<uint8_t> & body)
{
  std::vector<uint8_t> message(AXIOMATIC_TAG.begin(), AXIOMATIC_TAG.end());
  message.reserve(HEADER_BYTES + body.size());
  appendLittleEndian(message, protocol_id, PROTOCOL_ID_BYTES);
  appendLittleEndian(message, message_id, MESSAGE_ID_BYTES);
  message.push_back(message_version);
  appendLittleEndian(message, static_cast<uint32_t>(body.size()), MESSAGE_DATA_LENGTH_BYTES);
  message.insert(message.end(), body.begin(), body.end());
  return message;
}

ParsedMessages parseMessages(uint16_t protocol_id, const uint8_t * data, size_t size)
{
  ParsedMessages parsed;
  if (size < HEADER_BYTES) {
    parsed.diagnostics.push_back(
      "DROP: received " + std::to_string(size) + " bytes, too short to contain a complete protocol header");
    return parsed;
  }

  size_t scan_pos = 0;
  while (scan_pos + HEADER_BYTES <= size) {
    const uint8_t * header = data + scan_pos;
    const bool tag_match = std::equal(AXIOMATIC_TAG.begin(), AXIOMATIC_TAG.end(), header);
    const uint16_t received_protocol_id =
      static_cast<uint16_t>(readLittleEndian(header + PROTOCOL_ID_OFFSET, PROTOCOL_ID_BYTES));
    if (!tag_match || protocol_id != received_protocol_id) {
      std::ostringstream message;
      message << "DROP: sync prefix mismatch at offset " << scan_pos << " of " << size
              << "-byte read; bytes there:" << std::hex;
      for (size_t i = 0; i < PROTOCOL_ID_OFFSET + PROTOCOL_ID_BYTES; ++i) {
        message << ' ' << static_cast<int>(header[i]);
      }
      message << std::dec << " (expected Protocol ID 0x" << std::hex << protocol_id << std::dec << "; remaining "
              << (size - scan_pos) << " bytes ignored)";
      parsed.diagnostics.push_back(message.str());
      return parsed;
    }

    const size_t declared_length = readLittleEndian(header + MESSAGE_DATA_LENGTH_OFFSET, MESSAGE_DATA_LENGTH_BYTES);
    const size_t body_start = scan_pos + HEADER_BYTES;
    const size_t body_end = std::min(body_start + declared_length, size);
    if (body_start + declared_length > size) {
      parsed.diagnostics.push_back(
        "TRUNCATED: message at offset " + std::to_string(scan_pos) + " declares " + std::to_string(declared_length) +
        " body bytes but only " + std::to_string(body_end - body_start) + " were received");
    }
    parsed.messages.push_back(MessageView{
      static_cast<uint16_t>(readLittleEndian(header + MESSAGE_ID_OFFSET, MESSAGE_ID_BYTES)),
      header[MESSAGE_VERSION_OFFSET],
      data + body_start,
      body_end - body_start});
    scan_pos = body_start + declared_length;
  }

  if (scan_pos < size) {
    parsed.diagnostics.push_back(
      "DROP: " + std::to_string(size - scan_pos) + " trailing bytes too short to contain a protocol header");
  }
  return parsed;
}

}  // namespace protocol
}  // namespace can
}  // namespace polymath
