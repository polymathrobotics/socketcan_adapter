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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath::can::protocol
{

/// @brief Message Header framing shared by every Axiomatic Ethernet converter.
/// Wire layout, multi-byte fields LSB first:
///   bytes 0-3  : Axiomatic Tag "AXIO"
///   bytes 4-5  : Protocol ID (model specific)
///   bytes 6-7  : Message ID
///   byte  8    : Message Version
///   bytes 9-10 : Message Data Length (body bytes that follow the header)
static constexpr std::array<uint8_t, 4> AXIOMATIC_TAG = {'A', 'X', 'I', 'O'};
static constexpr size_t HEADER_BYTES = 11;
static constexpr size_t PROTOCOL_ID_OFFSET = 4;
static constexpr size_t MESSAGE_ID_OFFSET = 6;
static constexpr size_t MESSAGE_VERSION_OFFSET = 8;
static constexpr size_t MESSAGE_DATA_LENGTH_OFFSET = 9;

/// @brief One protocol message located inside a receive buffer
struct MessageView
{
  uint16_t message_id;
  uint8_t message_version;
  /// @brief points into the buffer passed to parseMessages; valid only while that buffer is
  const uint8_t * body;
  size_t body_size;
};

/// @brief Protocol messages found in a buffer, plus a description of every byte range skipped
struct ParsedMessages
{
  std::vector<MessageView> messages;
  std::vector<std::string> diagnostics;
  /// @brief Leading bytes parsed or dropped; the rest is an incomplete message to retry with more data
  size_t consumed{0};
};

/// @brief CAN frames and raw data decoded from a buffer, plus a description of every byte range skipped or dropped
struct DecodeResult
{
  std::vector<polymath::socketcan::CanFrame> frames;
  /// @brief Raw data payloads, in arrival order
  std::vector<std::vector<uint8_t>> raw_data;
  std::vector<std::string> diagnostics;
  /// @brief Leading bytes decoded or dropped; the rest is an incomplete message to retry with more data
  size_t consumed{0};
};

/// @brief Read an unsigned little-endian integer
/// @param data first byte, LSB
/// @param num_bytes field width, at most 4
uint32_t readLittleEndian(const uint8_t * data, size_t num_bytes);

/// @brief Append the low num_bytes of value, LSB first
void appendLittleEndian(std::vector<uint8_t> & out, uint32_t value, size_t num_bytes);

/// @brief Frame a message body with the Axiomatic Message Header
/// @param protocol_id model specific Protocol ID
/// @param message_id Message ID for the body type
/// @param message_version Message Version
/// @param body Message Data, at most 65535 bytes
/// @return header followed by body
std::vector<uint8_t> encodeMessage(
  uint16_t protocol_id, uint16_t message_id, uint8_t message_version, const std::vector<uint8_t> & body);

/// @brief Split a buffer into back-to-back protocol messages carrying protocol_id.
/// Scanning stops at the first header whose tag or Protocol ID does not match, and the rest of the buffer is consumed.
/// A trailing message or header that runs past the buffer is not returned or consumed.
/// @param protocol_id Protocol ID every message must carry
/// @param data buffer start
/// @param size buffer length in bytes
ParsedMessages parseMessages(uint16_t protocol_id, const uint8_t * data, size_t size);

}  // namespace polymath::can::protocol

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_
