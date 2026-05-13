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

#include "axiomatic_adapter/axiomatic_frame_parser.hpp"

#include <linux/can.h>

#include <algorithm>
#include <iterator>

namespace polymath
{
namespace can
{

void AxiomaticFrameParser::append(const uint8_t * data, size_t len)
{
  if (data == nullptr || len == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + len);
}

void AxiomaticFrameParser::reset()
{
  buffer_.clear();
}

std::optional<polymath::socketcan::CanFrame> AxiomaticFrameParser::tryParseFrame()
{
  // Header(7) + reserved(2) + length(2) + control(1) = 12 bytes before any
  // variable-length fields. We need at least this much to decode the control byte.
  constexpr size_t MIN_PREFIX_BYTES = HEADER.size() + 5;

  while (true) {
    // Locate the sync header. Anything before it is garbage to be skipped.
    auto header_it = std::search(buffer_.begin(), buffer_.end(), HEADER.begin(), HEADER.end());

    if (header_it == buffer_.end()) {
      // No header in buffer. The tail may still hold the start of a header
      // that will arrive in the next read, so keep up to HEADER.size()-1 bytes.
      size_t keep = std::min(buffer_.size(), HEADER.size() - 1);
      size_t drop = buffer_.size() - keep;
      if (drop > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + drop);
        dropped_bytes_ += drop;
      }
      return std::nullopt;
    }

    if (header_it != buffer_.begin()) {
      size_t drop = static_cast<size_t>(std::distance(buffer_.begin(), header_it));
      buffer_.erase(buffer_.begin(), header_it);
      dropped_bytes_ += drop;
    }

    if (buffer_.size() < MIN_PREFIX_BYTES) {
      return std::nullopt;
    }

    // Decode the control byte (byte 11): bits 6-5 = timestamp size, bit 4 =
    // extended ID flag, bits 3-0 = DLC. We derive the total frame size from
    // these fields directly (rather than the message_length at bytes 9-10),
    // so framing and per-field decoding can never disagree.
    const uint8_t control_byte = buffer_[11];
    const size_t timestamp_size = (control_byte & 0x60) >> 5;
    const bool is_extended = (control_byte & 0x10) != 0;
    const size_t can_length = control_byte & 0x0F;
    const size_t frame_id_byte_length = is_extended ? 4 : 2;

    // A DLC > 8 means the bytes we matched as a header are almost certainly
    // garbage that happened to alias the sync sequence. Drop one byte and
    // resync rather than consuming an invalid frame.
    if (can_length > CAN_MAX_DLC) {
      buffer_.erase(buffer_.begin());
      dropped_bytes_ += 1;
      continue;
    }

    const size_t total_frame_size =
      HEADER.size() + 5 + timestamp_size + frame_id_byte_length + can_length;

    if (buffer_.size() < total_frame_size) {
      return std::nullopt;
    }

    const size_t can_id_start = HEADER.size() + 5 + timestamp_size;
    canid_t can_id = 0;
    for (size_t i = 0; i < frame_id_byte_length; ++i) {
      can_id |= static_cast<canid_t>(buffer_[can_id_start + i]) << (i * 8);
    }
    const size_t can_data_start = can_id_start + frame_id_byte_length;

    std::array<unsigned char, CAN_MAX_DLC> can_data{};
    std::copy_n(buffer_.begin() + can_data_start, can_length, can_data.begin());

    polymath::socketcan::CanFrame frame;
    if (is_extended) {
      frame.set_id_as_extended();
    }
    frame.set_can_id(can_id);
    frame.set_len(static_cast<unsigned char>(can_length));
    frame.set_data(can_data);

    buffer_.erase(buffer_.begin(), buffer_.begin() + total_frame_size);
    return frame;
  }
}

}  // namespace can
}  // namespace polymath
