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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_FRAME_PARSER_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_FRAME_PARSER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "socketcan_adapter/can_frame.hpp"

namespace polymath
{
namespace can
{

/// @class polymath::can::AxiomaticFrameParser
/// @brief Stateful framer for the Axiomatic CAN-over-TCP byte stream.
///
/// TCP is a byte stream, not a message stream: a single read may contain
/// multiple coalesced Axiomatic-encoded CAN frames, or only part of one.
/// This parser owns a persistent buffer of bytes read from the socket and
/// extracts complete CAN frames one at a time, leaving any trailing partial
/// bytes for the next read to complete.
class AxiomaticFrameParser
{
public:
  /// The 7-byte sync header that prefixes every Axiomatic message.
  static constexpr std::array<uint8_t, 7> HEADER = {'A', 'X', 'I', 'O', 0xBA, 0x36, 0x01};

  AxiomaticFrameParser() = default;
  ~AxiomaticFrameParser() = default;

  /// @brief Append raw bytes from a TCP read into the internal buffer.
  /// @param data pointer to the start of the freshly-read bytes
  /// @param len number of bytes to append
  void append(const uint8_t * data, size_t len);

  /// @brief Try to extract one complete CAN frame from the buffered bytes.
  /// @return populated CanFrame and consumes its bytes if one is available;
  ///         std::nullopt if more bytes are still needed to form a frame.
  ///
  /// If garbage bytes precede a valid header, they are silently skipped and
  /// counted in dropped_bytes().
  std::optional<polymath::socketcan::CanFrame> tryParseFrame();

  /// @brief Discard all buffered bytes (e.g. on reconnect).
  void reset();

  /// @brief How many bytes are currently buffered awaiting parse.
  size_t buffered_bytes() const { return buffer_.size(); }

  /// @brief Cumulative count of bytes skipped while resyncing on a header.
  /// Useful for diagnostics; large values indicate framing problems upstream.
  uint64_t dropped_bytes() const { return dropped_bytes_; }

private:
  std::vector<uint8_t> buffer_;
  uint64_t dropped_bytes_{0};
};

}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_FRAME_PARSER_HPP_
