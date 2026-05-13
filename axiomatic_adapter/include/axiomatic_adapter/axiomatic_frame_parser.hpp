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
#include <deque>
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
/// The Axiomatic protocol wraps every transmission in an 11-byte envelope:
///   bytes 0-3  : 'A','X','I','O' tag
///   bytes 4-5  : Protocol ID 0xBA 0x36
///   bytes 6-7  : Message ID (little-endian) — selects payload format
///   byte  8    : Message Version (currently always 0)
///   bytes 9-10 : Message Data Length (little-endian) — bytes that follow
/// Total wire size of a message is therefore 11 + data_length. Only the first
/// six bytes are constant across message types; the parser uses bytes 9-10 as
/// the canonical framing field so heartbeats, status responses, and CAN FD
/// streams can be skipped cleanly without losing alignment.
///
/// A single CAN Stream message may carry multiple CAN frames and Notification
/// frames packed sequentially. The parser decodes all CAN frames into an
/// internal queue and delivers them one per tryParseFrame() call so the
/// existing one-frame-per-call consumer continues to work.
class AxiomaticFrameParser
{
public:
  /// 6-byte constant prefix shared by every Axiomatic message type.
  static constexpr std::array<uint8_t, 6> SYNC_PREFIX = {'A', 'X', 'I', 'O', 0xBA, 0x36};

  /// Message IDs from the Axiomatic protocol spec.
  static constexpr uint16_t MSG_ID_CAN_STREAM_DEPRECATED = 1;
  static constexpr uint16_t MSG_ID_STATUS_RESPONSE = 3;
  static constexpr uint16_t MSG_ID_HEARTBEAT = 4;
  static constexpr uint16_t MSG_ID_CAN_FD_STREAM = 5;

  /// Sanity bound on the declared Message Data Length; anything larger almost
  /// certainly indicates the sync prefix matched garbage. Resync when exceeded.
  static constexpr uint16_t MAX_REASONABLE_DATA_LENGTH = 2048;

  AxiomaticFrameParser() = default;
  ~AxiomaticFrameParser() = default;

  /// Append raw bytes from a TCP read into the internal buffer.
  void append(const uint8_t * data, size_t len);

  /// Pop the next CAN frame from the parser. Returns nullopt if more bytes
  /// are required to assemble one. Non-CAN messages (heartbeat, status, etc.)
  /// are consumed and counted without ever surfacing through this method.
  std::optional<polymath::socketcan::CanFrame> tryParseFrame();

  /// Discard all buffered bytes and pending frames (e.g. on reconnect).
  void reset();

  /// When enabled, the parser prints one std::cout line per heartbeat,
  /// status response, FD-stream skip, unknown message, and notification frame
  /// it encounters. Off by default.
  void set_verbose(bool enabled) { verbose_ = enabled; }

  // -------- Diagnostics ----------------------------------------------------
  size_t buffered_bytes() const { return buffer_.size(); }
  uint64_t dropped_bytes() const { return dropped_bytes_; }
  uint64_t heartbeats_seen() const { return heartbeats_seen_; }
  uint64_t status_responses_seen() const { return status_responses_seen_; }
  uint64_t can_fd_messages_skipped() const { return can_fd_messages_skipped_; }
  uint64_t unknown_messages_skipped() const { return unknown_messages_skipped_; }
  uint64_t notification_frames_skipped() const { return notification_frames_skipped_; }

private:
  /// Decode the body of a CAN Stream message: a packed sequence of CAN frames
  /// (control byte bit 7 = 0) and Notification frames (bit 7 = 1, fixed 5 B).
  /// Pushes every decoded CAN frame onto pending_frames_. Returns true if the
  /// body parsed cleanly; false if a malformed control byte or truncated
  /// frame was encountered mid-iteration (caller still advances past the
  /// outer message using the protocol's data_length).
  bool decodeCanStreamBody(const uint8_t * body, size_t body_len);

  std::vector<uint8_t> buffer_;
  std::deque<polymath::socketcan::CanFrame> pending_frames_;

  bool verbose_{false};

  uint64_t dropped_bytes_{0};
  uint64_t heartbeats_seen_{0};
  uint64_t status_responses_seen_{0};
  uint64_t can_fd_messages_skipped_{0};
  uint64_t unknown_messages_skipped_{0};
  uint64_t notification_frames_skipped_{0};
};

}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_FRAME_PARSER_HPP_
