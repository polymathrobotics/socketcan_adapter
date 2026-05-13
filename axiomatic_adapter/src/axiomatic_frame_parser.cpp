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
#include <iostream>
#include <iterator>

namespace polymath
{
namespace can
{

namespace
{
// Bytes 0-5: SYNC_PREFIX. 6-7: Message ID. 8: Version. 9-10: Data Length.
constexpr size_t HEADER_SIZE = 11;
}  // namespace

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
  pending_frames_.clear();
}

std::optional<polymath::socketcan::CanFrame> AxiomaticFrameParser::tryParseFrame()
{
  while (true) {
    // Any frames decoded but not yet delivered take priority over fresh parsing.
    if (!pending_frames_.empty()) {
      auto frame = pending_frames_.front();
      pending_frames_.pop_front();
      return frame;
    }

    auto prefix_it =
      std::search(buffer_.begin(), buffer_.end(), SYNC_PREFIX.begin(), SYNC_PREFIX.end());

    if (prefix_it == buffer_.end()) {
      // No prefix in the buffer. The tail may still hold the start of a
      // prefix that completes in the next read; keep up to (SYNC_PREFIX-1)
      // bytes.
      size_t keep = std::min(buffer_.size(), SYNC_PREFIX.size() - 1);
      size_t drop = buffer_.size() - keep;
      if (drop > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + drop);
        dropped_bytes_ += drop;
      }
      return std::nullopt;
    }

    if (prefix_it != buffer_.begin()) {
      size_t drop = static_cast<size_t>(std::distance(buffer_.begin(), prefix_it));
      buffer_.erase(buffer_.begin(), prefix_it);
      dropped_bytes_ += drop;
    }

    if (buffer_.size() < HEADER_SIZE) {
      return std::nullopt;  // wait for the rest of the envelope
    }

    const uint16_t message_id =
      static_cast<uint16_t>(buffer_[6]) | (static_cast<uint16_t>(buffer_[7]) << 8);
    const uint16_t data_length =
      static_cast<uint16_t>(buffer_[9]) | (static_cast<uint16_t>(buffer_[10]) << 8);

    if (data_length > MAX_REASONABLE_DATA_LENGTH) {
      // The prefix we matched is almost certainly garbage that happened to
      // alias the sync bytes. Drop one byte and resync.
      buffer_.erase(buffer_.begin());
      dropped_bytes_ += 1;
      continue;
    }

    const size_t total_size = HEADER_SIZE + data_length;
    if (buffer_.size() < total_size) {
      return std::nullopt;  // wait for the full message body
    }

    switch (message_id) {
      case MSG_ID_CAN_STREAM_DEPRECATED: {
        if (!decodeCanStreamBody(buffer_.data() + HEADER_SIZE, data_length)) {
          // Body was malformed somewhere past the first frame. Frames that
          // did decode have already been queued. Count the rest as dropped.
          dropped_bytes_ += data_length;
        }
        break;
      }
      case MSG_ID_HEARTBEAT:
        heartbeats_seen_++;
        if (verbose_) {
          std::cout << "[AxiomaticFrameParser] heartbeat #" << heartbeats_seen_ << std::endl;
        }
        break;
      case MSG_ID_STATUS_RESPONSE:
        status_responses_seen_++;
        if (verbose_) {
          std::cout << "[AxiomaticFrameParser] status response #"
                    << status_responses_seen_ << std::endl;
        }
        break;
      case MSG_ID_CAN_FD_STREAM:
        can_fd_messages_skipped_++;
        if (verbose_) {
          std::cout << "[AxiomaticFrameParser] CAN FD message skipped #"
                    << can_fd_messages_skipped_ << " (FD unsupported)" << std::endl;
        }
        break;
      default:
        unknown_messages_skipped_++;
        if (verbose_) {
          std::cout << "[AxiomaticFrameParser] unknown message id=" << message_id
                    << " #" << unknown_messages_skipped_ << std::endl;
        }
        break;
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
    // Next loop iteration drains pending_frames_ or parses the next message.
  }
}

bool AxiomaticFrameParser::decodeCanStreamBody(const uint8_t * body, size_t body_len)
{
  size_t offset = 0;
  while (offset < body_len) {
    const uint8_t control = body[offset];

    // Bit 7 = 1 indicates a Notification Frame, which is a fixed 5-byte
    // record. We skip past it so the next iteration finds the following
    // CAN frame.
    if ((control & 0x80) != 0) {
      constexpr size_t NOTIFICATION_FRAME_SIZE = 5;
      if (offset + NOTIFICATION_FRAME_SIZE > body_len) {
        return false;  // truncated
      }
      notification_frames_skipped_++;
      if (verbose_) {
        std::cout << "[AxiomaticFrameParser] notification frame skipped #"
                  << notification_frames_skipped_ << std::endl;
      }
      offset += NOTIFICATION_FRAME_SIZE;
      continue;
    }

    const size_t timestamp_size = (control & 0x60) >> 5;
    const bool is_extended = (control & 0x10) != 0;
    const size_t can_length = control & 0x0F;

    if (can_length > CAN_MAX_DLC) {
      return false;
    }

    const size_t id_size = is_extended ? 4 : 2;
    const size_t frame_total = 1 + timestamp_size + id_size + can_length;
    if (offset + frame_total > body_len) {
      return false;
    }

    const size_t id_offset = offset + 1 + timestamp_size;
    canid_t can_id = 0;
    for (size_t i = 0; i < id_size; ++i) {
      can_id |= static_cast<canid_t>(body[id_offset + i]) << (i * 8);
    }

    const size_t data_offset = id_offset + id_size;
    std::array<unsigned char, CAN_MAX_DLC> can_data{};
    std::copy_n(body + data_offset, can_length, can_data.begin());

    polymath::socketcan::CanFrame frame;
    if (is_extended) {
      frame.set_id_as_extended();
    }
    frame.set_can_id(can_id);
    frame.set_len(static_cast<unsigned char>(can_length));
    frame.set_data(can_data);

    pending_frames_.push_back(frame);
    offset += frame_total;
  }
  return true;
}

}  // namespace can
}  // namespace polymath
