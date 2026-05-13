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

#include <array>
#include <cstdint>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif

#include "socketcan_adapter/can_frame.hpp"

using polymath::can::AxiomaticFrameParser;
using polymath::socketcan::CanFrame;
using polymath::socketcan::IdType;

namespace
{

struct CanFrameSpec
{
  uint32_t can_id;
  std::vector<uint8_t> data;
  bool is_extended{false};
  size_t timestamp_size{2};  // bits 6-5 of control byte; valid values 0..3
};

// Encode a single in-message frame (control byte + timestamp + id + data).
std::vector<uint8_t> encodeCanFrameInBody(const CanFrameSpec & spec)
{
  std::vector<uint8_t> out;
  const size_t id_size = spec.is_extended ? 4 : 2;
  const uint8_t control =
    static_cast<uint8_t>(((spec.timestamp_size & 0x3) << 5) |
                         (spec.is_extended ? (1 << 4) : 0) |
                         (spec.data.size() & 0x0F));
  out.push_back(control);
  for (size_t i = 0; i < spec.timestamp_size; ++i) {
    out.push_back(0xAB);
  }
  for (size_t i = 0; i < id_size; ++i) {
    out.push_back(static_cast<uint8_t>((spec.can_id >> (i * 8)) & 0xFF));
  }
  out.insert(out.end(), spec.data.begin(), spec.data.end());
  return out;
}

// Encode a 5-byte Notification Frame placeholder (control bit 7 = 1).
std::vector<uint8_t> encodeNotificationFrameInBody()
{
  return {0x80, 0x00, 0x00, 0x00, 0x00};
}

// Wrap an arbitrary message body in the 11-byte Axiomatic envelope.
std::vector<uint8_t> encodeMessage(uint16_t message_id, const std::vector<uint8_t> & body)
{
  std::vector<uint8_t> out;
  for (auto b : AxiomaticFrameParser::SYNC_PREFIX) {
    out.push_back(b);
  }
  out.push_back(static_cast<uint8_t>(message_id & 0xFF));
  out.push_back(static_cast<uint8_t>((message_id >> 8) & 0xFF));
  out.push_back(0x00);  // Message Version
  const uint16_t data_length = static_cast<uint16_t>(body.size());
  out.push_back(static_cast<uint8_t>(data_length & 0xFF));
  out.push_back(static_cast<uint8_t>((data_length >> 8) & 0xFF));
  out.insert(out.end(), body.begin(), body.end());
  return out;
}

// Convenience: a CAN Stream message carrying a single CAN frame.
std::vector<uint8_t> encodeCanStreamMessage(const CanFrameSpec & spec)
{
  return encodeMessage(
    AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED, encodeCanFrameInBody(spec));
}

// CAN Stream carrying an arbitrary sequence of CAN and Notification frames.
// items: positive value => index into `frames`; -1 => insert a Notification.
std::vector<uint8_t> encodeCanStreamMixed(
  const std::vector<CanFrameSpec> & frames,
  const std::vector<int> & layout)
{
  std::vector<uint8_t> body;
  for (int item : layout) {
    if (item < 0) {
      auto n = encodeNotificationFrameInBody();
      body.insert(body.end(), n.begin(), n.end());
    } else {
      auto f = encodeCanFrameInBody(frames[item]);
      body.insert(body.end(), f.begin(), f.end());
    }
  }
  return encodeMessage(AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED, body);
}

std::vector<uint8_t> encodeHeartbeatMessage()
{
  // The Axiomatic Heartbeat has a payload describing converter health, but the
  // parser does not introspect it -- any payload of declared length works.
  std::vector<uint8_t> body(8, 0x00);
  return encodeMessage(AxiomaticFrameParser::MSG_ID_HEARTBEAT, body);
}

std::vector<uint8_t> encodeStatusResponseMessage()
{
  std::vector<uint8_t> body(4, 0xAA);
  return encodeMessage(AxiomaticFrameParser::MSG_ID_STATUS_RESPONSE, body);
}

std::vector<uint8_t> encodeCanFdMessage()
{
  std::vector<uint8_t> body(16, 0x55);
  return encodeMessage(AxiomaticFrameParser::MSG_ID_CAN_FD_STREAM, body);
}

std::vector<uint8_t> encodeUnknownMessage(uint16_t id)
{
  std::vector<uint8_t> body(3, 0xEE);
  return encodeMessage(id, body);
}

}  // namespace

TEST_CASE("Parser: empty buffer", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.buffered_bytes() == 0);
  REQUIRE(parser.dropped_bytes() == 0);
}

TEST_CASE("Parser: single standard CAN frame in a CAN Stream message", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  auto wire = encodeCanStreamMessage({0x123, data, false, 2});

  parser.append(wire.data(), wire.size());
  auto frame = parser.tryParseFrame();

  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x123u);
  REQUIRE(frame->get_id_type() == IdType::STANDARD);
  REQUIRE(static_cast<size_t>(frame->get_len()) == data.size());
  auto got = frame->get_data();
  for (size_t i = 0; i < data.size(); ++i) {
    REQUIRE(got[i] == data[i]);
  }
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
}

TEST_CASE("Parser: single extended CAN frame", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0xAA, 0xBB, 0xCC};
  const uint32_t can_id = 0x1ABCDEF;
  auto wire = encodeCanStreamMessage({can_id, data, true, 2});

  parser.append(wire.data(), wire.size());
  auto frame = parser.tryParseFrame();

  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id_type() == IdType::EXTENDED);
  REQUIRE(frame->get_id() == can_id);
  REQUIRE(static_cast<size_t>(frame->get_len()) == data.size());
}

TEST_CASE("Parser: two coalesced CAN Stream messages, one frame each", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto a = encodeCanStreamMessage({0x100, {0x11, 0x22}, false, 2});
  auto b = encodeCanStreamMessage({0x200, {0x33, 0x44, 0x55, 0x66}, false, 2});

  std::vector<uint8_t> combined = a;
  combined.insert(combined.end(), b.begin(), b.end());
  parser.append(combined.data(), combined.size());

  auto f1 = parser.tryParseFrame();
  REQUIRE(f1.has_value());
  REQUIRE(f1->get_id() == 0x100u);

  auto f2 = parser.tryParseFrame();
  REQUIRE(f2.has_value());
  REQUIRE(f2->get_id() == 0x200u);

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("Parser: CAN Stream message carrying THREE packed CAN frames", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  std::vector<CanFrameSpec> specs = {
    {0x101, {0xA1}, false, 2},
    {0x102, {0xB1, 0xB2}, false, 2},
    {0x103, {0xC1, 0xC2, 0xC3, 0xC4}, false, 2},
  };
  auto wire = encodeCanStreamMixed(specs, {0, 1, 2});

  parser.append(wire.data(), wire.size());

  auto f1 = parser.tryParseFrame();
  REQUIRE(f1.has_value());
  REQUIRE(f1->get_id() == 0x101u);
  REQUIRE(f1->get_len() == 1);

  auto f2 = parser.tryParseFrame();
  REQUIRE(f2.has_value());
  REQUIRE(f2->get_id() == 0x102u);
  REQUIRE(f2->get_len() == 2);

  auto f3 = parser.tryParseFrame();
  REQUIRE(f3.has_value());
  REQUIRE(f3->get_id() == 0x103u);
  REQUIRE(f3->get_len() == 4);

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.notification_frames_skipped() == 0);
}

TEST_CASE("Parser: CAN Stream with CAN + Notification + CAN", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  std::vector<CanFrameSpec> specs = {
    {0x010, {0x01}, false, 2},
    {0x020, {0x02, 0x03}, false, 2},
  };
  auto wire = encodeCanStreamMixed(specs, {0, -1, 1});

  parser.append(wire.data(), wire.size());

  auto f1 = parser.tryParseFrame();
  REQUIRE(f1.has_value());
  REQUIRE(f1->get_id() == 0x010u);

  auto f2 = parser.tryParseFrame();
  REQUIRE(f2.has_value());
  REQUIRE(f2->get_id() == 0x020u);

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.notification_frames_skipped() == 1);
}

TEST_CASE("Parser: heartbeat alone returns nullopt and counts", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeHeartbeatMessage();
  parser.append(wire.data(), wire.size());

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.heartbeats_seen() == 1);
  REQUIRE(parser.dropped_bytes() == 0);
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("Parser: heartbeat followed by CAN Stream still delivers the frame",
          "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto hb = encodeHeartbeatMessage();
  auto cf = encodeCanStreamMessage({0x321, {0xDE, 0xAD}, false, 2});

  std::vector<uint8_t> combined = hb;
  combined.insert(combined.end(), cf.begin(), cf.end());
  parser.append(combined.data(), combined.size());

  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x321u);
  REQUIRE(parser.heartbeats_seen() == 1);
  REQUIRE(parser.dropped_bytes() == 0);
}

TEST_CASE("Parser: status response skipped cleanly", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeStatusResponseMessage();
  parser.append(wire.data(), wire.size());

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.status_responses_seen() == 1);
}

TEST_CASE("Parser: CAN FD stream skipped cleanly", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeCanFdMessage();
  parser.append(wire.data(), wire.size());

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.can_fd_messages_skipped() == 1);
}

TEST_CASE("Parser: unknown message id skipped cleanly", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeUnknownMessage(99);
  parser.append(wire.data(), wire.size());

  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.unknown_messages_skipped() == 1);
}

TEST_CASE("Parser: frame split across two appends", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeCanStreamMessage({0x456, {0x77, 0x88, 0x99}, false, 2});

  const size_t split = wire.size() / 2;
  parser.append(wire.data(), split);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());

  parser.append(wire.data() + split, wire.size() - split);
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x456u);
}

TEST_CASE("Parser: heartbeat split across two appends", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeHeartbeatMessage();

  parser.append(wire.data(), 7);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.heartbeats_seen() == 0);

  parser.append(wire.data() + 7, wire.size() - 7);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.heartbeats_seen() == 1);
}

TEST_CASE("Parser: garbage prefix is skipped, frame still delivered",
          "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  std::vector<uint8_t> wire = {0x00, 0xFF, 0x01, 0x02, 0x42};
  auto frame_bytes = encodeCanStreamMessage({0x555, {0xDE, 0xAD}, false, 2});
  wire.insert(wire.end(), frame_bytes.begin(), frame_bytes.end());

  parser.append(wire.data(), wire.size());
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x555u);
  REQUIRE(parser.dropped_bytes() == 5);
}

TEST_CASE("Parser: DLC 0 edge case", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeCanStreamMessage({0x010, {}, false, 2});
  parser.append(wire.data(), wire.size());

  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_len() == 0);
  REQUIRE(frame->get_id() == 0x010u);
}

TEST_CASE("Parser: DLC 8 edge case", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  auto wire = encodeCanStreamMessage({0x7FF, data, false, 2});
  parser.append(wire.data(), wire.size());

  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_len() == 8);
  REQUIRE(frame->get_id() == 0x7FFu);
}

TEST_CASE("Parser: reset clears state", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  parser.append(reinterpret_cast<const uint8_t *>("garbage"), 7);
  REQUIRE(parser.buffered_bytes() > 0);
  parser.reset();
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("Parser: heartbeat + 3-frame CAN Stream + heartbeat coalesced",
          "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  std::vector<CanFrameSpec> specs = {
    {0x701, {0xA1, 0xA2}, false, 2},
    {0x702, {0xB1, 0xB2, 0xB3}, false, 2},
    {0x703, {0xC1}, false, 2},
  };
  auto hb1 = encodeHeartbeatMessage();
  auto can = encodeCanStreamMixed(specs, {0, 1, 2});
  auto hb2 = encodeHeartbeatMessage();

  std::vector<uint8_t> combined;
  combined.insert(combined.end(), hb1.begin(), hb1.end());
  combined.insert(combined.end(), can.begin(), can.end());
  combined.insert(combined.end(), hb2.begin(), hb2.end());
  parser.append(combined.data(), combined.size());

  std::vector<uint32_t> got_ids;
  while (auto f = parser.tryParseFrame()) {
    got_ids.push_back(f->get_id());
  }
  REQUIRE(got_ids == std::vector<uint32_t>{0x701, 0x702, 0x703});
  REQUIRE(parser.heartbeats_seen() == 2);
  REQUIRE(parser.dropped_bytes() == 0);
}

TEST_CASE("Parser: many coalesced single-frame messages stress",
          "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  constexpr size_t kMessages = 64;
  std::vector<uint8_t> blob;
  for (size_t i = 0; i < kMessages; ++i) {
    auto m = encodeCanStreamMessage({static_cast<uint32_t>(0x100 + i),
                                     {static_cast<uint8_t>(i)}, false, 2});
    blob.insert(blob.end(), m.begin(), m.end());
  }
  parser.append(blob.data(), blob.size());

  for (size_t i = 0; i < kMessages; ++i) {
    auto frame = parser.tryParseFrame();
    REQUIRE(frame.has_value());
    REQUIRE(frame->get_id() == static_cast<uint32_t>(0x100 + i));
  }
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
}
