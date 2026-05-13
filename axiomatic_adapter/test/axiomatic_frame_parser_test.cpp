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

// Build the Axiomatic wire encoding for a single CAN frame.
std::vector<uint8_t> encodeFrame(
  uint32_t can_id,
  const std::vector<uint8_t> & data,
  bool is_extended,
  size_t timestamp_size = 2)
{
  std::vector<uint8_t> bytes;
  // Header
  for (auto b : AxiomaticFrameParser::HEADER) {
    bytes.push_back(b);
  }
  // Two reserved bytes
  bytes.push_back(0x00);
  bytes.push_back(0x00);

  const size_t id_len = is_extended ? 4 : 2;
  const size_t message_length = 1 + timestamp_size + id_len + data.size();
  bytes.push_back(static_cast<uint8_t>(message_length & 0xFF));
  bytes.push_back(static_cast<uint8_t>((message_length >> 8) & 0xFF));

  const uint8_t control =
    static_cast<uint8_t>(((timestamp_size & 0x3) << 5) | (is_extended ? (1 << 4) : 0) |
                         (data.size() & 0x0F));
  bytes.push_back(control);

  // Timestamp bytes (value does not matter to the parser).
  for (size_t i = 0; i < timestamp_size; ++i) {
    bytes.push_back(0xAB);
  }

  // CAN ID, little-endian.
  for (size_t i = 0; i < id_len; ++i) {
    bytes.push_back(static_cast<uint8_t>((can_id >> (i * 8)) & 0xFF));
  }

  // Data.
  bytes.insert(bytes.end(), data.begin(), data.end());
  return bytes;
}

}  // namespace

TEST_CASE("AxiomaticFrameParser: empty buffer returns nullopt", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.buffered_bytes() == 0);
  REQUIRE(parser.dropped_bytes() == 0);
}

TEST_CASE("AxiomaticFrameParser: single standard frame", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};
  auto wire = encodeFrame(0x123, data, /*is_extended=*/false);

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
  REQUIRE(parser.buffered_bytes() == 0);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
}

TEST_CASE("AxiomaticFrameParser: single extended frame", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0xAA, 0xBB, 0xCC};
  const uint32_t can_id = 0x1ABCDEF;
  auto wire = encodeFrame(can_id, data, /*is_extended=*/true);

  parser.append(wire.data(), wire.size());
  auto frame = parser.tryParseFrame();

  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id_type() == IdType::EXTENDED);
  REQUIRE(frame->get_id() == can_id);
  REQUIRE(static_cast<size_t>(frame->get_len()) == data.size());
  auto got = frame->get_data();
  for (size_t i = 0; i < data.size(); ++i) {
    REQUIRE(got[i] == data[i]);
  }
}

TEST_CASE("AxiomaticFrameParser: coalesced frames in one append", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto a = encodeFrame(0x100, {0x11, 0x22}, false);
  auto b = encodeFrame(0x200, {0x33, 0x44, 0x55, 0x66}, false);

  std::vector<uint8_t> combined = a;
  combined.insert(combined.end(), b.begin(), b.end());
  parser.append(combined.data(), combined.size());

  auto f1 = parser.tryParseFrame();
  REQUIRE(f1.has_value());
  REQUIRE(f1->get_id() == 0x100u);
  REQUIRE(f1->get_len() == 2);

  auto f2 = parser.tryParseFrame();
  REQUIRE(f2.has_value());
  REQUIRE(f2->get_id() == 0x200u);
  REQUIRE(f2->get_len() == 4);

  REQUIRE(parser.buffered_bytes() == 0);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
}

TEST_CASE("AxiomaticFrameParser: frame split across two appends", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeFrame(0x321, {0x01, 0x02, 0x03}, false);

  const size_t split = wire.size() / 2;
  parser.append(wire.data(), split);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.buffered_bytes() == split);

  parser.append(wire.data() + split, wire.size() - split);
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x321u);
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("AxiomaticFrameParser: garbage prefix is skipped", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  std::vector<uint8_t> wire;
  // Garbage that does not contain the header.
  for (uint8_t b : {0x00, 0xFF, 0x01, 0x02, 0x42}) {
    wire.push_back(b);
  }
  auto frame_bytes = encodeFrame(0x555, {0xDE, 0xAD}, false);
  wire.insert(wire.end(), frame_bytes.begin(), frame_bytes.end());

  parser.append(wire.data(), wire.size());
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x555u);
  REQUIRE(parser.dropped_bytes() == 5);
}

TEST_CASE("AxiomaticFrameParser: header split across appends", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeFrame(0x456, {0x77, 0x88}, false);

  // First append contains only the first 4 header bytes ("AXIO").
  parser.append(wire.data(), 4);
  REQUIRE_FALSE(parser.tryParseFrame().has_value());

  // Second append delivers the rest.
  parser.append(wire.data() + 4, wire.size() - 4);
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x456u);
  REQUIRE(parser.dropped_bytes() == 0);
}

TEST_CASE("AxiomaticFrameParser: DLC 0 edge case", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  auto wire = encodeFrame(0x010, {}, false);
  parser.append(wire.data(), wire.size());

  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_len() == 0);
  REQUIRE(frame->get_id() == 0x010u);
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("AxiomaticFrameParser: DLC 8 edge case", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  auto wire = encodeFrame(0x7FF, data, false);
  parser.append(wire.data(), wire.size());

  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_len() == 8);
  REQUIRE(frame->get_id() == 0x7FFu);
  auto got = frame->get_data();
  for (size_t i = 0; i < data.size(); ++i) {
    REQUIRE(got[i] == data[i]);
  }
}

TEST_CASE("AxiomaticFrameParser: reset clears state", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  parser.append(reinterpret_cast<const uint8_t *>("garbage"), 7);
  REQUIRE(parser.buffered_bytes() > 0);
  parser.reset();
  REQUIRE(parser.buffered_bytes() == 0);
}

TEST_CASE("AxiomaticFrameParser: many coalesced frames stress", "[AxiomaticFrameParser]")
{
  AxiomaticFrameParser parser;
  constexpr size_t kFrames = 64;
  std::vector<uint8_t> blob;
  for (size_t i = 0; i < kFrames; ++i) {
    auto f = encodeFrame(0x100 + i, {static_cast<uint8_t>(i)}, false);
    blob.insert(blob.end(), f.begin(), f.end());
  }
  parser.append(blob.data(), blob.size());

  for (size_t i = 0; i < kFrames; ++i) {
    auto frame = parser.tryParseFrame();
    REQUIRE(frame.has_value());
    REQUIRE(frame->get_id() == static_cast<uint32_t>(0x100 + i));
    REQUIRE(frame->get_len() == 1);
    REQUIRE(frame->get_data()[0] == static_cast<uint8_t>(i));
  }
  REQUIRE_FALSE(parser.tryParseFrame().has_value());
  REQUIRE(parser.buffered_bytes() == 0);
}
