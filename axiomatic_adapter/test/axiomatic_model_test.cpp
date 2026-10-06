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

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif
#include "axiomatic_adapter/axiomatic_adapter.hpp"
#include "axiomatic_adapter/axiomatic_model.hpp"
#include "axiomatic_adapter/models/ax140900.hpp"
#include "axiomatic_adapter/models/ax142100a.hpp"

using polymath::socketcan::CanFrame;
using polymath::socketcan::IdType;

namespace
{

CanFrame makeFrame(canid_t id, bool extended, const std::vector<unsigned char> & payload)
{
  std::array<unsigned char, CAN_MAX_DLC> data = {0};
  std::copy(payload.begin(), payload.end(), data.begin());
  CanFrame frame;
  frame.set_can_id(id);
  if (extended) {
    frame.set_id_as_extended();
  }
  frame.set_len(static_cast<unsigned char>(payload.size()));
  frame.set_data(data);
  return frame;
}

void requireSameFrame(const CanFrame & actual, const CanFrame & expected)
{
  REQUIRE(actual.get_id_type() == expected.get_id_type());
  REQUIRE(actual.get_id() == expected.get_id());
  REQUIRE(actual.get_len() == expected.get_len());
  REQUIRE(actual.get_data() == expected.get_data());
}

std::vector<uint8_t> concat(const std::vector<uint8_t> & a, const std::vector<uint8_t> & b)
{
  std::vector<uint8_t> out(a);
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

// UMAX142100A Figure 3: ID 0x18F00401, 29-bit, 8 data bytes
const std::vector<uint8_t> AX142100A_MANUAL_EXAMPLE = {
  0x41, 0x58, 0x49, 0x4F, 0x28, 0x4E, 0x01, 0x00, 0x00, 0x0D, 0x00, 0x18,
  0x01, 0x04, 0xF0, 0x18, 0x12, 0x44, 0x21, 0x55, 0x61, 0x09, 0x01, 0x02,
};
const CanFrame AX142100A_MANUAL_FRAME = makeFrame(0x18F00401, true, {0x12, 0x44, 0x21, 0x55, 0x61, 0x09, 0x01, 0x02});

}  // namespace

TEST_CASE("AX142100A matches the manual example", "[model][ax142100a]")
{
  SECTION("decode")
  {
    const auto result =
      polymath::can::ax142100a::Ax142100a().decode(AX142100A_MANUAL_EXAMPLE.data(), AX142100A_MANUAL_EXAMPLE.size());
    REQUIRE(result.diagnostics.empty());
    REQUIRE(1 == result.frames.size());
    requireSameFrame(result.frames[0], AX142100A_MANUAL_FRAME);
  }

  SECTION("encode")
  {
    REQUIRE(AX142100A_MANUAL_EXAMPLE == polymath::can::ax142100a::Ax142100a().encode(AX142100A_MANUAL_FRAME));
  }
}

TEST_CASE("AX142100A skips raw data payloads", "[model][ax142100a]")
{
  const std::vector<uint8_t> raw = {
    0x41, 0x58, 0x49, 0x4F, 0x28, 0x4E, 0x01, 0x00, 0x00, 0x04, 0x00, 0x40, 'a', 'b', 'c'};
  const auto stream = concat(raw, AX142100A_MANUAL_EXAMPLE);

  const auto result = polymath::can::ax142100a::Ax142100a().decode(stream.data(), stream.size());
  REQUIRE(1 == result.diagnostics.size());
  REQUIRE(1 == result.frames.size());
  requireSameFrame(result.frames[0], AX142100A_MANUAL_FRAME);
}

TEST_CASE("AX140900 encodes a CAN Stream message with all CAN_MAX_DLC data bytes", "[model][ax140900]")
{
  const std::vector<uint8_t> expected = {
    'A',  'X',  'I',  'O',  0xBA, 0x36, 0x01, 0x00, 0x00, 0x0B, 0x00,  // header, 11-byte body
    0x54,  // CB: 2-byte TS, extended, DLC 4
    0xC0, 0x46,  // time stamp
    0x01, 0x04, 0xF0, 0x18,  // ID
    0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00,  // data: 4 declared, 4 past the body length
  };
  REQUIRE(
    expected == polymath::can::ax140900::Ax140900().encode(makeFrame(0x18F00401, true, {0x01, 0x02, 0x03, 0x04})));
}

TEST_CASE("AX140900 decodes packed frames and skips non-CAN content", "[model][ax140900]")
{
  const std::vector<uint8_t> can_stream = {
    'A',  'X',  'I',  'O',  0xBA, 0x36, 0x01, 0x00, 0x00, 0x15, 0x00,  // CAN Stream, 21-byte body
    0x02, 0x23, 0x01, 0xAA, 0xBB,  // no TS, standard 0x123, DLC 2
    0x80, 0x00, 0x00, 0x00, 0x00,  // notification frame
    0x71, 0x01, 0x02, 0x03, 0x04, 0x01, 0x00, 0xF0, 0x18, 0xCC,  // 4-byte TS, extended 0x18F00001, DLC 1
    0x01,  // truncated: DLC 1 frame with no ID
  };
  const std::vector<uint8_t> heartbeat = {'A', 'X', 'I', 'O', 0xBA, 0x36, 0x04, 0x00, 0x00, 0x00, 0x00};
  const auto stream = concat(can_stream, heartbeat);

  const auto result = polymath::can::ax140900::Ax140900().decode(stream.data(), stream.size());
  REQUIRE(2 == result.frames.size());
  requireSameFrame(result.frames[0], makeFrame(0x123, false, {0xAA, 0xBB}));
  requireSameFrame(result.frames[1], makeFrame(0x18F00001, true, {0xCC}));
  // notification, truncated frame, heartbeat
  REQUIRE(3 == result.diagnostics.size());
}

TEST_CASE("Models round trip", "[model]")
{
  const std::vector<CanFrame> frames = {
    makeFrame(0x123, false, {0x01, 0x02, 0x03}),
    makeFrame(0x18F00401, true, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}),
    makeFrame(0x7FF, false, {}),
  };

  for (const auto & name : polymath::can::modelNames()) {
    INFO("model " << name);
    const auto model = polymath::can::makeModel(name);
    for (const auto & frame : frames) {
      const auto message = model->encode(frame);
      const auto result = model->decode(message.data(), message.size());
      REQUIRE(1 == result.frames.size());
      requireSameFrame(result.frames[0], frame);
    }
  }
}

TEST_CASE("Models reject another model's Protocol ID", "[model]")
{
  const auto result =
    polymath::can::ax140900::Ax140900().decode(AX142100A_MANUAL_EXAMPLE.data(), AX142100A_MANUAL_EXAMPLE.size());
  REQUIRE(result.frames.empty());
  REQUIRE(1 == result.diagnostics.size());
}

TEST_CASE("Models drop buffers shorter than a header", "[model]")
{
  const std::vector<uint8_t> partial(AX142100A_MANUAL_EXAMPLE.begin(), AX142100A_MANUAL_EXAMPLE.begin() + 5);
  const auto result = polymath::can::ax142100a::Ax142100a().decode(partial.data(), partial.size());
  REQUIRE(result.frames.empty());
  REQUIRE(1 == result.diagnostics.size());
}

TEST_CASE("Model names map to their classes", "[model]")
{
  REQUIRE(2 == polymath::can::modelNames().size());
  REQUIRE(
    nullptr != dynamic_cast<const polymath::can::ax140900::Ax140900 *>(polymath::can::makeModel("ax140900").get()));
  REQUIRE(
    nullptr != dynamic_cast<const polymath::can::ax142100a::Ax142100a *>(polymath::can::makeModel("ax142100a").get()));
  REQUIRE_THROWS_AS(polymath::can::makeModel("bogus"), std::invalid_argument);
}

TEST_CASE("AxiomaticAdapter rejects a null model", "[model]")
{
  REQUIRE_THROWS_AS(
    polymath::can::AxiomaticAdapter(
      "192.168.0.34",
      "4000",
      [](std::unique_ptr<const CanFrame> /*frame*/) {},
      [](polymath::can::AxiomaticAdapter::socket_error_string_t /*error*/) {},
      polymath::can::AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
      true,
      nullptr),
    std::invalid_argument);
}
