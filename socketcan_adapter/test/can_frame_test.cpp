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
#include <chrono>
#include <cstdint>
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif

#include "socketcan_adapter/socketcan_adapter.hpp"

TEST_CASE("CanFrame default constructor", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;

  REQUIRE(frame.get_id() == 0);
  REQUIRE(frame.get_len() == 0);
  REQUIRE(frame.get_frame_type() == polymath::socketcan::FrameType::DATA);
  REQUIRE(frame.get_id_type() == polymath::socketcan::IdType::STANDARD);
}

TEST_CASE("CanFrame constructor with can_frame", "[CanFrame]")
{
  struct can_frame canFrame = {};
  canFrame.can_id = 0x123;
  canFrame.can_dlc = 8;
  std::fill(canFrame.data, canFrame.data + 8, 0xFF);

  polymath::socketcan::CanFrame frame(canFrame);

  REQUIRE(frame.get_id() == 0x123);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(frame.get_data() == std::array<unsigned char, CAN_MAX_DLC>{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

TEST_CASE("CanFrame constructor with raw_id, data, and timestamp", "[CanFrame]")
{
  canid_t raw_id = 0x456;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};
  std::uint64_t timestamp = 123456789;

  polymath::socketcan::CanFrame frame(raw_id, data, timestamp);

  REQUIRE(frame.get_id() == raw_id);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(frame.get_data() == data);
}

TEST_CASE("CanFrame constructor with additional parameters", "[CanFrame]")
{
  canid_t raw_id = 0x789;
  std::array<unsigned char, CAN_MAX_DLC> data = {9, 8, 7, 6, 5, 4, 3, 2};
  std::uint64_t timestamp = 987654321;
  auto frame_type = polymath::socketcan::FrameType::REMOTE;
  auto frame_id_type = polymath::socketcan::IdType::EXTENDED;

  polymath::socketcan::CanFrame frame(raw_id, data, timestamp, frame_type, frame_id_type);

  REQUIRE(frame.get_id() == raw_id);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(frame.get_data() == data);
  REQUIRE(frame.get_frame_type() == frame_type);
  REQUIRE(frame.get_id_type() == frame_id_type);
}

TEST_CASE("Set and get frame", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  struct can_frame canFrame = {};
  canFrame.can_id = 0xABC;
  canFrame.can_dlc = 6;
  std::fill(canFrame.data, canFrame.data + 6, 0xEE);

  REQUIRE(frame.set_frame(canFrame));
  REQUIRE(frame.get_id() == (0xABC & CAN_SFF_MASK));
  REQUIRE(frame.get_len() == 6);
  REQUIRE(frame.get_data() == std::array<unsigned char, CAN_MAX_DLC>{0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x00, 0x00});
}

TEST_CASE("Set and get ID types", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;

  frame.set_id_as_extended();
  REQUIRE(frame.get_id_type() == polymath::socketcan::IdType::EXTENDED);

  frame.set_id_as_standard();
  REQUIRE(frame.get_id_type() == polymath::socketcan::IdType::STANDARD);
}

TEST_CASE("Set and get frame types", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;

  frame.set_type_error();
  REQUIRE(frame.get_frame_type() == polymath::socketcan::FrameType::ERROR);

  frame.set_type_remote();
  REQUIRE(frame.get_frame_type() == polymath::socketcan::FrameType::REMOTE);

  frame.set_type_data();
  REQUIRE(frame.get_frame_type() == polymath::socketcan::FrameType::DATA);
}

TEST_CASE("Set and get length", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  frame.set_len(5);

  REQUIRE(frame.get_len() == 5);
}

TEST_CASE("Set and get data", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};

  frame.set_data(data);
  REQUIRE(frame.get_data() == data);
}

TEST_CASE("Get masked ID", "[CanFrame]")
{
  canid_t id = 0x1FFFFFFF;
  std::array<unsigned char, CAN_MAX_DLC> data = {};
  polymath::socketcan::CanFrame frame(id, data, 0);

  REQUIRE(frame.get_masked_id(12, 0) == (id & 0xFFF));
  REQUIRE(frame.get_masked_id(8, 8) == ((id >> 8) & 0xFF) << 8);
}

TEST_CASE("Get frame", "[CanFrame]")
{
  struct can_frame canFrame = {};
  canFrame.can_id = 0x123;
  canFrame.can_dlc = 4;
  std::fill(canFrame.data, canFrame.data + 4, 0xAA);

  polymath::socketcan::CanFrame frame(canFrame);
  struct can_frame retrievedFrame = frame.get_frame();

  REQUIRE(retrievedFrame.can_id == canFrame.can_id);
  REQUIRE(retrievedFrame.can_dlc == canFrame.can_dlc);
  REQUIRE(std::equal(std::begin(retrievedFrame.data), std::end(retrievedFrame.data), std::begin(canFrame.data)));
}

TEST_CASE("Get error as string", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  frame.set_type_error();

  std::string error = frame.get_error();
  REQUIRE(!error.empty());
}

TEST_CASE("Set and get bus timestamp with uint64_t", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  std::uint64_t timestamp_us = 1000000;  // 1 second in microseconds

  frame.set_bus_timestamp(timestamp_us);
  auto bus_time = frame.get_bus_time();

  auto expected = std::chrono::system_clock::time_point(std::chrono::microseconds(timestamp_us));
  REQUIRE(bus_time == expected);
}

TEST_CASE("Set and get receive timestamp with uint64_t", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  std::uint64_t timestamp_us = 2000000;  // 2 seconds in microseconds

  frame.set_receive_timestamp(timestamp_us);
  auto receive_time = frame.get_receive_time();

  auto expected = std::chrono::steady_clock::time_point(std::chrono::microseconds(timestamp_us));
  REQUIRE(receive_time == expected);
}

TEST_CASE("Set and get bus timestamp with time_point", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  auto timestamp = std::chrono::system_clock::now();

  frame.set_bus_timestamp(timestamp);
  auto bus_time = frame.get_bus_time();

  REQUIRE(bus_time == timestamp);
}

TEST_CASE("Set and get receive timestamp with time_point", "[CanFrame]")
{
  polymath::socketcan::CanFrame frame;
  auto timestamp = std::chrono::steady_clock::now();

  frame.set_receive_timestamp(timestamp);
  auto receive_time = frame.get_receive_time();

  REQUIRE(receive_time == timestamp);
}

TEST_CASE("Constructor initializes receive_time to now", "[CanFrame]")
{
  auto before = std::chrono::steady_clock::now();

  canid_t raw_id = 0x123;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};
  std::uint64_t bus_timestamp = 123456789;
  polymath::socketcan::CanFrame frame(raw_id, data, bus_timestamp);

  auto after = std::chrono::steady_clock::now();
  auto receive_time = frame.get_receive_time();

  REQUIRE(receive_time >= before);
  REQUIRE(receive_time <= after);
}

TEST_CASE("Constructor sets bus_time from timestamp parameter", "[CanFrame]")
{
  canid_t raw_id = 0x123;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};
  std::uint64_t bus_timestamp = 5000000;  // 5 seconds in microseconds

  polymath::socketcan::CanFrame frame(raw_id, data, bus_timestamp);
  auto bus_time = frame.get_bus_time();

  auto expected = std::chrono::system_clock::time_point(std::chrono::microseconds(bus_timestamp));
  REQUIRE(bus_time == expected);
}

TEST_CASE("Extended constructor initializes timestamps correctly", "[CanFrame]")
{
  auto before = std::chrono::steady_clock::now();

  canid_t raw_id = 0x789;
  std::array<unsigned char, CAN_MAX_DLC> data = {9, 8, 7, 6, 5, 4, 3, 2};
  std::uint64_t bus_timestamp = 10000000;  // 10 seconds in microseconds
  auto frame_type = polymath::socketcan::FrameType::REMOTE;
  auto frame_id_type = polymath::socketcan::IdType::EXTENDED;

  polymath::socketcan::CanFrame frame(raw_id, data, bus_timestamp, frame_type, frame_id_type);

  auto after = std::chrono::steady_clock::now();

  // Check receive_time was set to approximately now
  auto receive_time = frame.get_receive_time();
  REQUIRE(receive_time >= before);
  REQUIRE(receive_time <= after);

  // Check bus_time was set from the parameter
  auto bus_time = frame.get_bus_time();
  auto expected_bus_time = std::chrono::system_clock::time_point(std::chrono::microseconds(bus_timestamp));
  REQUIRE(bus_time == expected_bus_time);
}
