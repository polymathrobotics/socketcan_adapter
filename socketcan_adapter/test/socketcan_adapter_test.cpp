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

#include "socketcan_adapter/socketcan_adapter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif

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

TEST_CASE("SocketcanAdapter receive sets timestamps", "[SocketcanAdapter]")
{
  // Create two adapters - one to send, one to receive
  polymath::socketcan::SocketcanAdapter sender("vcan0");
  polymath::socketcan::SocketcanAdapter receiver("vcan0");

  REQUIRE(sender.openSocket());
  REQUIRE(receiver.openSocket());

  // Set up promise/future for synchronization - no sleep needed
  std::promise<polymath::socketcan::CanFrame> frame_promise;
  std::future<polymath::socketcan::CanFrame> frame_future = frame_promise.get_future();

  // Register callback that fulfills the promise when frame is received
  receiver.setOnReceiveCallback(
    [&frame_promise](std::unique_ptr<const polymath::socketcan::CanFrame> frame) { frame_promise.set_value(*frame); });

  REQUIRE(receiver.startReceptionThread());

  // Create a frame to send
  polymath::socketcan::CanFrame tx_frame;
  tx_frame.set_can_id(0x123);
  std::array<unsigned char, CAN_MAX_DLC> data = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
  tx_frame.set_data(data);
  tx_frame.set_len(4);

  // Record time before send
  auto before_send = std::chrono::steady_clock::now();

  // Send the frame
  auto send_result = sender.send(tx_frame);
  REQUIRE_FALSE(send_result.has_value());

  // Wait for frame with explicit timeout (1 second is generous for vcan loopback)
  auto status = frame_future.wait_for(std::chrono::seconds(1));
  REQUIRE(status == std::future_status::ready);

  auto rx_frame = frame_future.get();
  auto after_receive = std::chrono::steady_clock::now();

  // Verify frame data matches
  REQUIRE(rx_frame.get_id() == 0x123);

  // Verify receive_time is set and within expected bounds
  auto receive_time = rx_frame.get_receive_time();
  REQUIRE(receive_time >= before_send);
  REQUIRE(receive_time <= after_receive);

  // Verify bus_time is set (non-default)
  auto bus_time = rx_frame.get_bus_time();
  auto epoch = std::chrono::system_clock::time_point{};
  REQUIRE(bus_time != epoch);

  receiver.joinReceptionThread();
  sender.closeSocket();
  receiver.closeSocket();
}

TEST_CASE("SocketcanAdapter receive timestamps are monotonic", "[SocketcanAdapter]")
{
  polymath::socketcan::SocketcanAdapter sender("vcan0");
  polymath::socketcan::SocketcanAdapter receiver("vcan0");

  REQUIRE(sender.openSocket());
  REQUIRE(receiver.openSocket());

  // Thread-safe frame collection using mutex-protected vector and a promise for completion
  constexpr size_t FRAME_COUNT = 3;
  std::mutex mtx;
  std::vector<polymath::socketcan::CanFrame> received_frames;
  std::promise<void> all_received_promise;
  std::future<void> all_received_future = all_received_promise.get_future();

  receiver.setOnReceiveCallback([&](std::unique_ptr<const polymath::socketcan::CanFrame> frame) {
    std::lock_guard<std::mutex> lock(mtx);
    received_frames.push_back(*frame);
    if (received_frames.size() == FRAME_COUNT) {
      all_received_promise.set_value();
    }
  });

  REQUIRE(receiver.startReceptionThread());

  // Send all frames
  for (size_t i = 0; i < FRAME_COUNT; ++i) {
    polymath::socketcan::CanFrame tx_frame;
    tx_frame.set_can_id(0x200 + static_cast<canid_t>(i));
    std::array<unsigned char, CAN_MAX_DLC> data = {static_cast<unsigned char>(i), 0, 0, 0, 0, 0, 0, 0};
    tx_frame.set_data(data);
    tx_frame.set_len(1);

    auto send_result = sender.send(tx_frame);
    REQUIRE_FALSE(send_result.has_value());
  }

  // Wait for all frames with explicit timeout
  auto status = all_received_future.wait_for(std::chrono::seconds(2));
  REQUIRE(status == std::future_status::ready);

  // Verify we received all frames
  {
    std::lock_guard<std::mutex> lock(mtx);
    REQUIRE(received_frames.size() == FRAME_COUNT);

    // Verify receive times are monotonically increasing
    for (size_t i = 1; i < received_frames.size(); ++i) {
      REQUIRE(received_frames[i].get_receive_time() >= received_frames[i - 1].get_receive_time());
    }
  }

  receiver.joinReceptionThread();
  sender.closeSocket();
  receiver.closeSocket();
}
