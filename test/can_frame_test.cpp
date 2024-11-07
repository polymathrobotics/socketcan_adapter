#define CATCH_CONFIG_MAIN // this tells Catch to provide a main()
                          // only do this when all tests in 1 cpp file

#include <catch2/catch.hpp>
#include "socketcan_adapter/socketcan_adapter.hpp"

using namespace polymath::socketcan;

TEST_CASE("CanFrame default constructor", "[CanFrame]") {
  CanFrame frame;

  REQUIRE(frame.get_id() == 0);
  REQUIRE(frame.get_len() == 0);
  REQUIRE(frame.get_frame_type() == FrameType::DATA);
  REQUIRE(frame.get_id_type() == IdType::STANDARD);
}

TEST_CASE("CanFrame constructor with can_frame", "[CanFrame]") {
  struct can_frame canFrame = {};
  canFrame.can_id = 0x123;
  canFrame.can_dlc = 8;
  std::fill(canFrame.data, canFrame.data + 8, 0xFF);

  CanFrame frame(canFrame);

  REQUIRE(frame.get_id() == 0x123);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(
    frame.get_data() ==
    std::array<unsigned char, CAN_MAX_DLC>{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

TEST_CASE("CanFrame constructor with raw_id, data, and timestamp", "[CanFrame]") {
  canid_t raw_id = 0x456;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};
  uint64_t timestamp = 123456789;

  CanFrame frame(raw_id, data, timestamp);

  REQUIRE(frame.get_id() == raw_id);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(frame.get_data() == data);
}

TEST_CASE("CanFrame constructor with additional parameters", "[CanFrame]") {
  canid_t raw_id = 0x789;
  std::array<unsigned char, CAN_MAX_DLC> data = {9, 8, 7, 6, 5, 4, 3, 2};
  uint64_t timestamp = 987654321;
  FrameType frame_type = FrameType::REMOTE;
  IdType frame_id_type = IdType::EXTENDED;

  CanFrame frame(raw_id, data, timestamp, frame_type, frame_id_type);

  REQUIRE(frame.get_id() == raw_id);
  REQUIRE(frame.get_len() == 8);
  REQUIRE(frame.get_data() == data);
  REQUIRE(frame.get_frame_type() == frame_type);
  REQUIRE(frame.get_id_type() == frame_id_type);
}

TEST_CASE("Set and get frame", "[CanFrame]") {
  CanFrame frame;
  struct can_frame canFrame = {};
  canFrame.can_id = 0xABC;
  canFrame.can_dlc = 6;
  std::fill(canFrame.data, canFrame.data + 6, 0xEE);

  REQUIRE(frame.set_frame(canFrame));
  REQUIRE(frame.get_id() == (0xABC & CAN_SFF_MASK));
  REQUIRE(frame.get_len() == 6);
  REQUIRE(
    frame.get_data() ==
    std::array<unsigned char, CAN_MAX_DLC>{0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0xEE, 0x00, 0x00});
}

TEST_CASE("Set and get ID types", "[CanFrame]") {
  CanFrame frame;

  frame.set_id_as_extended();
  REQUIRE(frame.get_id_type() == IdType::EXTENDED);

  frame.set_id_as_standard();
  REQUIRE(frame.get_id_type() == IdType::STANDARD);
}

TEST_CASE("Set and get frame types", "[CanFrame]") {
  CanFrame frame;

  frame.set_type_error();
  REQUIRE(frame.get_frame_type() == FrameType::ERROR);

  frame.set_type_remote();
  REQUIRE(frame.get_frame_type() == FrameType::REMOTE);

  frame.set_type_data();
  REQUIRE(frame.get_frame_type() == FrameType::DATA);
}

TEST_CASE("Set and get length", "[CanFrame]") {
  CanFrame frame;
  frame.set_len(5);

  REQUIRE(frame.get_len() == 5);
}

TEST_CASE("Set and get data", "[CanFrame]") {
  CanFrame frame;
  std::array<unsigned char, CAN_MAX_DLC> data = {1, 2, 3, 4, 5, 6, 7, 8};

  frame.set_data(data);
  REQUIRE(frame.get_data() == data);
}

TEST_CASE("Get masked ID", "[CanFrame]") {
  canid_t id = 0x1FFFFFFF;
  std::array<unsigned char, CAN_MAX_DLC> data = {};
  CanFrame frame(id, data, 0);

  REQUIRE(frame.get_masked_id(12, 0) == (id & 0xFFF));
  REQUIRE(frame.get_masked_id(8, 8) == ((id >> 8) & 0xFF) << 8);
}

TEST_CASE("Get frame", "[CanFrame]") {
  struct can_frame canFrame = {};
  canFrame.can_id = 0x123;
  canFrame.can_dlc = 4;
  std::fill(canFrame.data, canFrame.data + 4, 0xAA);

  CanFrame frame(canFrame);
  struct can_frame retrievedFrame = frame.get_frame();

  REQUIRE(retrievedFrame.can_id == canFrame.can_id);
  REQUIRE(retrievedFrame.can_dlc == canFrame.can_dlc);
  REQUIRE(
    std::equal(
      std::begin(retrievedFrame.data), std::end(retrievedFrame.data),
      std::begin(canFrame.data)));
}

TEST_CASE("Get error as string", "[CanFrame]") {
  CanFrame frame;
  frame.set_type_error();

  std::string error = frame.get_error();
  REQUIRE(!error.empty());   // This test can be expanded based on the actual implementation of get_error()
}
