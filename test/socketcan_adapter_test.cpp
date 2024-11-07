#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <cstdlib>
#include <iostream>
#include "socketcan_adapter/socketcan_adapter.hpp"

using namespace polymath::socketcan;

TEST_CASE("SocketcanAdapter tests", "[SocketcanAdapter]")
{

  SECTION("Constructor and destructor")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    REQUIRE(adapter.get_socket_state() == SocketState::CLOSED);
  }

  SECTION("Open and close socket")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(adapter.get_socket_state() == SocketState::OPEN);
    REQUIRE(adapter.closeSocket());
    REQUIRE(adapter.get_socket_state() == SocketState::CLOSED);
  }

  SECTION("Receive and send CanFrame")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter receipt_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(receipt_adapter.openSocket());

    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    auto send_result = adapter.send(frame);
    REQUIRE(!send_result.has_value());

    std::optional<CanFrame> received_frame_opt = receipt_adapter.receive();
    REQUIRE(received_frame_opt.has_value());

    CanFrame received_frame = *received_frame_opt;
    REQUIRE(received_frame.get_id() == 0x123);
    REQUIRE(received_frame.get_len() == 4);
    REQUIRE(received_frame.get_data() == data);

    REQUIRE(adapter.closeSocket());
    REQUIRE(receipt_adapter.closeSocket());
  }

  SECTION("Reception thread")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    bool callback_called = false;
    adapter.setOnReceiveCallback(
      [&callback_called](std::unique_ptr<const CanFrame>/*frame*/)
      {
        callback_called = true;
      });
    REQUIRE(adapter.startReceptionThread());

    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    auto send_result = send_adapter.send(frame);
    REQUIRE(!send_result.has_value());

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Allow some time for the reception thread
    REQUIRE(callback_called);

    REQUIRE(adapter.joinReceptionThread());
    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Error handling")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    adapter.openSocket();

    int32_t num_error_callbacks_called = 0;
    std::string error_message = "";
    adapter.setOnErrorCallback(
      [&num_error_callbacks_called, &error_message](std::string error)
      {
        num_error_callbacks_called++;
        error_message = error;
      });

    adapter.startReceptionThread();

    REQUIRE(adapter.closeSocket());

    // Allow for poll to fail
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    adapter.joinReceptionThread();

    REQUIRE(num_error_callbacks_called > 0);
    REQUIRE(!error_message.empty());
  }

  SECTION("Set receive timeout")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    std::chrono::duration<float> new_timeout = std::chrono::milliseconds(100);
    adapter.set_receive_timeout(new_timeout);
    REQUIRE(adapter.openSocket());
    // Additional tests to verify the timeout effect can be added
    REQUIRE(adapter.closeSocket());
  }

  SECTION("Check thread running state")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    REQUIRE(!adapter.is_thread_running());
    REQUIRE(adapter.openSocket());

    adapter.setOnReceiveCallback([](std::unique_ptr<const CanFrame>/*frame*/) { /* No-op */});
    REQUIRE(adapter.startReceptionThread());
    REQUIRE(adapter.is_thread_running());
    REQUIRE(adapter.joinReceptionThread());
    REQUIRE(!adapter.is_thread_running());
    REQUIRE(adapter.closeSocket());
  }
}

TEST_CASE("socketcan filters", "[SocketcanAdapter]")
{
  SECTION("Set filters")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    REQUIRE(adapter.openSocket());

    SocketcanAdapter::filter_vector_t filters = {
      // Example filter
      {0x123, 0xFFF},
    };
    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    REQUIRE(adapter.closeSocket());
  }

  SECTION("Verify Specific Standard Filters SFF")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x123, 0xFFF},
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);

    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that shouldn't come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Verify Specific Inverse Filters SFF")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x123 | CAN_INV_FILTER, 0xFFF},
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    // 0x123 should be blacklisted
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    // Now test something that should come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Verify Groups of Standard Filters SFF")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x123, 0xFF0},
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);

    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that should ALSO come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that shouldn't come through
    frame.set_can_id(0x135);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Verify Groups of Inverse Filters SFF")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x123 | CAN_INV_FILTER, 0xFF0},
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    // 0x123 shouldn't come through
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    // Now test something that should ALSO not come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    // Now test something that should come through
    frame.set_can_id(0x135);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Allow 0x123 and 0x125 ONLY")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x123, CAN_EFF_MASK},   // allow 0x123 ONLY, including EFF
      {0x125, CAN_EFF_MASK},   // allow 0x125 ONLY, including EFF
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);

    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that should ALSO come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that shouldn't come through
    frame.set_can_id(0x135);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Allow 0x120 - 0x12F but not 0x125")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    SocketcanAdapter send_adapter(interface_name);
    REQUIRE(adapter.openSocket());
    REQUIRE(send_adapter.openSocket());

    // Example filter, let's 12X through
    SocketcanAdapter::filter_vector_t filters = {
      {0x120, 0xFF0},                    // allow 0x123 ONLY
      {0x125 | CAN_INV_FILTER, 0xFFF},   // dis allow 0x125 ONLY
    };

    auto result = adapter.setFilters(filters);
    REQUIRE(!result.has_value());

    result = adapter.setJoinOverwrite(true);
    REQUIRE(!result.has_value());

    // Send frame that should pass through the filter
    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    std::optional<SocketcanAdapter::socket_error_string_t> err_str_optional;
    CanFrame receive_frame;

    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);

    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Now test something that should ALSO come through
    frame.set_can_id(0x12F);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(!err_str_optional.has_value());
    REQUIRE(receive_frame.get_id() == frame.get_id());

    // Also make sure 0x125 doesn't come through
    frame.set_can_id(0x125);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    // Now test something that shouldn't come through
    frame.set_can_id(0x135);
    err_str_optional = send_adapter.send(frame);
    REQUIRE(!err_str_optional.has_value());

    err_str_optional = adapter.receive(receive_frame);
    REQUIRE(err_str_optional.has_value());

    REQUIRE(adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Set error mask")
  {
    std::string interface_name = "vcan0";
    SocketcanAdapter adapter(interface_name);
    REQUIRE(adapter.openSocket());

    can_err_mask_t error_mask = CAN_ERR_MASK;
    auto result = adapter.setErrorMaskOverwrite(error_mask);
    REQUIRE(!result.has_value());

    REQUIRE(adapter.closeSocket());
  }
}
