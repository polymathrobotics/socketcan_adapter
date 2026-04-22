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

#include "axiomatic_adapter/axiomatic_adapter.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif
#include "socketcan_adapter/socketcan_adapter.hpp"

using polymath::can::AxiomaticAdapter;
using polymath::can::TCPSocketState;
using polymath::socketcan::CanFrame;

// NOTE: These tests were written to run on hq0-robot04 with the in office CAN setup
// NOTE: these tests leverage the socketcan adapter to act as a "secondary device" transmitting messages
// NOTE: the first run may fail. This is due to an issue with the Axiomatic adapter that causes just the first message
//       read to fail.
TEST_CASE("AxiomaticAdapter tests", "[AxiomaticAdapter]")
{
  SECTION("Constructor and destructor")
  {
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";
    AxiomaticAdapter adapter(ip_address, port);
    REQUIRE(adapter.get_socket_state() == TCPSocketState::CLOSED);
  }

  SECTION("Open and close socket")
  {
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";
    AxiomaticAdapter adapter(ip_address, port);
    REQUIRE(adapter.openSocket());
    REQUIRE(adapter.get_socket_state() == TCPSocketState::OPEN);
    REQUIRE(adapter.closeSocket());
    REQUIRE(adapter.get_socket_state() == TCPSocketState::CLOSED);
  }

  SECTION("Receive CanFrame")
  {
    std::string socketcan_send_inteface = "can0";
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";

    polymath::socketcan::SocketcanAdapter send_adapter(socketcan_send_inteface);
    REQUIRE(send_adapter.openSocket());

    AxiomaticAdapter receipt_adapter(ip_address, port);
    REQUIRE(receipt_adapter.openSocket());

    CanFrame frame;
    frame.set_can_id(0x123);
    REQUIRE(frame.get_id() == 0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    auto send_result = send_adapter.send(frame);
    REQUIRE(!send_result.has_value());

    std::optional<CanFrame> maybe_frame;
    // try to receive 20 times
    for (int i = 0; i < 20; i++) {
      maybe_frame = receipt_adapter.receive();
      if (maybe_frame.has_value()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(maybe_frame.has_value());

    CanFrame received_frame = *maybe_frame;
    REQUIRE(received_frame.get_id() == 0x123);
    REQUIRE(received_frame.get_len() == 4);
    REQUIRE(received_frame.get_data() == data);

    REQUIRE(send_adapter.closeSocket());
    REQUIRE(receipt_adapter.closeSocket());
  }

  SECTION("Send CanFrame")
  {
    std::string socketcan_inteface = "can0";
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";

    polymath::socketcan::SocketcanAdapter receipt_adapter(socketcan_inteface);
    REQUIRE(receipt_adapter.openSocket());

    AxiomaticAdapter send_adapter(ip_address, port);
    REQUIRE(send_adapter.openSocket());

    CanFrame frame;
    frame.set_can_id(0x123);
    REQUIRE(frame.get_id() == 0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    auto send_result = send_adapter.send(frame);
    REQUIRE(!send_result.has_value());

    std::optional<CanFrame> maybe_frame;
    // try to receive 20 times
    for (int i = 0; i < 20; i++) {
      maybe_frame = receipt_adapter.receive();
      if (maybe_frame.has_value()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    REQUIRE(maybe_frame.has_value());

    CanFrame received_frame = *maybe_frame;
    REQUIRE(received_frame.get_id() == 0x123);
    REQUIRE(received_frame.get_len() == 4);
    REQUIRE(received_frame.get_data() == data);

    REQUIRE(send_adapter.closeSocket());
    REQUIRE(receipt_adapter.closeSocket());
  }

  SECTION("Reception thread")
  {
    std::string socketcan_send_inteface = "can0";
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";

    polymath::socketcan::SocketcanAdapter send_adapter(socketcan_send_inteface);
    REQUIRE(send_adapter.openSocket());

    bool callback_called = false;
    AxiomaticAdapter receipt_adapter(
      ip_address, port, [&callback_called](std::unique_ptr<const CanFrame> /*frame*/) { callback_called = true; });

    REQUIRE(receipt_adapter.openSocket());
    REQUIRE(receipt_adapter.startReceptionThread());

    CanFrame frame;
    frame.set_can_id(0x123);
    frame.set_len(4);
    std::array<unsigned char, CAN_MAX_DLC> data = {0x01, 0x02, 0x03, 0x04};
    frame.set_data(data);

    auto send_result = send_adapter.send(frame);
    REQUIRE(!send_result.has_value());

    // Allow some time for the reception thread
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    REQUIRE(callback_called);

    REQUIRE(receipt_adapter.joinReceptionThread());
    REQUIRE(receipt_adapter.closeSocket());
    REQUIRE(send_adapter.closeSocket());
  }

  SECTION("Error handling")
  {
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";

    int32_t num_error_callbacks_called = 0;
    std::string error_message = "";
    AxiomaticAdapter adapter(
      ip_address,
      port,
      [](std::unique_ptr<const polymath::socketcan::CanFrame> /*frame*/) { /*do nothing*/ },
      [&num_error_callbacks_called, &error_message](std::string error) {
        num_error_callbacks_called++;
        error_message = error;
      });

    adapter.openSocket();
    adapter.startReceptionThread();

    REQUIRE(adapter.closeSocket());

    // Allow for poll to fail
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    adapter.joinReceptionThread();

    REQUIRE(num_error_callbacks_called > 0);
    REQUIRE(!error_message.empty());
  }

  SECTION("Check thread running state")
  {
    std::string ip_address = "192.168.0.34";
    std::string port = "4000";
    AxiomaticAdapter adapter(ip_address, port, [](std::unique_ptr<const CanFrame> /*frame*/) { /* No-op */ });
    REQUIRE(!adapter.is_thread_running());
    REQUIRE(adapter.openSocket());

    REQUIRE(adapter.startReceptionThread());
    REQUIRE(adapter.is_thread_running());
    REQUIRE(adapter.joinReceptionThread());
    REQUIRE(!adapter.is_thread_running());
    REQUIRE(adapter.closeSocket());
  }
}
