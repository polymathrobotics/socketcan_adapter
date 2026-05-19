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

// These tests stand up a localhost TCP acceptor in-process, point an
// AxiomaticAdapter at it, then assert that the bytes the adapter writes on
// send() match the Axiomatic wire format -- specifically that the declared
// message_length agrees with the actual number of payload bytes, regardless
// of DLC.

#include "axiomatic_adapter/axiomatic_adapter.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif

#include "axiomatic_adapter/axiomatic_frame_parser.hpp"
#include "socketcan_adapter/can_frame.hpp"

using polymath::can::AxiomaticAdapter;
using polymath::can::AxiomaticFrameParser;
using polymath::socketcan::CanFrame;

namespace
{

// Spins up a one-shot localhost acceptor that captures everything written by
// the first client until the client disconnects or `max_bytes` is reached.
class LoopbackServer
{
public:
  LoopbackServer()
  : io_context_()
  , acceptor_(io_context_, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 0))
  {
    port_ = acceptor_.local_endpoint().port();
  }

  uint16_t port() const { return port_; }

  // Blocks until a client connects and either disconnects or has sent
  // `min_bytes`. Returns whatever was read.
  std::vector<uint8_t> collect(size_t min_bytes, std::chrono::milliseconds timeout)
  {
    std::vector<uint8_t> out;
    std::thread reader([&]() {
      boost::asio::ip::tcp::socket peer(io_context_);
      acceptor_.accept(peer);

      std::array<uint8_t, 1024> buf{};
      boost::system::error_code ec;
      while (out.size() < min_bytes) {
        size_t n = peer.read_some(boost::asio::buffer(buf), ec);
        if (ec) {
          break;
        }
        out.insert(out.end(), buf.data(), buf.data() + n);
      }
    });

    if (reader.joinable()) {
      // Detach if timeout elapses so the test does not hang on regression.
      auto start = std::chrono::steady_clock::now();
      while (out.size() < min_bytes &&
             std::chrono::steady_clock::now() - start < timeout)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      // Close the acceptor's pending side so read_some unblocks if the client
      // is gone.
      boost::system::error_code ec;
      acceptor_.close(ec);
      reader.join();
    }
    return out;
  }

private:
  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::acceptor acceptor_;
  uint16_t port_{0};
};

void assertEncodingMatchesDlc(uint16_t can_id, const std::vector<uint8_t> & payload)
{
  LoopbackServer server;
  const std::string ip = "127.0.0.1";
  const std::string port = std::to_string(server.port());

  AxiomaticAdapter adapter(ip, port);
  REQUIRE(adapter.openSocket());

  // Issue the send from another thread; the server collects on this thread.
  std::thread sender([&]() {
    CanFrame frame;
    frame.set_can_id(can_id);
    frame.set_len(static_cast<unsigned char>(payload.size()));
    std::array<unsigned char, CAN_MAX_DLC> data{};
    for (size_t i = 0; i < payload.size(); ++i) {
      data[i] = payload[i];
    }
    frame.set_data(data);
    auto err = adapter.send(frame);
    REQUIRE_FALSE(err.has_value());
  });

  // Total wire bytes: header(7) + reserved(2) + length(2) + message_length
  // where message_length = control(1) + timestamp(2) + id_len(2 for standard) + dlc.
  const size_t expected_total = 11 + 1 + 2 + 2 + payload.size();
  auto bytes = server.collect(expected_total, std::chrono::milliseconds(500));
  sender.join();
  adapter.closeSocket();

  REQUIRE(bytes.size() >= expected_total);

  // 6-byte sync prefix.
  for (size_t i = 0; i < AxiomaticFrameParser::SYNC_PREFIX.size(); ++i) {
    REQUIRE(bytes[i] == AxiomaticFrameParser::SYNC_PREFIX[i]);
  }
  // Message ID (bytes 6-7) must be the deprecated CAN Stream (= 1).
  const uint16_t msg_id = static_cast<uint16_t>(bytes[6]) |
                          (static_cast<uint16_t>(bytes[7]) << 8);
  REQUIRE(msg_id == AxiomaticFrameParser::MSG_ID_CAN_STREAM_DEPRECATED);
  // Message Version is currently always 0.
  REQUIRE(bytes[8] == 0x00);

  // Declared message_length must equal control(1) + timestamp(2) + id(2) + dlc.
  const uint16_t declared = static_cast<uint16_t>(bytes[9]) |
                            (static_cast<uint16_t>(bytes[10]) << 8);
  const uint16_t expected_declared = static_cast<uint16_t>(1 + 2 + 2 + payload.size());
  REQUIRE(declared == expected_declared);

  // Actual payload bytes after the 11-byte fixed prefix must equal declared.
  REQUIRE((bytes.size() - 11) == declared);

  // Payload data bytes match -- after control(1) + timestamp(2) + id(2) = 5 bytes
  // past byte 11, so they start at offset 11 + 5 = 16.
  for (size_t i = 0; i < payload.size(); ++i) {
    REQUIRE(bytes[16 + i] == payload[i]);
  }
}

}  // namespace

TEST_CASE("AxiomaticAdapter::send encodes DLC 4 with matching length", "[AxiomaticAdapter][send]")
{
  assertEncodingMatchesDlc(0x123, {0x01, 0x02, 0x03, 0x04});
}

TEST_CASE("AxiomaticAdapter::send encodes DLC 0 with matching length", "[AxiomaticAdapter][send]")
{
  assertEncodingMatchesDlc(0x123, {});
}

TEST_CASE("AxiomaticAdapter::send encodes DLC 8 with matching length", "[AxiomaticAdapter][send]")
{
  assertEncodingMatchesDlc(0x123, {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88});
}

TEST_CASE("AxiomaticAdapter::send wire format round-trips through the parser", "[AxiomaticAdapter][send]")
{
  // The receive parser is the inverse of send(); verify they agree end-to-end.
  LoopbackServer server;
  const std::string ip = "127.0.0.1";
  const std::string port = std::to_string(server.port());

  AxiomaticAdapter adapter(ip, port);
  REQUIRE(adapter.openSocket());

  const std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF};

  std::thread sender([&]() {
    CanFrame frame;
    frame.set_can_id(0x321);
    frame.set_len(static_cast<unsigned char>(payload.size()));
    std::array<unsigned char, CAN_MAX_DLC> data{};
    for (size_t i = 0; i < payload.size(); ++i) {
      data[i] = payload[i];
    }
    frame.set_data(data);
    REQUIRE_FALSE(adapter.send(frame).has_value());
  });

  const size_t expected_total = 11 + 1 + 2 + 2 + payload.size();
  auto bytes = server.collect(expected_total, std::chrono::milliseconds(500));
  sender.join();
  adapter.closeSocket();

  AxiomaticFrameParser parser;
  parser.append(bytes.data(), bytes.size());
  auto frame = parser.tryParseFrame();
  REQUIRE(frame.has_value());
  REQUIRE(frame->get_id() == 0x321u);
  REQUIRE(static_cast<size_t>(frame->get_len()) == payload.size());
  auto got = frame->get_data();
  for (size_t i = 0; i < payload.size(); ++i) {
    REQUIRE(got[i] == payload[i]);
  }
}
