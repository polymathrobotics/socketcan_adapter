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

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif
#include "axiomatic_adapter/axiomatic_adapter.hpp"
#include "axiomatic_adapter/models/ax140900.hpp"
#include "axiomatic_adapter/models/ax142100a.hpp"

using boost::asio::ip::tcp;
using polymath::can::AxiomaticAdapter;
using polymath::socketcan::CanFrame;

namespace
{

constexpr std::chrono::seconds RECEIVE_WAIT{2};
constexpr std::chrono::milliseconds SPLIT_WRITE_GAP{20};

// UMAX142100A Figure 3: ID 0x18F00401, 29-bit, 8 data bytes
const std::vector<uint8_t> AX142100A_MANUAL_EXAMPLE = {
  0x41, 0x58, 0x49, 0x4F, 0x28, 0x4E, 0x01, 0x00, 0x00, 0x0D, 0x00, 0x18,
  0x01, 0x04, 0xF0, 0x18, 0x12, 0x44, 0x21, 0x55, 0x61, 0x09, 0x01, 0x02,
};

/// Local TCP server standing in for the converter
class FakeConverter
{
public:
  FakeConverter()
  : acceptor_(io_context_, tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0))
  , socket_(io_context_)
  {}

  std::string port() const
  {
    return std::to_string(acceptor_.local_endpoint().port());
  }

  /// Call after the adapter's openSocket()
  void accept()
  {
    acceptor_.accept(socket_);
  }

  void write(const std::vector<uint8_t> & data)
  {
    boost::asio::write(socket_, boost::asio::buffer(data));
  }

  std::vector<uint8_t> read(size_t size)
  {
    std::vector<uint8_t> data(size);
    boost::asio::read(socket_, boost::asio::buffer(data));
    return data;
  }

private:
  boost::asio::io_context io_context_;
  tcp::acceptor acceptor_;
  tcp::socket socket_;
};

}  // namespace

TEST_CASE("AxiomaticAdapter delivers raw data and CAN frames split across TCP reads", "[adapter][ax142100a]")
{
  FakeConverter converter;

  std::promise<CanFrame> frame_promise;
  std::promise<std::vector<uint8_t>> raw_data_promise;
  std::mutex errors_mutex;
  std::vector<std::string> errors;

  AxiomaticAdapter adapter(
    "127.0.0.1",
    converter.port(),
    [&](std::unique_ptr<const CanFrame> frame) { frame_promise.set_value(*frame); },
    [&](AxiomaticAdapter::socket_error_string_t error) {
      std::lock_guard<std::mutex> guard(errors_mutex);
      errors.push_back(error);
    },
    AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
    true,
    std::make_unique<polymath::can::ax142100a::Ax142100a>());
  REQUIRE(adapter.setOnRawDataCallback([&](std::vector<uint8_t> data) { raw_data_promise.set_value(data); }));

  REQUIRE(adapter.openSocket());
  converter.accept();
  REQUIRE(adapter.startReceptionThread());
  REQUIRE_FALSE(adapter.setOnRawDataCallback([](std::vector<uint8_t> /*data*/) {}));

  const std::vector<uint8_t> raw_data = {'$', 'G', 'S', ',', '1', '\r', '\n'};
  std::vector<uint8_t> stream = *polymath::can::ax142100a::Ax142100a().encodeRawData(raw_data);
  stream.insert(stream.end(), AX142100A_MANUAL_EXAMPLE.begin(), AX142100A_MANUAL_EXAMPLE.end());
  const size_t split = stream.size() - 5;
  converter.write(std::vector<uint8_t>(stream.begin(), stream.begin() + split));
  std::this_thread::sleep_for(SPLIT_WRITE_GAP);
  converter.write(std::vector<uint8_t>(stream.begin() + split, stream.end()));

  auto raw_data_future = raw_data_promise.get_future();
  REQUIRE(std::future_status::ready == raw_data_future.wait_for(RECEIVE_WAIT));
  REQUIRE(raw_data == raw_data_future.get());

  auto frame_future = frame_promise.get_future();
  REQUIRE(std::future_status::ready == frame_future.wait_for(RECEIVE_WAIT));
  const CanFrame frame = frame_future.get();
  REQUIRE(0x18F00401 == frame.get_id());
  REQUIRE(8 == frame.get_len());

  REQUIRE(adapter.joinReceptionThread());
  std::lock_guard<std::mutex> guard(errors_mutex);
  for (const auto & error : errors) {
    INFO(error);
    REQUIRE("Receive operation timed out" == error);
  }
}

TEST_CASE("AxiomaticAdapter sends raw data as one protocol message", "[adapter][ax142100a]")
{
  FakeConverter converter;
  AxiomaticAdapter adapter(
    "127.0.0.1",
    converter.port(),
    [](std::unique_ptr<const CanFrame> /*frame*/) {},
    [](AxiomaticAdapter::socket_error_string_t /*error*/) {},
    AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
    true,
    std::make_unique<polymath::can::ax142100a::Ax142100a>());
  REQUIRE(adapter.supportsRawData());
  REQUIRE(adapter.openSocket());
  converter.accept();

  const std::vector<uint8_t> raw_data = {'h', 'e', 'l', 'l', 'o'};
  const std::vector<uint8_t> expected = *polymath::can::ax142100a::Ax142100a().encodeRawData(raw_data);
  REQUIRE_FALSE(adapter.sendRawData(raw_data).has_value());
  REQUIRE(expected == converter.read(expected.size()));
}

TEST_CASE("AxiomaticAdapter refuses raw data for a model without a raw data channel", "[adapter][ax140900]")
{
  AxiomaticAdapter adapter(
    "127.0.0.1",
    "4000",
    [](std::unique_ptr<const CanFrame> /*frame*/) {},
    [](AxiomaticAdapter::socket_error_string_t /*error*/) {},
    AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
    true,
    std::make_unique<polymath::can::ax140900::Ax140900>());
  REQUIRE_FALSE(adapter.supportsRawData());
  REQUIRE(adapter.sendRawData({'a'}).has_value());
}
