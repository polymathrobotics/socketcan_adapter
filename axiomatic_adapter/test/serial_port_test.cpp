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

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#if __has_include(<catch2/catch_all.hpp>)
  #include <catch2/catch_all.hpp>  // v3
#else
  #include <catch2/catch.hpp>  // v2
#endif
#include "axiomatic_adapter/serial_port.hpp"

using polymath::can::SerialPort;

namespace
{

constexpr std::chrono::seconds RECEIVE_WAIT{2};
constexpr int PEER_POLL_TIMEOUT_MS = 2000;
constexpr std::chrono::milliseconds HANG_UP_WAIT{500};

/// A pseudo-terminal pair: SerialPort opens device_path, the test plays the other end through peer_fd
class VirtualSerialPair
{
public:
  VirtualSerialPair()
  : peer_fd_(::posix_openpt(O_RDWR | O_NOCTTY))
  {
    REQUIRE(0 <= peer_fd_);
    REQUIRE(0 == ::grantpt(peer_fd_));
    REQUIRE(0 == ::unlockpt(peer_fd_));
    device_path_ = ::ptsname(peer_fd_);
  }

  ~VirtualSerialPair()
  {
    ::close(peer_fd_);
  }

  const std::string & devicePath() const
  {
    return device_path_;
  }

  int fd() const
  {
    return peer_fd_;
  }

  void write(const std::vector<uint8_t> & data)
  {
    REQUIRE(static_cast<ssize_t>(data.size()) == ::write(peer_fd_, data.data(), data.size()));
  }

  std::vector<uint8_t> read(size_t size)
  {
    std::vector<uint8_t> data;
    while (data.size() < size) {
      struct pollfd poll_fd = {peer_fd_, POLLIN, 0};
      REQUIRE(0 < ::poll(&poll_fd, 1, PEER_POLL_TIMEOUT_MS));
      std::vector<uint8_t> chunk(size - data.size());
      const ssize_t bytes_read = ::read(peer_fd_, chunk.data(), chunk.size());
      REQUIRE(0 < bytes_read);
      data.insert(data.end(), chunk.begin(), chunk.begin() + bytes_read);
    }
    return data;
  }

private:
  int peer_fd_;
  std::string device_path_;
};

}  // namespace

TEST_CASE("SerialPort closes the device on destruction", "[serial_port]")
{
  VirtualSerialPair pair;
  {
    SerialPort port(pair.devicePath());
  }
  // The peer sees a hang-up once no one holds the device open.
  struct pollfd poll_fd = {pair.fd(), POLLIN, 0};
  REQUIRE(0 < ::poll(&poll_fd, 1, PEER_POLL_TIMEOUT_MS));
  REQUIRE(0 != (poll_fd.revents & POLLHUP));
}

TEST_CASE("SerialPort passes bytes both ways in raw mode", "[serial_port]")
{
  VirtualSerialPair pair;
  const std::vector<uint8_t> from_peer = {'$', 'G', 'S', ',', '1', '\r', '\n'};
  std::vector<uint8_t> received;
  std::promise<std::vector<uint8_t>> read_promise;
  SerialPort port(pair.devicePath());
  REQUIRE(port.setOnReceiveCallback([&](std::vector<uint8_t> data) {
    received.insert(received.end(), data.begin(), data.end());
    if (received.size() == from_peer.size()) {
      read_promise.set_value(received);
    }
  }));

  REQUIRE(port.startReceptionThread());
  REQUIRE_FALSE(port.setOnReceiveCallback([](std::vector<uint8_t> /*data*/) {}));
  REQUIRE_FALSE(port.setOnErrorCallback([](SerialPort::error_string_t /*error*/) {}));

  pair.write(from_peer);
  auto read_future = read_promise.get_future();
  REQUIRE(std::future_status::ready == read_future.wait_for(RECEIVE_WAIT));
  REQUIRE(from_peer == read_future.get());

  const std::vector<uint8_t> to_peer = {0x00, 0x0A, 0x0D, 0xFF, 'o', 'k'};
  REQUIRE_FALSE(port.send(to_peer).has_value());
  REQUIRE(to_peer == pair.read(to_peer.size()));

  REQUIRE(port.joinReceptionThread());
}

TEST_CASE("SerialPort reports a hang-up once", "[serial_port]")
{
  auto pair = std::make_unique<VirtualSerialPair>();
  SerialPort port(pair->devicePath());
  std::mutex errors_mutex;
  std::vector<SerialPort::error_string_t> errors;
  REQUIRE(port.setOnErrorCallback([&](SerialPort::error_string_t error) {
    std::lock_guard<std::mutex> guard(errors_mutex);
    errors.push_back(error);
  }));
  REQUIRE(port.startReceptionThread());

  pair.reset();
  std::this_thread::sleep_for(HANG_UP_WAIT);

  REQUIRE(port.joinReceptionThread());
  std::lock_guard<std::mutex> guard(errors_mutex);
  REQUIRE(1 == errors.size());
}

TEST_CASE("SerialPort throws for a missing device", "[serial_port]")
{
  REQUIRE_THROWS_AS(SerialPort("/dev/nonexistent_serial_port_test"), std::system_error);
}

TEST_CASE("SerialPort throws for a file that is not a terminal", "[serial_port]")
{
  const auto path = std::filesystem::temp_directory_path() /
                    ("serial_port_test_" + std::to_string(::getpid()) + "_" + std::to_string(::rand()));
  std::ofstream(path) << "not a tty";

  REQUIRE_THROWS_AS(SerialPort(path.string()), std::system_error);
  std::filesystem::remove(path);
}
