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

#include "axiomatic_adapter/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <future>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace polymath::can
{

namespace
{
constexpr size_t RECEIVE_BUFFER_SIZE = 4096;

std::string errnoMessage(const std::string & call)
{
  return call + " failed: " + std::strerror(errno);
}

/// Open device_path in raw mode, nonblocking
/// @throws std::system_error on failure
int openRaw(const std::string & device_path)
{
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (0 > fd) {
    throw std::system_error(errno, std::generic_category(), "open " + device_path);
  }
  struct termios attributes;
  if (0 != ::tcgetattr(fd, &attributes)) {
    const int error = errno;
    ::close(fd);
    throw std::system_error(error, std::generic_category(), "tcgetattr " + device_path);
  }
  ::cfmakeraw(&attributes);
  if (0 != ::tcsetattr(fd, TCSANOW, &attributes)) {
    const int error = errno;
    ::close(fd);
    throw std::system_error(error, std::generic_category(), "tcsetattr " + device_path);
  }
  return fd;
}
}  // namespace

SerialPort::SerialPort(const std::string & device_path)
: device_path_(device_path)
, receive_callback_([](std::vector<uint8_t> /*data*/) { /*do nothing*/ })
, error_callback_([](error_string_t /*error*/) { /*do nothing*/ })
, fd_(openRaw(device_path))
{}

SerialPort::~SerialPort()
{
  joinReceptionThread();
  ::close(fd_);
}

bool SerialPort::setOnReceiveCallback(std::function<void(std::vector<uint8_t> data)> && callback_function)
{
  if (thread_running_) {
    return false;
  }
  receive_callback_ = std::move(callback_function);
  return true;
}

bool SerialPort::setOnErrorCallback(std::function<void(error_string_t error)> && callback_function)
{
  if (thread_running_) {
    return false;
  }
  error_callback_ = std::move(callback_function);
  return true;
}

bool SerialPort::startReceptionThread()
{
  if (reception_thread_.joinable()) {
    return false;
  }
  stop_thread_requested_ = false;
  thread_running_ = true;
  reception_thread_ = std::thread([this]() {
    std::vector<uint8_t> buffer(RECEIVE_BUFFER_SIZE);
    std::optional<error_string_t> last_error;
    while (!stop_thread_requested_) {
      struct pollfd poll_fd = {fd_, POLLIN, 0};
      if (0 >= ::poll(&poll_fd, 1, static_cast<int>(RECEIVE_POLL_TIMEOUT_MS.count()))) {
        continue;
      }
      const ssize_t bytes_read = ::read(fd_, buffer.data(), buffer.size());
      if (0 < bytes_read) {
        last_error.reset();
        receive_callback_(std::vector<uint8_t>(buffer.begin(), buffer.begin() + bytes_read));
        continue;
      }
      if (0 > bytes_read && (EAGAIN == errno || EINTR == errno)) {
        continue;
      }
      const error_string_t error = 0 == bytes_read ? device_path_ + " hung up" : errnoMessage("read " + device_path_);
      if (error != last_error) {
        error_callback_(error);
        last_error = error;
      }
      // A hung-up device (e.g. the other end of a virtual pair closed) polls ready forever.
      std::this_thread::sleep_for(RECEIVE_POLL_TIMEOUT_MS);
    }
    thread_running_ = false;
  });
  return true;
}

bool SerialPort::joinReceptionThread(const std::chrono::milliseconds & timeout)
{
  stop_thread_requested_ = true;
  if (!reception_thread_.joinable()) {
    return false;
  }
  std::future<void> join_future = std::async(std::launch::async, [this] { reception_thread_.join(); });
  return join_future.wait_for(timeout) == std::future_status::ready;
}

std::optional<SerialPort::error_string_t> SerialPort::send(const std::vector<uint8_t> & data)
{
  size_t written = 0;
  while (written < data.size()) {
    const ssize_t result = ::write(fd_, data.data() + written, data.size() - written);
    if (0 > result) {
      if (EINTR == errno) {
        continue;
      }
      return errnoMessage("write " + device_path_) + "; dropped " + std::to_string(data.size() - written) + " bytes";
    }
    written += static_cast<size_t>(result);
  }
  return std::nullopt;
}

const std::string & SerialPort::devicePath() const
{
  return device_path_;
}

}  // namespace polymath::can
