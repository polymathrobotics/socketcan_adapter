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

#ifndef AXIOMATIC_ADAPTER__SERIAL_PORT_HPP_
#define AXIOMATIC_ADAPTER__SERIAL_PORT_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace polymath::can
{

/// @class polymath::can::SerialPort
/// @brief An existing serial device, e.g. one end of a virtual serial port pair, open in raw mode for the object's lifetime.
/// send() may run concurrently with the reception thread.
class SerialPort
{
public:
  using error_string_t = std::string;

  static constexpr std::chrono::milliseconds RECEIVE_POLL_TIMEOUT_MS{100};
  static constexpr std::chrono::milliseconds JOIN_RECEPTION_TIMEOUT_MS{500};

  /// @brief Open device_path and set it to raw mode
  /// @param device_path serial device to open, e.g. /dev/ttyAX0
  /// @throws std::system_error when the device is missing or not a terminal
  explicit SerialPort(const std::string & device_path);

  /// @brief Joins the reception thread and closes the device
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// @brief Set the callback for each chunk of bytes read from the device
  /// @return false while the reception thread is running
  bool setOnReceiveCallback(std::function<void(std::vector<uint8_t> data)> && callback_function);

  /// @brief Set the callback for read errors and hang-ups; a repeated error is reported once until a read succeeds
  /// @return false while the reception thread is running
  bool setOnErrorCallback(std::function<void(error_string_t error)> && callback_function);

  /// @brief Start the thread that calls the receive and error callbacks
  /// @return false when already running
  bool startReceptionThread();

  /// @brief Stop and join the reception thread
  /// @return success on joined thread within timeout
  bool joinReceptionThread(const std::chrono::milliseconds & timeout = JOIN_RECEPTION_TIMEOUT_MS);

  /// @brief Write bytes to the device
  /// @return error message; bytes not yet written are dropped, e.g. when nothing drains a full buffer
  std::optional<error_string_t> send(const std::vector<uint8_t> & data);

  const std::string & devicePath() const;

private:
  std::string device_path_;
  std::function<void(std::vector<uint8_t> data)> receive_callback_;
  std::function<void(error_string_t error)> error_callback_;

  int fd_;

  std::thread reception_thread_;
  std::atomic<bool> thread_running_{false};
  std::atomic<bool> stop_thread_requested_{false};
};

}  // namespace polymath::can

#endif  // AXIOMATIC_ADAPTER__SERIAL_PORT_HPP_
