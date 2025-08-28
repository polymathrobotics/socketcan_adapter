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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace polymath::socketcan
{

SocketcanAdapter::SocketcanAdapter(
  const std::string & interface_name, const std::chrono::duration<float> & receive_timeout_s)
: interface_name_(interface_name)
, receive_timeout_s_(receive_timeout_s)
, receive_callback_unique_ptr_([](std::unique_ptr<const CanFrame> /*frame*/) { /*do nothing*/ })
, receive_error_callback_([](socket_error_string_t /*error*/) { /*do nothing*/ })
, socket_state_(SocketState::CLOSED)
{}

SocketcanAdapter::~SocketcanAdapter()
{
  closeSocket();
}

bool SocketcanAdapter::openSocket()
{
  struct sockaddr_can addr{};
  socket_file_descriptor_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (socket_file_descriptor_ < 0) {
    return false;
  }

  auto if_index = if_nametoindex(interface_name_.c_str());

  addr.can_family = AF_CAN;
  addr.can_ifindex = if_index;

  if (bind(socket_file_descriptor_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    closeSocket();
    return false;
  }

  socket_state_ = SocketState::OPEN;

  /// TODO: Add CAN FD Support

  return true;
}

bool SocketcanAdapter::closeSocket()
{
  if (socket_file_descriptor_ != CLOSED_SOCKET_VALUE) {
    bool success = close(socket_file_descriptor_);
    socket_state_ = SocketState::CLOSED;
    socket_file_descriptor_ = CLOSED_SOCKET_VALUE;

    return success == 0;
  }

  return false;
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::setFilters(
  const filter_vector_t & filters, FilterMode /*mode*/)
{
  /// TODO: mode unused
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/6
  filter_list_ = filters;
  return sendFilters();
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::setFilters(
  const std::shared_ptr<filter_vector_t> filters, FilterMode /*mode*/)
{
  /// TODO: mode unused
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/6
  filter_list_ = *filters;
  return sendFilters();
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::setErrorMaskOverwrite(
  const can_err_mask_t & error_mask)
{
  error_mask_ = error_mask;
  return sendErrorMask();
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::setJoinOverwrite(const bool & join)
{
  join_ = join;
  return sendJoin();
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::receive(CanFrame & polymath_can_frame)
{
  struct pollfd fds[1];
  struct can_frame frame{};

  fds[0].fd = socket_file_descriptor_;
  fds[0].events = POLLIN | POLLERR;

  /// TODO: We don't need to call duration cast every time we run this, we should store milliseconds instead
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  int return_value = poll(
    fds, NUM_SOCKETS_IN_ADAPTER, std::chrono::duration_cast<std::chrono::milliseconds>(receive_timeout_s_).count());

  socket_error_string_t error_string;

  if (return_value < 0) {
    socket_error_string_t err_string = socket_error_string_t("poll failed with error") + strerror(errno);
    return std::optional<socket_error_string_t>(err_string);
  }

  if (return_value == 0) {
    return std::optional<socket_error_string_t>("poll timed out with no data");
  }

  if (fds[0].revents & POLLERR) {
    int32_t error;
    socklen_t len = sizeof(error);
    if (getsockopt(socket_file_descriptor_, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
      error_string += socket_error_string_t("socket errored but failed to get the error\n");
    } else {
      error_string +=
        socket_error_string_t("Socket error: ") + socket_error_string_t(strerror(error) + socket_error_string_t("\n"));
    }
  }

  if (fds[0].revents & POLLIN) {
    int32_t num_bytes = read(socket_file_descriptor_, &frame, sizeof(struct can_frame));

    if (num_bytes < 0) {
      error_string += socket_error_string_t("Failed to read socket\n");
    } else if (static_cast<size_t>(num_bytes) < sizeof(struct can_frame)) {
      error_string += socket_error_string_t("Incomplete Can Frame\n");
    }

    if (frame.can_id & CAN_ERR_FLAG) {
      error_string += socket_error_string_t("Error frame received\n");
    }

    struct timeval tv;
    ioctl(socket_file_descriptor_, SIOCGSTAMP, &tv);

    uint64_t timestamp_uint64 = static_cast<uint64_t>(tv.tv_sec) * 1e6 + tv.tv_usec;
    polymath_can_frame.set_frame(frame);
    polymath_can_frame.set_timestamp(timestamp_uint64);
  }

  return error_string.empty() ? std::nullopt : std::optional<socket_error_string_t>(error_string);
}

std::optional<const CanFrame> SocketcanAdapter::receive()
{
  CanFrame can_frame = CanFrame();

  auto result = receive(can_frame);

  return !result ? std::optional<const CanFrame>(can_frame) : std::nullopt;
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::receive(std::shared_ptr<CanFrame> frame)
{
  CanFrame can_frame = CanFrame();
  auto result = receive(can_frame);

  *frame = can_frame;

  return result;
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::send(const CanFrame & frame)
{
  struct can_frame raw_frame = frame.get_frame();

  constexpr int32_t flags = 0;

  /// TODO: Add Timeout verification in case we need it for multithreading as in ros2_socketcan
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  const auto bytes_sent = ::send(socket_file_descriptor_, &raw_frame, sizeof(struct can_frame), flags);

  return (bytes_sent < 0)
           ? std::optional<socket_error_string_t>(std::string("socket send failed: ") + std::strerror(errno))
           : std::nullopt;
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::send(
  const std::shared_ptr<const CanFrame> frame)
{
  struct can_frame raw_frame = frame->get_frame();

  constexpr int32_t flags = 0;

  /// TODO: Add Timeout verification in case we need it for multithreading as in ros2_socketcan
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  const auto bytes_sent = ::send(socket_file_descriptor_, &raw_frame, sizeof(struct can_frame), flags);

  return (bytes_sent < 0)
           ? std::optional<socket_error_string_t>(std::string("socket send failed: ") + std::strerror(errno))
           : std::nullopt;
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::send(const can_frame & frame)
{
  return send(polymath::socketcan::CanFrame(frame));
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::sendFilters()
{
  socket_error_string_t error_output("");

  if (
    0 != setsockopt(
           socket_file_descriptor_,
           SOL_CAN_RAW,
           CAN_RAW_FILTER,
           filter_list_.empty() ? NULL : filter_list_.data(),
           sizeof(struct can_filter) * filter_list_.size()))
  {
    error_output += socket_error_string_t("Failed to send CAN filters: ") + socket_error_string_t(strerror(errno));
  }

  return error_output.empty() ? std::nullopt : std::optional<socket_error_string_t>(error_output);
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::sendErrorMask()
{
  socket_error_string_t error_output("");

  if (0 != setsockopt(socket_file_descriptor_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask_, sizeof(can_err_mask_t))) {
    error_output += socket_error_string_t("Failed to send Error Mask: ") + socket_error_string_t(strerror(errno));
  }

  return error_output.empty() ? std::nullopt : std::optional<socket_error_string_t>(error_output);
}

std::optional<SocketcanAdapter::socket_error_string_t> SocketcanAdapter::sendJoin()
{
  socket_error_string_t error_output("");
  auto join = static_cast<int32_t>(join_);
  if (0 != setsockopt(socket_file_descriptor_, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &join, sizeof(int32_t))) {
    error_output += socket_error_string_t("Failed to set Join Filter ") + socket_error_string_t(strerror(errno));
  }

  return error_output.empty() ? std::nullopt : std::optional<socket_error_string_t>(error_output);
}

void SocketcanAdapter::set_receive_timeout(const std::chrono::duration<float> & receive_timeout)
{
  receive_timeout_s_ = receive_timeout;
  return;
}

SocketState SocketcanAdapter::get_socket_state()
{
  return socket_state_;
}

bool SocketcanAdapter::is_thread_running()
{
  return thread_running_;
}

bool SocketcanAdapter::setOnReceiveCallback(
  std::function<void(std::unique_ptr<const CanFrame> frame)> && callback_function)
{
  receive_callback_unique_ptr_ = std::move(callback_function);
  return true;
}

bool SocketcanAdapter::setOnErrorCallback(std::function<void(socket_error_string_t error)> && callback_function)
{
  receive_error_callback_ = std::move(callback_function);
  return true;
}

bool SocketcanAdapter::startReceptionThread()
{
  if (socket_state_ == SocketState::CLOSED) {
    return false;
  }

  stop_thread_requested_ = false;
  thread_running_ = true;

  can_receive_thread_ = std::thread([this]() {
    while (!stop_thread_requested_) {
      CanFrame frame = CanFrame();
      std::optional<socket_error_string_t> error = receive(frame);

      if (!error) {
        receive_callback_unique_ptr_(std::make_unique<CanFrame>(frame));
      } else {
        receive_error_callback_(*error);
      }
    }

    thread_running_ = false;
  });

  return true;
}

bool SocketcanAdapter::joinReceptionThread(const std::chrono::duration<float> & timeout_s)
{
  stop_thread_requested_ = true;

  auto start_time = std::chrono::steady_clock::now();
  // Check thread running to tell when to join the thread
  while (thread_running_ && ((std::chrono::steady_clock::now() - start_time) < timeout_s)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Attempt to force joining at this point

  if (can_receive_thread_.joinable()) {
    can_receive_thread_.join();
    return true;
  } else {
    return false;
  }
}

std::string SocketcanAdapter::get_interface()
{
  return interface_name_;
}

}  // namespace polymath::socketcan
