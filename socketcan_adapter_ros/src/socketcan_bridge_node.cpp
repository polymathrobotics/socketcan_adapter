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

#include "socketcan_adapter_ros/socketcan_bridge_node.hpp"

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace polymath
{

namespace socketcan
{

SocketcanBridgeNode::SocketcanBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("socketcan_bridge", "", options)
{
  declare_parameter("can_interface", std::string("can0"));
  declare_parameter("can_error_mask", static_cast<int32_t>(CAN_ERR_MASK));
  declare_parameter("can_filter_list", std::vector<std::string>{"0:0"});  // vector of strings
  declare_parameter("join_filters", false);
  declare_parameter("receive_timeout_s", SOCKET_RECEIVE_TIMEOUT_S.count());
}

SocketcanBridgeNode::~SocketcanBridgeNode()
{
  on_deactivate(get_current_state());
  on_cleanup(get_current_state());
}

SocketcanBridgeNode::rclcpp_lifecycle_callback_return SocketcanBridgeNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  std::string interface_name = get_parameter("can_interface").as_string();
  float receive_timeout_s = get_parameter("receive_timeout_s").as_double();
  std::vector<std::string> filter_list = get_parameter("can_filter_list").as_string_array();
  can_err_mask_t error_mask = get_parameter("can_error_mask").as_int();
  bool join = get_parameter("join_filters").as_bool();

  auto filter_vector = stringVectorToFilterVector(filter_list);

  socketcan_adapter_ =
    std::make_unique<SocketcanAdapter>(interface_name, std::chrono::duration<float>(receive_timeout_s));

  frame_publisher_ = create_publisher<can_msgs::msg::Frame>("can_rx", 10);
  frame_subscriber_ = create_subscription<can_msgs::msg::Frame>(
    "can_tx", 10, std::bind(&SocketcanBridgeNode::transmitCanFrame, this, std::placeholders::_1));

  bool success;
  success = socketcan_adapter_->openSocket();

  if (!success) {
    /// TODO: should this be error?
    RCLCPP_ERROR(get_logger(), "Failed to open socket");
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  auto result = socketcan_adapter_->setFilters(filter_vector);

  // If it's not null, this means that we have an error string and failed
  if (result) {
    auto string_result = *result;
    RCLCPP_ERROR(get_logger(), "Setting filters failed with error %s", string_result.c_str());
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  result = socketcan_adapter_->setErrorMaskOverwrite(error_mask);

  if (result) {
    auto string_result = *result;
    RCLCPP_ERROR(get_logger(), "Setting error mask failed with error %s", string_result.c_str());
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  result = socketcan_adapter_->setJoinOverwrite(join);

  if (result) {
    auto string_result = *result;
    RCLCPP_ERROR(get_logger(), "Setting join failed with error %s", string_result.c_str());
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  socketcan_adapter_->setOnReceiveCallback(
    std::bind(&SocketcanBridgeNode::publishCanFrame, this, std::placeholders::_1));

  socketcan_adapter_->setOnErrorCallback(
    std::bind(&SocketcanBridgeNode::socketErrorCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Finished configuration!");

  return LifecycleNode::on_configure(state);
}

SocketcanBridgeNode::rclcpp_lifecycle_callback_return SocketcanBridgeNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (socketcan_adapter_->get_socket_state() != SocketState::OPEN) {
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  socketcan_adapter_->startReceptionThread();

  if (!socketcan_adapter_->is_thread_running()) {
    return rclcpp_lifecycle_callback_return::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Finished activation!");
  return LifecycleNode::on_activate(state);
}

SocketcanBridgeNode::rclcpp_lifecycle_callback_return SocketcanBridgeNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  auto result = socketcan_adapter_->joinReceptionThread();

  if (!result) {
    RCLCPP_ERROR(get_logger(), "Failed to join socket thread");
    return rclcpp_lifecycle_callback_return::ERROR;
  }

  return LifecycleNode::on_deactivate(state);
}

SocketcanBridgeNode::rclcpp_lifecycle_callback_return SocketcanBridgeNode::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  auto result = socketcan_adapter_->closeSocket();

  if (!result) {
    RCLCPP_ERROR(get_logger(), "Failed to close socket");
  }

  frame_subscriber_.reset();
  frame_publisher_.reset();
  socketcan_adapter_.reset();

  return LifecycleNode::on_cleanup(state);
}

SocketcanBridgeNode::rclcpp_lifecycle_callback_return SocketcanBridgeNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  return LifecycleNode::on_shutdown(state);
}

SocketcanAdapter::filter_vector_t SocketcanBridgeNode::stringVectorToFilterVector(
  const std::vector<std::string> & filter_vector)
{
  SocketcanAdapter::filter_vector_t filter_list;

  struct can_filter can_filter{};

  for (std::string filter : filter_vector) {
    if (std::sscanf(filter.c_str(), "%x:%x", &can_filter.can_id, &can_filter.can_mask) == 2) {
      can_filter.can_mask &= ~CAN_ERR_FLAG;
      if ((filter.size() > 8) && (filter.at(8) == ':')) {
        can_filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.emplace_back(can_filter);
    } else if (std::sscanf(filter.c_str(), "%x~%x", &can_filter.can_id, &can_filter.can_mask) == 2) {
      can_filter.can_id |= CAN_INV_FILTER;
      can_filter.can_mask &= ~CAN_ERR_FLAG;
      if (filter.size() > 8 && filter[8] == '~') {
        can_filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.emplace_back(can_filter);
    } else {
      RCLCPP_INFO(get_logger(), "Bad Filter passed in %s, ignoring", filter.c_str());
    }
  }

  return filter_list;
}

void SocketcanBridgeNode::publishCanFrame(std::unique_ptr<const CanFrame> frame)
{
  std::unique_ptr<can_msgs::msg::Frame> publishable_frame = std::make_unique<can_msgs::msg::Frame>();

  publishable_frame->header.stamp = get_clock()->now();
  publishable_frame->id = frame->get_id();
  publishable_frame->dlc = frame->get_len();
  publishable_frame->is_error = frame->get_frame_type() == FrameType::ERROR;
  publishable_frame->is_rtr = frame->get_frame_type() == FrameType::REMOTE;
  publishable_frame->is_extended = frame->get_id_type() == IdType::EXTENDED;
  publishable_frame->data = frame->get_data();

  frame_publisher_->publish(std::move(publishable_frame));
}

void SocketcanBridgeNode::socketErrorCallback(SocketcanAdapter::socket_error_string_t error)
{
  RCLCPP_ERROR(get_logger(), "Socket error received: %s", error.c_str());
}

void SocketcanBridgeNode::transmitCanFrame(can_msgs::msg::Frame::UniquePtr frame)
{
  if (socketcan_adapter_->get_socket_state() != SocketState::OPEN) {
    return;
  }

  /// TODO: Switch to unique_ptr
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  auto can_frame = std::make_shared<CanFrame>();

  /// TODO: Unnecessary branching, can we turn these functions into bools or something?
  /// https://gitlab.com/polymathrobotics/polymath_core/-/issues/8
  if (frame->is_extended) {
    can_frame->set_id_as_extended();
  }

  if (frame->is_error) {
    can_frame->set_type_error();
  }

  if (frame->is_rtr) {
    can_frame->set_type_remote();
  }

  can_frame->set_len(frame->dlc);
  can_frame->set_data(frame->data);
  can_frame->set_can_id(frame->id);
  socketcan_adapter_->send(can_frame);
}

}  // namespace socketcan
}  // namespace polymath
