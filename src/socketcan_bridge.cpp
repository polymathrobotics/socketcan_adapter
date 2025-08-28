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

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "socketcan_adapter/socketcan_bridge_node.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = std::make_shared<polymath::socketcan::SocketcanBridgeNode>();
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node->get_node_base_interface());

  node->configure();
  node->activate();

  executor.spin();

  node->deactivate();
  node->cleanup();
  node->shutdown();

  rclcpp::shutdown();

  return 0;
}
