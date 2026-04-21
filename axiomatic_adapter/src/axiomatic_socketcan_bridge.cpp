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

#include "axiomatic_adapter/axiomatic_socketcan_bridge.hpp"

#include <iostream>
#include <string>

namespace polymath
{
namespace can
{

AxiomaticSocketcanBridge::AxiomaticSocketcanBridge(
  const std::string & can_interface_name, const std::string & ip, const std::string & port, bool verbose)
: socketcan_adapter_(can_interface_name)
, axiomatic_adapter_(ip, port, std::bind(&AxiomaticSocketcanBridge::ethcanReceiveCallback, this, std::placeholders::_1))
, verbose_(verbose)
{
  socketcan_adapter_.setOnReceiveCallback(
    std::bind(&AxiomaticSocketcanBridge::socketcanReceiveCallback, this, std::placeholders::_1));
}

AxiomaticSocketcanBridge::~AxiomaticSocketcanBridge()
{
  on_deactivate();
  on_shutdown();
}

bool AxiomaticSocketcanBridge::on_configure()
{
  // open sockets
  if (!socketcan_adapter_.openSocket()) {
    std::cout << "Socketcan Adapter can't open socket..." << std::endl;
    return false;
  }
  if (!axiomatic_adapter_.openSocket()) {
    std::cout << "Axiomatic Adapter can't open socket..." << std::endl;
    return false;
  }
  return true;
}

bool AxiomaticSocketcanBridge::on_activate()
{
  if (!socketcan_adapter_.startReceptionThread()) {
    return false;
  }
  if (!axiomatic_adapter_.startReceptionThread()) {
    return false;
  }
  return true;
}

bool AxiomaticSocketcanBridge::on_deactivate()
{
  bool success = true;
  if (!socketcan_adapter_.joinReceptionThread()) {
    success = false;
  }
  if (!axiomatic_adapter_.joinReceptionThread()) {
    success = false;
  }
  return success;
}

bool AxiomaticSocketcanBridge::on_shutdown()
{
  bool success = true;
  if (!socketcan_adapter_.closeSocket()) {
    success = false;
  }
  if (!axiomatic_adapter_.closeSocket()) {
    success = false;
  }
  return success;
}

void AxiomaticSocketcanBridge::socketcanReceiveCallback(std::unique_ptr<const polymath::socketcan::CanFrame> frame)
{
  if (!frame) {
    return;
  }
  auto frame_copy = polymath::socketcan::CanFrame(*frame);

  if (verbose_) {
    std::cout << "[SocketCAN RX] Received CAN Frame: ID = " << std::hex << frame_copy.get_id() << ", Data = [";

    auto data = frame_copy.get_data();
    for (size_t i = 0; i < data.size(); ++i) {
      std::cout << " " << std::hex << static_cast<int>(data[i]);
    }
    std::cout << " ]" << std::endl;
  }

  auto result = axiomatic_adapter_.send(frame_copy);
  if (!result) {
    std::cerr << "[SocketCAN RX] Failed to send CAN frame with message: " << *result << std::endl;
  }
}

void AxiomaticSocketcanBridge::ethcanReceiveCallback(std::unique_ptr<const polymath::socketcan::CanFrame> frame)
{
  if (!frame) {
    return;
  }
  auto frame_copy = polymath::socketcan::CanFrame(*frame);

  if (verbose_) {
    std::cout << "[EthCAN RX] Received CAN Frame: ID = " << std::hex << frame_copy.get_id() << ", Data = [";

    auto data = frame_copy.get_data();
    for (size_t i = 0; i < data.size(); ++i) {
      std::cout << " " << std::hex << static_cast<int>(data[i]);
    }
    std::cout << " ]" << std::endl;
  }

  auto result = socketcan_adapter_.send(frame_copy);
  if (!result) {
    std::cerr << "[EthCAN RX] Failed to send CAN frame with message: " << *result << std::endl;
  }
}

}  // namespace can
}  // namespace polymath
