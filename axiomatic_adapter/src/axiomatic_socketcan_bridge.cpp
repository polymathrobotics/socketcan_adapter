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

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace polymath::can
{

namespace
{
std::string hexBytes(const std::vector<uint8_t> & data)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const uint8_t byte : data) {
    stream << ' ' << std::setw(2) << static_cast<int>(byte);
  }
  return stream.str();
}
}  // namespace

AxiomaticSocketcanBridge::AxiomaticSocketcanBridge(
  const std::string & can_interface_name,
  const std::string & ip,
  const std::string & port,
  bool verbose,
  bool tcp_nodelay,
  std::unique_ptr<const AxiomaticModel> model,
  const std::optional<std::string> & serial_port_path)
: socketcan_adapter_(can_interface_name)
, axiomatic_adapter_(
    ip,
    port,
    std::bind(&AxiomaticSocketcanBridge::ethcanReceiveCallback, this, std::placeholders::_1),
    [](AxiomaticAdapter::socket_error_string_t /*error*/) { /* no-op */ },
    AxiomaticAdapter::DEFAULT_SOCKET_RECEIVE_TIMEOUT_MS,
    tcp_nodelay,
    std::move(model))
, serial_port_path_(serial_port_path)
, verbose_(verbose)
{
  socketcan_adapter_.setOnReceiveCallback(
    std::bind(&AxiomaticSocketcanBridge::socketcanReceiveCallback, this, std::placeholders::_1));
  if (serial_port_path_) {
    axiomatic_adapter_.setOnRawDataCallback(
      std::bind(&AxiomaticSocketcanBridge::ethRawDataReceiveCallback, this, std::placeholders::_1));
  }
}

AxiomaticSocketcanBridge::~AxiomaticSocketcanBridge()
{
  on_deactivate();
  on_shutdown();
}

bool AxiomaticSocketcanBridge::on_configure()
{
  if (serial_port_path_ && !axiomatic_adapter_.supportsRawData()) {
    std::cout << "Axiomatic model has no raw data channel for a serial port" << std::endl;
    return false;
  }
  // open sockets
  if (!socketcan_adapter_.openSocket()) {
    std::cout << "Socketcan Adapter can't open socket..." << std::endl;
    return false;
  }
  if (!axiomatic_adapter_.openSocket()) {
    std::cout << "Axiomatic Adapter can't open socket..." << std::endl;
    return false;
  }
  if (serial_port_path_ && !serial_port_) {
    try {
      serial_port_ = std::make_unique<SerialPort>(*serial_port_path_);
    } catch (const std::system_error & e) {
      std::cout << "Serial port can't open: " << e.what() << std::endl;
      return false;
    }
    serial_port_->setOnReceiveCallback(
      std::bind(&AxiomaticSocketcanBridge::serialPortReceiveCallback, this, std::placeholders::_1));
    serial_port_->setOnErrorCallback(
      [](SerialPort::error_string_t error) { std::cerr << "[Serial RX] Error: " << error << std::endl; });
    std::cout << "Serial port " << serial_port_->devicePath() << " opened" << std::endl;
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
  if (serial_port_ && !serial_port_->startReceptionThread()) {
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
  if (serial_port_ && !serial_port_->joinReceptionThread()) {
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
  serial_port_.reset();
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
  if (result) {
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
  if (result) {
    std::cerr << "[EthCAN RX] Failed to send CAN frame with message: " << *result << std::endl;
  }
}

void AxiomaticSocketcanBridge::serialPortReceiveCallback(std::vector<uint8_t> data)
{
  if (verbose_) {
    std::cout << "[Serial RX] Received " << std::dec << data.size() << " bytes: [" << hexBytes(data) << " ]"
              << std::endl;
  }
  auto result = axiomatic_adapter_.sendRawData(data);
  if (result) {
    std::cerr << "[Serial RX] Failed to send raw data with message: " << *result << std::endl;
  }
}

void AxiomaticSocketcanBridge::ethRawDataReceiveCallback(std::vector<uint8_t> data)
{
  if (verbose_) {
    std::cout << "[EthSerial RX] Received " << std::dec << data.size() << " bytes: [" << hexBytes(data) << " ]"
              << std::endl;
  }
  if (!serial_port_) {
    return;
  }
  auto result = serial_port_->send(data);
  if (result) {
    std::cerr << "[EthSerial RX] Failed to write raw data with message: " << *result << std::endl;
  }
}

}  // namespace polymath::can
