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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_SOCKETCAN_BRIDGE_NODE_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_SOCKETCAN_BRIDGE_NODE_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "axiomatic_adapter/axiomatic_adapter.hpp"
#include "axiomatic_adapter/models/ax140900.hpp"
#include "axiomatic_adapter/serial_port.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

namespace polymath::can
{

/// @class polymath::can::AxiomaticSocketcanBridge
/// @brief This class will bridge a socketcan connection and read/write to the can-eth device tcp connection.
/// Optionally bridges the device's raw (serial) data to a local serial port.
class AxiomaticSocketcanBridge
{
public:
  /// @brief Constructor
  /// @param can_interface_name name of the can interface for socketcan
  /// @param ip IP address of the ethernet can device
  /// @param port port for the ethernet can device
  /// @param verbose enables printing of debug logs if true. Defaults to false
  /// @param tcp_nodelay when true (default), sets TCP_NODELAY on the underlying
  ///                    AxiomaticAdapter socket. See AxiomaticAdapter for the
  ///                    full tradeoff discussion.
  /// @param model converter model, e.g. std::make_unique<ax142100a::Ax142100a>(); must not be null
  /// @param serial_port_path when set, existing serial device bridged to the model's raw data, e.g. /dev/ttyAX0
  AxiomaticSocketcanBridge(
    const std::string & can_interface_name,
    const std::string & ip,
    const std::string & port,
    bool verbose = false,
    bool tcp_nodelay = true,
    std::unique_ptr<const AxiomaticModel> model = std::make_unique<ax140900::Ax140900>(),
    const std::optional<std::string> & serial_port_path = std::nullopt);

  /// @brief Destruct axiomatic socketcan bridge
  ~AxiomaticSocketcanBridge();

  /// @brief configures and opens the TCP and CAN sockets, and the serial port when a path was given
  /// @return success for opening all of them; false when a serial port is configured for a model without raw data
  bool on_configure();

  /// @brief activates and starts the reception threads for the socketcan and axiomatic adapters
  /// @return success for starting both threads
  bool on_activate();

  /// @brief joins the reception threads to the main thread on deactivate
  /// @return success for joining bpth threads
  bool on_deactivate();

  /// @brief Shuts down and closes the TCP and CAN sockets, and the serial port
  /// @return success for closing both sockets
  bool on_shutdown();

private:
  polymath::socketcan::SocketcanAdapter socketcan_adapter_;
  polymath::can::AxiomaticAdapter axiomatic_adapter_;
  std::optional<std::string> serial_port_path_;
  // open from on_configure until on_shutdown
  std::unique_ptr<SerialPort> serial_port_;

  bool verbose_;

  void socketcanReceiveCallback(std::unique_ptr<const polymath::socketcan::CanFrame> frame);
  void ethcanReceiveCallback(std::unique_ptr<const polymath::socketcan::CanFrame> frame);
  void serialPortReceiveCallback(std::vector<uint8_t> data);
  void ethRawDataReceiveCallback(std::vector<uint8_t> data);
};

}  // namespace polymath::can

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_SOCKETCAN_BRIDGE_NODE_HPP_
