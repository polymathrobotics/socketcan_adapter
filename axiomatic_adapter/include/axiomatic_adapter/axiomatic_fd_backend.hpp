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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_FD_BACKEND_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_FD_BACKEND_HPP_

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "axiomatic_adapter/udp_client.hpp"
#include "socketcan_adapter/can_frame.hpp"
#include "socketcan_adapter/i_can_backend.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"  // for SocketState enum

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace polymath
{
namespace can
{

/// @class polymath::can::AxiomaticFdBackend
/// @brief ICanBackend implementation for the Axiomatic AX140970 Dual CAN FD to
/// Ethernet converter (UDP transport).
///
/// Wraps polymath::axiomatic::UdpClient and translates between the Axiomatic
/// wire protocol (envelope + CAN FD Frame Stream, see axiomatic_protocol.hpp)
/// and the polymath::socketcan::CanFrame value type that ICanBackend exposes
/// to callers.
///
/// Classical-CAN ONLY in this revision. CanFrame currently wraps a Linux
/// `struct can_frame` with an 8-byte payload limit. The AX140970 itself
/// supports up to 64-byte CAN FD frames with BRS/ESI, but those cannot fit
/// through CanFrame yet. Until CanFrame is generalized (separate PR), this
/// backend drops inbound frames whose length > 8 or whose FDF flag is set;
/// those drops are counted on `fd_frames_dropped()` so callers can detect
/// the limitation rather than silently lose data.
///
/// Lifecycle mirrors the SocketcanAdapter pattern:
///
///   AxiomaticFdBackend backend(opts);
///   backend.openSocket();                     // resolves + binds UDP socket
///   backend.setOnReceiveCallback(...);        // optional; set before start
///   backend.setOnErrorCallback(...);          // optional
///   backend.startReceptionThread();           // begins delivering callbacks
///   ...
///   backend.joinReceptionThread(timeout);
///   backend.closeSocket();
class AxiomaticFdBackend : public polymath::socketcan::ICanBackend
{
public:
  using socket_error_string_t = polymath::socketcan::ICanBackend::socket_error_string_t;

  struct Options
  {
    std::string device_ip;
    uint16_t    device_port = 4000;
    /// Local channel address advertised in our heartbeats; the device uses
    /// this to filter which CAN frames it forwards to us. Default (0,0) =
    /// NULL filter = accept all.
    polymath::axiomatic::ChannelAddress local_address{0u, 0u};
    /// Destination address for transmitted frames. (CG=0, CIDS=0x1) is CAN1
    /// in the device's default routing; (0, 0x2) is CAN2. Override per-channel.
    polymath::axiomatic::ChannelAddress tx_address{0u, 0x1u};
    std::chrono::milliseconds heartbeat_interval{1000};
  };

  explicit AxiomaticFdBackend(Options opts);
  ~AxiomaticFdBackend() override;

  AxiomaticFdBackend(const AxiomaticFdBackend &) = delete;
  AxiomaticFdBackend & operator=(const AxiomaticFdBackend &) = delete;

  // ---- ICanBackend ----
  bool openSocket() override;
  bool closeSocket() override;
  polymath::socketcan::SocketState get_socket_state() override;

  std::optional<socket_error_string_t> send(
    const std::shared_ptr<const polymath::socketcan::CanFrame> frame) override;
  std::optional<socket_error_string_t> send(
    const polymath::socketcan::CanFrame & frame) override;
  std::optional<socket_error_string_t> receive(
    polymath::socketcan::CanFrame & frame) override;

  bool startReceptionThread() override;
  bool joinReceptionThread(const std::chrono::duration<float> & timeout_s) override;
  bool is_thread_running() override;

  bool setOnReceiveCallback(
    std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> &&
      callback_function) override;
  bool setOnErrorCallback(
    std::function<void(socket_error_string_t error)> && callback_function) override;

  /// @brief Count of inbound frames dropped because they exceed CanFrame's
  /// 8-byte Classical-CAN payload limit (length > 8 or FDF flag set). When
  /// CanFrame is generalized to CAN FD this will stay at 0.
  uint64_t fd_frames_dropped() const noexcept;

private:
  void onUdpFrame(const polymath::axiomatic::CanFdFrameRecord & frame);
  void onUdpErrorFrame(uint8_t error_code);

  Options opts_;
  std::unique_ptr<polymath::axiomatic::UdpClient> udp_client_;
  std::atomic<bool> rx_dispatch_active_{false};
  std::atomic<uint64_t> fd_frames_dropped_{0};

  std::mutex callback_mu_;
  std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)>
    on_receive_;
  std::function<void(socket_error_string_t error)> on_error_;
};

}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_FD_BACKEND_HPP_
