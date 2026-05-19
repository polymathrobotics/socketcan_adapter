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

#include "axiomatic_adapter/axiomatic_fd_backend.hpp"

#include <linux/can.h>

#include <array>
#include <cstring>

namespace polymath
{
namespace can
{

namespace ax = polymath::axiomatic;

// ---------------------------------------------------------------------------
// Helpers: convert between socketcan::CanFrame and axiomatic::CanFdFrameRecord
// ---------------------------------------------------------------------------
//
// Classical-CAN-only. CanFrame currently wraps `struct can_frame` (8-byte data
// limit). When CanFrame is generalized to support `canfd_frame`, this is the
// translation layer that grows to handle 64-byte payloads + BRS / ESI flags.

namespace
{

/// Pack a CanFrame into an outbound CanFdFrameRecord. Returns nullopt if the
/// source frame doesn't fit our current Classical-CAN-only constraint.
std::optional<ax::CanFdFrameRecord> toFdRecord(
  const polymath::socketcan::CanFrame & src,
  ax::ChannelAddress tx_addr)
{
  const auto len = src.get_len();
  if (len > 8u) {
    return std::nullopt;  // shouldn't happen for CanFrame, but defensive
  }
  ax::CanFdFrameRecord out{};
  out.address = tx_addr;
  out.length = len;
  out.can_id = src.get_id();
  // Map FrameType / IdType → CAN flag bits.
  uint8_t flags = 0;
  if (src.get_id_type() == polymath::socketcan::IdType::EXTENDED) {
    flags |= ax::can_flag::kExtId;
  }
  switch (src.get_frame_type()) {
    case polymath::socketcan::FrameType::REMOTE:
      flags |= ax::can_flag::kRemote;
      break;
    case polymath::socketcan::FrameType::ERROR:
      flags |= ax::can_flag::kError;
      break;
    case polymath::socketcan::FrameType::DATA:
    default:
      break;
  }
  out.flags = flags;
  const auto data = src.get_data();
  for (uint8_t i = 0; i < len; ++i) {
    out.data[i] = data[i];
  }
  return out;
}

/// Unpack an inbound CanFdFrameRecord into a CanFrame. Returns nullopt if the
/// record exceeds CanFrame's Classical-CAN limits (length > 8 or FDF flag set);
/// caller must count those drops as a known limitation pending CanFrame
/// generalization.
std::optional<polymath::socketcan::CanFrame> toCanFrame(
  const ax::CanFdFrameRecord & src)
{
  if (src.isFd() || src.length > CAN_MAX_DLC) {
    return std::nullopt;
  }
  struct can_frame raw {};
  raw.can_id = src.can_id;
  if (src.isExtId()) {
    raw.can_id |= CAN_EFF_FLAG;
  }
  if (src.isRemote()) {
    raw.can_id |= CAN_RTR_FLAG;
  }
  if (src.isError()) {
    raw.can_id |= CAN_ERR_FLAG;
  }
  raw.len = src.length;
  for (uint8_t i = 0; i < src.length; ++i) {
    raw.data[i] = src.data[i];
  }
  return polymath::socketcan::CanFrame(raw);
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

AxiomaticFdBackend::AxiomaticFdBackend(Options opts) : opts_(std::move(opts)) {}

AxiomaticFdBackend::~AxiomaticFdBackend()
{
  closeSocket();
}

bool AxiomaticFdBackend::openSocket()
{
  if (udp_client_) {
    return true;  // already open
  }
  ax::UdpClient::Options uopts;
  uopts.device_ip = opts_.device_ip;
  uopts.device_port = opts_.device_port;
  uopts.heartbeat_interval = opts_.heartbeat_interval;
  uopts.supported_features =
    ax::supported_features::kCanFdStream | ax::supported_features::kOneFramePerMessage;
  udp_client_ = std::make_unique<ax::UdpClient>(uopts);

  // Wire up our internal dispatchers BEFORE start() so we never miss frames.
  udp_client_->setOnFrame(
    [this](const ax::CanFdFrameRecord & f) { onUdpFrame(f); });
  udp_client_->setOnErrorFrame(
    [this](uint8_t code) { onUdpErrorFrame(code); });

  return udp_client_->start();
}

bool AxiomaticFdBackend::closeSocket()
{
  rx_dispatch_active_.store(false);
  if (udp_client_) {
    udp_client_->stop();
    udp_client_.reset();
  }
  return true;
}

polymath::socketcan::SocketState AxiomaticFdBackend::get_socket_state()
{
  if (!udp_client_) {
    return polymath::socketcan::SocketState::CLOSED;
  }
  return (udp_client_->localPort() != 0)
         ? polymath::socketcan::SocketState::OPEN
         : polymath::socketcan::SocketState::CLOSED;
}

// ---------------------------------------------------------------------------
// Send / receive
// ---------------------------------------------------------------------------

std::optional<AxiomaticFdBackend::socket_error_string_t> AxiomaticFdBackend::send(
  const polymath::socketcan::CanFrame & frame)
{
  if (!udp_client_) {
    return socket_error_string_t{"send: socket not open"};
  }
  const auto record = toFdRecord(frame, opts_.tx_address);
  if (!record) {
    return socket_error_string_t{"send: frame doesn't fit Classical-CAN constraint"};
  }
  if (!udp_client_->sendFrame(*record)) {
    return socket_error_string_t{"send: UdpClient::sendFrame failed"};
  }
  return std::nullopt;
}

std::optional<AxiomaticFdBackend::socket_error_string_t> AxiomaticFdBackend::send(
  const std::shared_ptr<const polymath::socketcan::CanFrame> frame)
{
  if (!frame) {
    return socket_error_string_t{"send: null frame"};
  }
  return send(*frame);
}

std::optional<AxiomaticFdBackend::socket_error_string_t> AxiomaticFdBackend::receive(
  polymath::socketcan::CanFrame & /*frame*/)
{
  // Synchronous receive isn't a natural fit for the device's asynchronous UDP
  // stream — the UdpClient delivers frames via the rx-thread callback. Most
  // ICanBackend consumers (including the ROS2 bridge node) only use the
  // callback path; the SocketcanAdapter's blocking `receive()` is a legacy
  // affordance. Leaving this unimplemented until a real caller surfaces.
  return socket_error_string_t{"receive(CanFrame&) not yet implemented; "
                               "use setOnReceiveCallback + startReceptionThread"};
}

// ---------------------------------------------------------------------------
// Reception thread + callbacks
// ---------------------------------------------------------------------------

bool AxiomaticFdBackend::startReceptionThread()
{
  if (!udp_client_) {
    return false;
  }
  rx_dispatch_active_.store(true);
  return true;
}

bool AxiomaticFdBackend::joinReceptionThread(
  const std::chrono::duration<float> & /*timeout_s*/)
{
  rx_dispatch_active_.store(false);
  // The UdpClient's rx thread keeps running until closeSocket() — we only
  // stop dispatching to our caller's callback. That matches SocketcanAdapter's
  // join semantics: the OS socket stays open, just no more callbacks.
  return true;
}

bool AxiomaticFdBackend::is_thread_running()
{
  return rx_dispatch_active_.load();
}

bool AxiomaticFdBackend::setOnReceiveCallback(
  std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame> frame)> && cb)
{
  std::lock_guard<std::mutex> lk(callback_mu_);
  on_receive_ = std::move(cb);
  return true;
}

bool AxiomaticFdBackend::setOnErrorCallback(
  std::function<void(socket_error_string_t error)> && cb)
{
  std::lock_guard<std::mutex> lk(callback_mu_);
  on_error_ = std::move(cb);
  return true;
}

uint64_t AxiomaticFdBackend::fd_frames_dropped() const noexcept
{
  return fd_frames_dropped_.load();
}

// ---------------------------------------------------------------------------
// Internal: UdpClient → caller bridging
// ---------------------------------------------------------------------------

void AxiomaticFdBackend::onUdpFrame(const ax::CanFdFrameRecord & frame)
{
  if (!rx_dispatch_active_.load(std::memory_order_relaxed)) {
    return;
  }
  auto cf = toCanFrame(frame);
  if (!cf) {
    fd_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  std::function<void(std::unique_ptr<const polymath::socketcan::CanFrame>)> cb;
  {
    std::lock_guard<std::mutex> lk(callback_mu_);
    cb = on_receive_;
  }
  if (cb) {
    cb(std::make_unique<polymath::socketcan::CanFrame>(std::move(*cf)));
  }
}

void AxiomaticFdBackend::onUdpErrorFrame(uint8_t error_code)
{
  if (!rx_dispatch_active_.load(std::memory_order_relaxed)) {
    return;
  }
  std::function<void(socket_error_string_t)> cb;
  {
    std::lock_guard<std::mutex> lk(callback_mu_);
    cb = on_error_;
  }
  if (cb) {
    const char * name = nullptr;
    switch (error_code) {
      case 0: name = "CAN error undefined"; break;
      case 1: name = "CAN warning"; break;
      case 2: name = "CAN passive"; break;
      case 3: name = "CAN bus-off"; break;
      default: name = "CAN error"; break;
    }
    cb(socket_error_string_t{name});
  }
}

}  // namespace can
}  // namespace polymath
