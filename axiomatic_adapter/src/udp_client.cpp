#include "axiomatic_adapter/udp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>

namespace polymath::axiomatic {

UdpClient::UdpClient(Options opts) : opts_(std::move(opts)) {}

UdpClient::~UdpClient() { stop(); }

bool UdpClient::start() {
  if (running_.load()) {
    return true;
  }

  sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0) {
    return false;
  }

  // Bind to ephemeral local port; kernel picks one.
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  local.sin_port = 0;
  if (::bind(sock_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
    ::close(sock_);
    sock_ = -1;
    return false;
  }
  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  ::getsockname(sock_, reinterpret_cast<sockaddr *>(&bound), &bound_len);
  local_port_ = ntohs(bound.sin_port);

  // Resolve destination.
  std::memset(&dst_, 0, sizeof(dst_));
  dst_.sin_family = AF_INET;
  dst_.sin_port = htons(opts_.device_port);
  if (::inet_pton(AF_INET, opts_.device_ip.c_str(), &dst_.sin_addr) != 1) {
    ::close(sock_);
    sock_ = -1;
    return false;
  }

  // Optional one-shot Status Request to register the connection device-side.
  if (opts_.send_initial_status_request) {
    std::array<uint8_t, kEnvelopeHeaderSize> sr{};
    writeMessageHeader(sr.data(), MessageId::StatusRequest, 0, 0);
    sendBytes(sr.data(), sr.size());
  }

  running_.store(true);
  tx_thread_ = std::thread(&UdpClient::txLoop, this);
  rx_thread_ = std::thread(&UdpClient::rxLoop, this);
  return true;
}

void UdpClient::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  // Shut the socket down to break any blocked recvfrom() in rx_thread.
  if (sock_ >= 0) {
    ::shutdown(sock_, SHUT_RDWR);
  }
  if (tx_thread_.joinable()) tx_thread_.join();
  if (rx_thread_.joinable()) rx_thread_.join();
  if (sock_ >= 0) {
    ::close(sock_);
    sock_ = -1;
  }
  local_port_ = 0;
}

bool UdpClient::sendFrame(const CanFdFrameRecord & frame) {
  const auto wire = encodeCanFdStreamSingleFrame(frame);
  if (wire.empty()) {
    stats_.tx_errors.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!sendBytes(wire.data(), wire.size())) {
    return false;
  }
  stats_.frames_sent.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void UdpClient::setOnFrame(FrameCallback cb)               { on_frame_ = std::move(cb); }
void UdpClient::setOnErrorFrame(ErrorFrameCallback cb)     { on_error_frame_ = std::move(cb); }
void UdpClient::setOnHeartbeat(HeartbeatCallback cb)       { on_heartbeat_ = std::move(cb); }

uint16_t UdpClient::localPort() const { return local_port_; }

bool UdpClient::sendBytes(const uint8_t * data, std::size_t len) {
  if (sock_ < 0) {
    stats_.tx_errors.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  std::lock_guard<std::mutex> lk(tx_mutex_);
  ssize_t n = ::sendto(sock_, data, len, 0,
                       reinterpret_cast<const sockaddr *>(&dst_), sizeof(dst_));
  if (n != static_cast<ssize_t>(len)) {
    stats_.tx_errors.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  return true;
}

void UdpClient::txLoop() {
  auto next = std::chrono::steady_clock::now();
  while (running_.load(std::memory_order_relaxed)) {
    HeartbeatV2 hb{};
    hb.message_number = hb_msg_num_.fetch_add(1, std::memory_order_relaxed);
    hb.time_interval_ms =
        static_cast<uint32_t>(opts_.heartbeat_interval.count());
    hb.supported_features = opts_.supported_features;
    const auto wire = encodeHeartbeatV2(hb);
    if (sendBytes(wire.data(), wire.size())) {
      stats_.heartbeats_sent.fetch_add(1, std::memory_order_relaxed);
    }

    // Pace at exactly heartbeat_interval using absolute deadlines.
    next += opts_.heartbeat_interval;
    while (running_.load(std::memory_order_relaxed)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next) break;
      const auto chunk = std::min<std::chrono::nanoseconds>(
          next - now, std::chrono::milliseconds(100));
      std::this_thread::sleep_for(chunk);
    }
  }
}

void UdpClient::rxLoop() {
  uint8_t buf[2048];
  while (running_.load(std::memory_order_relaxed)) {
    pollfd pfd{sock_, POLLIN, 0};
    int rc = ::poll(&pfd, 1, /*timeout_ms=*/100);
    if (rc < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (rc == 0) continue;
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      // Socket shut down (likely by stop()).
      break;
    }
    if ((pfd.revents & POLLIN) == 0) continue;

    sockaddr_in src{};
    socklen_t src_len = sizeof(src);
    ssize_t n = ::recvfrom(sock_, buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr *>(&src), &src_len);
    if (n <= 0) {
      if (n < 0 && errno == EINTR) continue;
      // n == 0 or other error: treat as shutdown.
      break;
    }

    auto hdr = parseMessageHeader(buf, static_cast<std::size_t>(n));
    if (!hdr) {
      stats_.parse_errors.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    switch (static_cast<MessageId>(hdr->message_id)) {
      case MessageId::Heartbeat: {
        auto hb = parseHeartbeat(buf, static_cast<std::size_t>(n));
        if (hb) {
          stats_.heartbeats_received.fetch_add(1, std::memory_order_relaxed);
          if (on_heartbeat_) on_heartbeat_(*hb);
        } else {
          stats_.parse_errors.fetch_add(1, std::memory_order_relaxed);
        }
        break;
      }
      case MessageId::CanFdStream: {
        auto frame = parseCanFdStreamSingleFrame(buf, static_cast<std::size_t>(n));
        if (frame) {
          if (frame->isError()) {
            stats_.error_frames_received.fetch_add(1, std::memory_order_relaxed);
            if (on_error_frame_ && frame->length >= 1) {
              on_error_frame_(frame->data[0]);
            }
          } else {
            stats_.frames_received.fetch_add(1, std::memory_order_relaxed);
            if (on_frame_) on_frame_(*frame);
          }
        } else {
          stats_.parse_errors.fetch_add(1, std::memory_order_relaxed);
        }
        break;
      }
      case MessageId::StatusRequest:
      case MessageId::StatusResponse:
        // Tracked but no callback yet — health/status decoding is its own concern.
        break;
      default:
        stats_.parse_errors.fetch_add(1, std::memory_order_relaxed);
        break;
    }
  }
}

}  // namespace polymath::axiomatic
