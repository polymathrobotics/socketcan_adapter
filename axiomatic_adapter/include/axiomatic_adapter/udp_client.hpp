// UdpClient — host-side UDP transport to an Axiomatic AX140970.
//
// Owns one connected UDP socket plus two threads:
//   tx_thread: emits a 1 Hz HeartbeatV2 to keep the device-side connection alive
//              (10 s idle timeout per spec §2.3.1.1).
//   rx_thread: blocks on poll(), recvfrom()s every datagram, parses the envelope,
//              dispatches by Message ID to user-supplied callbacks.
//
// Callbacks run on the rx thread. Keep them fast — anything blocking will back
// up the device's internal queue and eventually drop frames.
//
// All public methods are thread-safe except setOn*() which must be called
// before start().
//
// This is *not* yet the ICanBackend implementation from DESIGN.md — it's the
// transport layer it will sit on. Minimal by intent.

#ifndef AXIOMATIC_ADAPTER__UDP_CLIENT_HPP_
#define AXIOMATIC_ADAPTER__UDP_CLIENT_HPP_

#include "axiomatic_adapter/axiomatic_protocol.hpp"

#include <netinet/in.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace polymath::axiomatic {

class UdpClient {
 public:
  struct Options {
    std::string device_ip;
    uint16_t    device_port = 4000;
    // Bitmask of supported_features::* advertised in our outbound heartbeat.
    uint32_t    supported_features =
        supported_features::kCanFdStream | supported_features::kOneFramePerMessage;
    std::chrono::milliseconds heartbeat_interval{1000};
    // If true, send a one-shot Status Request immediately after socket bind.
    // The bench probe confirmed this is the cleanest registration handshake.
    bool        send_initial_status_request = true;
  };

  struct Stats {
    std::atomic<uint64_t> heartbeats_sent{0};
    std::atomic<uint64_t> heartbeats_received{0};
    std::atomic<uint64_t> frames_sent{0};
    std::atomic<uint64_t> frames_received{0};
    std::atomic<uint64_t> error_frames_received{0};
    std::atomic<uint64_t> parse_errors{0};
    std::atomic<uint64_t> tx_errors{0};
  };

  // Callbacks. All invoked from the rx thread. Default: do nothing.
  using FrameCallback     = std::function<void(const CanFdFrameRecord &)>;
  using ErrorFrameCallback = std::function<void(uint8_t code)>;       // Warning/Passive/Bus-Off
  using HeartbeatCallback = std::function<void(const HeartbeatV2 &)>;

  explicit UdpClient(Options opts);
  ~UdpClient();

  UdpClient(const UdpClient &)              = delete;
  UdpClient & operator=(const UdpClient &)  = delete;
  UdpClient(UdpClient &&)                   = delete;
  UdpClient & operator=(UdpClient &&)       = delete;

  // Open the socket, register with the device, start tx + rx threads.
  // Returns true on success, false on socket / bind / connect failure.
  bool start();

  // Signal threads to exit, join, close the socket. Safe to call repeatedly.
  void stop();

  // Send a CAN FD Stream message containing exactly one CAN FD Frame to the device.
  // Thread-safe with respect to the heartbeat thread (mutex around sendto).
  // Returns false on encode failure (bad frame) or socket error.
  bool sendFrame(const CanFdFrameRecord & frame);

  // Setters — call before start().
  void setOnFrame(FrameCallback cb);
  void setOnErrorFrame(ErrorFrameCallback cb);
  void setOnHeartbeat(HeartbeatCallback cb);

  // Local socket port the kernel assigned us (after start()), or 0 if not running.
  uint16_t localPort() const;

  // Read-only stats.
  const Stats & stats() const noexcept { return stats_; }

 private:
  void txLoop();
  void rxLoop();
  bool sendBytes(const uint8_t * data, std::size_t len);

  Options                       opts_;
  int                           sock_ = -1;
  sockaddr_in                   dst_{};
  uint16_t                      local_port_ = 0;
  std::atomic<bool>             running_{false};
  std::atomic<uint32_t>         hb_msg_num_{0};
  std::thread                   tx_thread_;
  std::thread                   rx_thread_;
  std::mutex                    tx_mutex_;     // serializes sendto across hb + frames
  Stats                         stats_;

  FrameCallback                 on_frame_;
  ErrorFrameCallback            on_error_frame_;
  HeartbeatCallback             on_heartbeat_;
};

}  // namespace polymath::axiomatic

#endif  // AXIOMATIC_ADAPTER__UDP_CLIENT_HPP_
