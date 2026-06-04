// Lifecycle tests for UdpClient — no hardware required.
//
// We start against a private TEST-NET-1 address (192.0.2.1, RFC 5737) that
// is guaranteed never to respond. UDP is connectionless so sendto() just
// queues bytes that go nowhere; the rx thread blocks on poll() until stop().
// This is sufficient to catch races/leaks in start/stop, double-start,
// destructor cleanup, and sendFrame() encode-validation paths.

#include "axiomatic_adapter/udp_client.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace ax = polymath::axiomatic;

namespace {
constexpr const char * kBlackHoleIp = "192.0.2.1";  // RFC 5737 TEST-NET-1
}

TEST(UdpClient, ConstructWithoutStartIsSafe) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  // Destructor runs without start().
}

TEST(UdpClient, StartStopRoundTrip) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  opts.heartbeat_interval = std::chrono::milliseconds(50);
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  EXPECT_NE(client.localPort(), 0u);
  // Let the heartbeat thread fire at least once.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  client.stop();
  EXPECT_GE(client.stats().heartbeats_sent.load(), 1u);
  EXPECT_EQ(client.localPort(), 0u);
}

TEST(UdpClient, DoubleStartIsIdempotent) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  EXPECT_TRUE(client.start());  // already running
  client.stop();
}

TEST(UdpClient, DoubleStopIsSafe) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  client.stop();
  client.stop();  // no-op
}

TEST(UdpClient, RestartAfterStop) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  const uint16_t port_a = client.localPort();
  client.stop();
  ASSERT_TRUE(client.start());
  const uint16_t port_b = client.localPort();
  EXPECT_NE(port_b, 0u);
  // ports may or may not be reused; we only assert non-zero on the second go.
  (void)port_a;
  client.stop();
}

TEST(UdpClient, RejectsBadIp) {
  ax::UdpClient::Options opts;
  opts.device_ip = "not-an-ip";
  ax::UdpClient client(opts);
  EXPECT_FALSE(client.start());
}

TEST(UdpClient, SendFrameWithoutStartFails) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ax::CanFdFrameRecord f{};
  f.address = {0u, 0x01u};
  f.flags = 0;
  f.length = 0;
  f.can_id = 0x123;
  EXPECT_FALSE(client.sendFrame(f));
  EXPECT_EQ(client.stats().tx_errors.load(), 1u);
}

TEST(UdpClient, SendFrameValidatesEncode) {
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  // Out-of-range standard ID — encoder rejects.
  ax::CanFdFrameRecord bad{};
  bad.address = {0u, 0x01u};
  bad.flags = 0;
  bad.length = 0;
  bad.can_id = 0x800;  // > 0x7FF and EID flag not set
  EXPECT_FALSE(client.sendFrame(bad));
  EXPECT_GE(client.stats().tx_errors.load(), 1u);
  // Good frame succeeds (sendto to black hole still returns OK; bytes go to /dev/null).
  ax::CanFdFrameRecord good{};
  good.address = {0u, 0x01u};
  good.flags = 0;
  good.length = 0;
  good.can_id = 0x123;
  EXPECT_TRUE(client.sendFrame(good));
  EXPECT_EQ(client.stats().frames_sent.load(), 1u);
  client.stop();
}

TEST(UdpClient, CallbacksNotSetIsSafe) {
  // No on-frame, on-error, on-heartbeat callbacks set — rx thread must not crash
  // even though we won't receive anything from the black hole anyway.
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  client.stop();
}

TEST(UdpClient, HeartbeatPacingApprox1Hz) {
  // Run with a tighter interval to keep the test fast, verify the count.
  ax::UdpClient::Options opts;
  opts.device_ip = kBlackHoleIp;
  opts.heartbeat_interval = std::chrono::milliseconds(50);
  opts.send_initial_status_request = false;
  ax::UdpClient client(opts);
  ASSERT_TRUE(client.start());
  std::this_thread::sleep_for(std::chrono::milliseconds(525));
  client.stop();
  const auto sent = client.stats().heartbeats_sent.load();
  // 525 ms / 50 ms = ~10.5 heartbeats. Allow [9, 12] for jitter.
  EXPECT_GE(sent, 9u);
  EXPECT_LE(sent, 12u);
}
