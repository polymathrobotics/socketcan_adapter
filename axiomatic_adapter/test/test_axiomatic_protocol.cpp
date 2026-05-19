#include "axiomatic_adapter/axiomatic_protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace ax = polymath::axiomatic;

namespace {

ax::CanFdFrameRecord makeClassicalDataFrame(uint32_t id,
                                            std::initializer_list<uint8_t> payload,
                                            bool extended = false) {
  ax::CanFdFrameRecord f{};
  f.address = {0u, 0x00000001u};   // CG=0, CID=1 (CAN1 default)
  f.timestamp_ms = 12345u;
  f.flags = extended ? ax::can_flag::kExtId : 0u;
  f.length = static_cast<uint8_t>(payload.size());
  f.can_id = id;
  std::size_t i = 0;
  for (uint8_t b : payload) {
    f.data[i++] = b;
  }
  return f;
}

ax::CanFdFrameRecord makeFdFrame(uint32_t id, uint8_t length,
                                 bool extended = true,
                                 bool brs = true,
                                 bool esi = false) {
  ax::CanFdFrameRecord f{};
  f.address = {0u, 0x00000001u};
  f.timestamp_ms = 0xCAFEBABEu;
  uint8_t flags = ax::can_flag::kFd;
  if (extended) flags |= ax::can_flag::kExtId;
  if (brs) flags |= ax::can_flag::kBrs;
  if (esi) flags |= ax::can_flag::kEsi;
  f.flags = flags;
  f.length = length;
  f.can_id = id;
  for (uint8_t i = 0; i < length; ++i) {
    f.data[i] = static_cast<uint8_t>(i ^ 0xA5);
  }
  return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Endianness helpers
// ---------------------------------------------------------------------------

TEST(Endianness, ReadWriteLE16) {
  uint8_t buf[2] = {};
  ax::writeLE16(buf, 0x1234);
  EXPECT_EQ(buf[0], 0x34);
  EXPECT_EQ(buf[1], 0x12);
  EXPECT_EQ(ax::readLE16(buf), 0x1234);
}

TEST(Endianness, ReadWriteLE32) {
  uint8_t buf[4] = {};
  ax::writeLE32(buf, 0xDEADBEEF);
  EXPECT_EQ(buf[0], 0xEF);
  EXPECT_EQ(buf[1], 0xBE);
  EXPECT_EQ(buf[2], 0xAD);
  EXPECT_EQ(buf[3], 0xDE);
  EXPECT_EQ(ax::readLE32(buf), 0xDEADBEEFu);
}

TEST(Endianness, ProtocolIdOnWireIsBA36) {
  // Per spec: Protocol ID 14010 = 0x36BA, presented LSB-first as bytes (0xBA, 0x36).
  uint8_t buf[2] = {};
  ax::writeLE16(buf, ax::kProtocolId);
  EXPECT_EQ(buf[0], 0xBAu);
  EXPECT_EQ(buf[1], 0x36u);
}

// ---------------------------------------------------------------------------
// CAN FD length validation
// ---------------------------------------------------------------------------

TEST(Length, AllValidFdLengths) {
  for (uint8_t l = 0; l <= 8; ++l) {
    EXPECT_TRUE(ax::isValidCanFdLength(l)) << "len=" << +l;
  }
  for (uint8_t l : std::array<uint8_t, 7>{12, 16, 20, 24, 32, 48, 64}) {
    EXPECT_TRUE(ax::isValidCanFdLength(l)) << "len=" << +l;
  }
}

TEST(Length, RejectsInvalidFdLengths) {
  for (uint8_t l : std::array<uint8_t, 12>{9, 10, 11, 13, 17, 25, 33, 49, 63, 65, 100, 255}) {
    EXPECT_FALSE(ax::isValidCanFdLength(l)) << "len=" << +l;
  }
}

// ---------------------------------------------------------------------------
// Envelope codec
// ---------------------------------------------------------------------------

TEST(Envelope, WriteThenParseRoundTrip) {
  std::array<uint8_t, ax::kEnvelopeHeaderSize> buf{};
  const std::size_t n = ax::writeMessageHeader(buf.data(),
                                               ax::MessageId::Heartbeat,
                                               /*version=*/2,
                                               /*data_length=*/42);
  ASSERT_EQ(n, ax::kEnvelopeHeaderSize);

  // Wire-level byte checks.
  EXPECT_EQ(buf[0], 'A');
  EXPECT_EQ(buf[1], 'X');
  EXPECT_EQ(buf[2], 'I');
  EXPECT_EQ(buf[3], 'O');
  EXPECT_EQ(buf[4], 0xBAu);  // protocol id LSB
  EXPECT_EQ(buf[5], 0x36u);  // protocol id MSB
  EXPECT_EQ(buf[6], 0x04u);  // message id LSB (Heartbeat = 4)
  EXPECT_EQ(buf[7], 0x00u);
  EXPECT_EQ(buf[8], 2u);     // version
  EXPECT_EQ(buf[9], 42u);    // data length LSB
  EXPECT_EQ(buf[10], 0u);    // data length MSB

  const auto h = ax::parseMessageHeader(buf.data(), buf.size());
  ASSERT_TRUE(h.has_value());
  EXPECT_EQ(h->protocol_id, ax::kProtocolId);
  EXPECT_EQ(h->message_id, static_cast<uint16_t>(ax::MessageId::Heartbeat));
  EXPECT_EQ(h->message_version, 2u);
  EXPECT_EQ(h->data_length, 42u);
}

TEST(Envelope, RejectsTruncatedBuffer) {
  std::array<uint8_t, ax::kEnvelopeHeaderSize> buf{};
  ax::writeMessageHeader(buf.data(), ax::MessageId::CanFdStream, 0, 0);
  for (std::size_t n = 0; n < ax::kEnvelopeHeaderSize; ++n) {
    EXPECT_FALSE(ax::parseMessageHeader(buf.data(), n).has_value()) << "n=" << n;
  }
}

TEST(Envelope, RejectsBadAxioTag) {
  std::array<uint8_t, ax::kEnvelopeHeaderSize> buf{};
  ax::writeMessageHeader(buf.data(), ax::MessageId::CanFdStream, 0, 0);
  buf[0] = 'B';  // corrupt
  EXPECT_FALSE(ax::parseMessageHeader(buf.data(), buf.size()).has_value());
}

TEST(Envelope, RejectsWrongProtocolId) {
  std::array<uint8_t, ax::kEnvelopeHeaderSize> buf{};
  ax::writeMessageHeader(buf.data(), ax::MessageId::CanFdStream, 0, 0);
  buf[4] = 0x00;  // protocol id LSB → 0x3600, not 0x36BA
  EXPECT_FALSE(ax::parseMessageHeader(buf.data(), buf.size()).has_value());
}

TEST(Envelope, RejectsOversizedDataLength) {
  std::array<uint8_t, ax::kEnvelopeHeaderSize> buf{};
  ax::writeMessageHeader(buf.data(), ax::MessageId::CanFdStream, 0, 246);
  EXPECT_FALSE(ax::parseMessageHeader(buf.data(), buf.size()).has_value());
}

TEST(Envelope, AcceptsNullData) {
  EXPECT_FALSE(ax::parseMessageHeader(nullptr, 11).has_value());
}

// ---------------------------------------------------------------------------
// CAN FD Frame: standard CAN ID
// ---------------------------------------------------------------------------

TEST(CanFdFrame, ClassicalStdIdRoundTrip_AllDlcs) {
  for (uint8_t dlc = 0; dlc <= 8; ++dlc) {
    ax::CanFdFrameRecord src{};
    src.address = {7, 0x00000005u};
    src.timestamp_ms = 1000u + dlc;
    src.flags = 0u;     // classical, std id
    src.length = dlc;
    src.can_id = 0x123u;
    for (uint8_t i = 0; i < dlc; ++i) {
      src.data[i] = static_cast<uint8_t>(0x10 + i);
    }
    std::vector<uint8_t> wire;
    const std::size_t n = ax::writeCanFdFrame(wire, src);
    ASSERT_EQ(n, ax::kCanFdFrameHeaderSize + dlc) << "dlc=" << +dlc;

    const auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value()) << "dlc=" << +dlc;
    EXPECT_EQ(got->address.channel_group, src.address.channel_group);
    EXPECT_EQ(got->address.channel_id_set, src.address.channel_id_set);
    EXPECT_EQ(got->timestamp_ms, src.timestamp_ms);
    EXPECT_EQ(got->flags, src.flags);
    EXPECT_EQ(got->length, src.length);
    EXPECT_EQ(got->can_id, src.can_id);
    for (uint8_t i = 0; i < dlc; ++i) {
      EXPECT_EQ(got->data[i], src.data[i]);
    }
  }
}

TEST(CanFdFrame, StdIdBoundary) {
  for (uint32_t id : {0u, 1u, ax::kStdIdMax}) {
    auto f = makeClassicalDataFrame(id, {0xAA, 0xBB}, /*extended=*/false);
    std::vector<uint8_t> wire;
    ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u);
    auto parsed = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->can_id, id);
    EXPECT_FALSE(parsed->isExtId());
  }
}

TEST(CanFdFrame, RejectsStdIdOutOfRange) {
  auto f = makeClassicalDataFrame(0x800u, {}, /*extended=*/false);
  std::vector<uint8_t> wire;
  EXPECT_EQ(ax::writeCanFdFrame(wire, f), 0u);
}

// ---------------------------------------------------------------------------
// CAN FD Frame: extended CAN ID
// ---------------------------------------------------------------------------

TEST(CanFdFrame, ExtIdBoundary) {
  for (uint32_t id : {0u, 0x800u, ax::kExtIdMax}) {
    auto f = makeClassicalDataFrame(id, {1, 2, 3, 4, 5, 6, 7, 8}, /*extended=*/true);
    std::vector<uint8_t> wire;
    ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u);
    auto parsed = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(parsed->can_id, id);
    EXPECT_TRUE(parsed->isExtId());
  }
}

TEST(CanFdFrame, RejectsExtIdOutOfRange) {
  auto f = makeClassicalDataFrame(0x20000000u, {}, /*extended=*/true);
  std::vector<uint8_t> wire;
  EXPECT_EQ(ax::writeCanFdFrame(wire, f), 0u);
}

TEST(CanFdFrame, ParseRejectsReservedBitsSetInStdId) {
  // Build a valid-ish wire frame, then poke a reserved bit in CANID.
  auto f = makeClassicalDataFrame(0x123u, {0x55}, /*extended=*/false);
  std::vector<uint8_t> wire;
  ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u);
  // CANID lives at bytes [13..16]; bits [11..31] must be 0 for std ID.
  wire[14] |= 0x80u;  // poke bit 15 (0x8000 in the 16-bit-equivalent)
  EXPECT_FALSE(ax::parseCanFdFrame(wire.data(), wire.size()).has_value());
}

// ---------------------------------------------------------------------------
// CAN FD: all FD DLCs and flag combinations
// ---------------------------------------------------------------------------

TEST(CanFdFrame, FdDataAllValidDlcs) {
  for (uint8_t dlc : {uint8_t{0}, uint8_t{1}, uint8_t{8}, uint8_t{12},
                      uint8_t{16}, uint8_t{20}, uint8_t{24}, uint8_t{32},
                      uint8_t{48}, uint8_t{64}}) {
    auto src = makeFdFrame(0x1ABCDEFu, dlc);
    std::vector<uint8_t> wire;
    ASSERT_EQ(ax::writeCanFdFrame(wire, src), ax::kCanFdFrameHeaderSize + dlc);

    auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value()) << "dlc=" << +dlc;
    EXPECT_EQ(got->length, dlc);
    EXPECT_TRUE(got->isFd());
    EXPECT_TRUE(got->isExtId());
    EXPECT_TRUE(got->isBrs());
    EXPECT_FALSE(got->isEsi());
    for (uint8_t i = 0; i < dlc; ++i) {
      EXPECT_EQ(got->data[i], static_cast<uint8_t>(i ^ 0xA5));
    }
  }
}

TEST(CanFdFrame, RejectsInvalidFdDlcOnWrite) {
  auto src = makeFdFrame(0x100u, /*length=*/9);
  std::vector<uint8_t> wire;
  EXPECT_EQ(ax::writeCanFdFrame(wire, src), 0u);
}

TEST(CanFdFrame, ParseRejectsInvalidDlcInFlags) {
  // Write a valid frame, corrupt the length field on the wire.
  auto src = makeFdFrame(0x100u, /*length=*/8);
  std::vector<uint8_t> wire;
  ASSERT_GT(ax::writeCanFdFrame(wire, src), 0u);
  wire[12] = 9;  // CANLB byte; 9 is invalid for FD
  EXPECT_FALSE(ax::parseCanFdFrame(wire.data(), wire.size()).has_value());
}

TEST(CanFdFrame, EveryFlagBitIndependent) {
  struct Case { uint8_t flag; const char * name; };
  // Flags that are testable in isolation under "data frame" semantics.
  // ERR is exercised separately (different length semantics).
  // RTR is mutually exclusive with FD per CAN spec, so we pair it with classical.
  const Case classical_cases[] = {
      {ax::can_flag::kExtId, "EID"},
      {ax::can_flag::kRemote, "RTR"},
  };
  for (auto c : classical_cases) {
    ax::CanFdFrameRecord f{};
    f.address = {0, 1u};
    f.flags = c.flag;
    f.length = 0;  // empty payload ok for classical and rtr
    f.can_id = (c.flag == ax::can_flag::kExtId) ? 0x12345u : 0x123u;
    std::vector<uint8_t> wire;
    ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u) << c.name;
    auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value()) << c.name;
    EXPECT_EQ(got->flags, c.flag) << c.name;
  }

  // FD-specific flags layered on top of FDF.
  const uint8_t fd_only_flags[] = {ax::can_flag::kFd,
                                   ax::can_flag::kFd | ax::can_flag::kBrs,
                                   ax::can_flag::kFd | ax::can_flag::kEsi,
                                   ax::can_flag::kFd | ax::can_flag::kBrs |
                                       ax::can_flag::kEsi};
  for (uint8_t fl : fd_only_flags) {
    ax::CanFdFrameRecord f{};
    f.address = {0, 1u};
    f.flags = fl;
    f.length = 12;
    f.can_id = 0x456u;
    std::vector<uint8_t> wire;
    ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u) << "flags=" << +fl;
    auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value()) << "flags=" << +fl;
    EXPECT_EQ(got->flags, fl);
  }
}

TEST(CanFdFrame, RemoteFrameCarriesNoPayloadBytes) {
  // RTR with length=8 means "remote request for 8 data bytes" — wire payload still empty.
  ax::CanFdFrameRecord f{};
  f.address = {0, 1u};
  f.flags = ax::can_flag::kRemote;
  f.length = 8;
  f.can_id = 0x111u;
  std::vector<uint8_t> wire;
  const std::size_t n = ax::writeCanFdFrame(wire, f);
  EXPECT_EQ(n, ax::kCanFdFrameHeaderSize);  // no data bytes appended
  auto got = ax::parseCanFdFrame(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->length, 8u);
  EXPECT_TRUE(got->isRemote());
}

TEST(CanFdFrame, ErrorMessageWithLength1To64) {
  for (uint8_t len : {uint8_t{1}, uint8_t{32}, uint8_t{64}}) {
    ax::CanFdFrameRecord f{};
    f.address = {0, 1u};
    f.flags = ax::can_flag::kError;
    f.length = len;
    f.can_id = 0;
    f.data[0] = 0xFF;  // first error code byte
    std::vector<uint8_t> wire;
    ASSERT_EQ(ax::writeCanFdFrame(wire, f), ax::kCanFdFrameHeaderSize + len);
    auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->length, len);
    EXPECT_TRUE(got->isError());
  }
}

TEST(CanFdFrame, RejectsErrorMessageLengthZero) {
  ax::CanFdFrameRecord f{};
  f.address = {0, 1u};
  f.flags = ax::can_flag::kError;
  f.length = 0;
  std::vector<uint8_t> wire;
  EXPECT_EQ(ax::writeCanFdFrame(wire, f), 0u);
}

// ---------------------------------------------------------------------------
// CAN FD Frame: routing + physical channel
// ---------------------------------------------------------------------------

TEST(CanFdFrame, ChannelAddressIsRoundTripped) {
  ax::CanFdFrameRecord f{};
  f.address = {255, 0xFFFFFFFFu};
  f.flags = 0;
  f.length = 0;
  f.can_id = 0;
  std::vector<uint8_t> wire;
  ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u);
  auto got = ax::parseCanFdFrame(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->address.channel_group, 255u);
  EXPECT_EQ(got->address.channel_id_set, 0xFFFFFFFFu);
}

TEST(CanFdFrame, PhysicalChannelMaxBoundary) {
  ax::CanFdFrameRecord f{};
  f.physical_channel = 0x1FFFu;  // max 13-bit
  f.address = {0, 1u};
  f.flags = 0;
  f.length = 0;
  f.can_id = 0;
  std::vector<uint8_t> wire;
  ASSERT_GT(ax::writeCanFdFrame(wire, f), 0u);
  auto got = ax::parseCanFdFrame(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->physical_channel, 0x1FFFu);
}

TEST(CanFdFrame, RejectsPhysicalChannelOverflow) {
  ax::CanFdFrameRecord f{};
  f.physical_channel = 0x2000u;  // > 13 bits
  f.address = {0, 1u};
  f.length = 0;
  std::vector<uint8_t> wire;
  EXPECT_EQ(ax::writeCanFdFrame(wire, f), 0u);
}

TEST(Routing, AddressesMatchOnlyWhenGroupEqualAndAnyChannelOverlap) {
  EXPECT_TRUE(ax::addressesMatch({0, 0b0011u}, {0, 0b0010u}));
  EXPECT_TRUE(ax::addressesMatch({5, 0b0001u}, {5, 0b0001u}));
  EXPECT_FALSE(ax::addressesMatch({0, 0b0001u}, {1, 0b0001u})) << "different group";
  EXPECT_FALSE(ax::addressesMatch({0, 0b0001u}, {0, 0b0010u})) << "no overlap";
  EXPECT_FALSE(ax::addressesMatch({0, 0u}, {0, 0xFFFFFFFFu})) << "null address never matches";
}

// ---------------------------------------------------------------------------
// Truncated / malformed parse
// ---------------------------------------------------------------------------

TEST(CanFdFrame, RejectsHeaderShorterThan17) {
  std::vector<uint8_t> wire(16, 0xFF);
  EXPECT_FALSE(ax::parseCanFdFrame(wire.data(), wire.size()).has_value());
}

TEST(CanFdFrame, RejectsTruncatedPayload) {
  auto src = makeFdFrame(0x100u, /*length=*/16);
  std::vector<uint8_t> wire;
  ax::writeCanFdFrame(wire, src);
  for (std::size_t n = ax::kCanFdFrameHeaderSize;
       n < ax::kCanFdFrameHeaderSize + 16u; ++n) {
    EXPECT_FALSE(ax::parseCanFdFrame(wire.data(), n).has_value()) << "n=" << n;
  }
}

// ---------------------------------------------------------------------------
// Single-frame stream message (envelope + frame, end-to-end)
// ---------------------------------------------------------------------------

TEST(StreamSingleFrame, RoundTripFd64) {
  auto src = makeFdFrame(0x1FFFFFFFu, 64);
  const auto wire = ax::encodeCanFdStreamSingleFrame(src);
  ASSERT_FALSE(wire.empty());
  EXPECT_EQ(wire.size(), ax::kEnvelopeHeaderSize + ax::kCanFdFrameHeaderSize + 64u);
  EXPECT_EQ(wire[0], 'A');
  EXPECT_EQ(wire[6], 0x05u);  // CAN FD Stream message id LSB
  EXPECT_EQ(wire[7], 0x00u);

  auto got = ax::parseCanFdStreamSingleFrame(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->can_id, src.can_id);
  EXPECT_EQ(got->length, 64u);
  EXPECT_TRUE(got->isFd());
  for (uint8_t i = 0; i < 64; ++i) {
    EXPECT_EQ(got->data[i], static_cast<uint8_t>(i ^ 0xA5));
  }
}

TEST(StreamSingleFrame, RoundTripClassical) {
  auto src = makeClassicalDataFrame(0x7FFu, {0xDE, 0xAD, 0xBE, 0xEF});
  const auto wire = ax::encodeCanFdStreamSingleFrame(src);
  ASSERT_FALSE(wire.empty());

  auto got = ax::parseCanFdStreamSingleFrame(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_FALSE(got->isFd());
  EXPECT_EQ(got->length, 4u);
  EXPECT_EQ(got->data[0], 0xDEu);
  EXPECT_EQ(got->data[3], 0xEFu);
}

TEST(StreamSingleFrame, RejectsWrongMessageId) {
  auto src = makeFdFrame(0x100u, 8);
  auto wire = ax::encodeCanFdStreamSingleFrame(src);
  ASSERT_FALSE(wire.empty());
  // Flip message id from 5 (CanFdStream) to 4 (Heartbeat).
  wire[6] = 0x04;
  EXPECT_FALSE(ax::parseCanFdStreamSingleFrame(wire.data(), wire.size()).has_value());
}

TEST(StreamSingleFrame, RejectsLengthMismatch) {
  auto src = makeFdFrame(0x100u, 8);
  auto wire = ax::encodeCanFdStreamSingleFrame(src);
  ASSERT_FALSE(wire.empty());
  // Lie about envelope data length (claim 1 byte instead of 17 + 8).
  wire[9] = 1;
  wire[10] = 0;
  EXPECT_FALSE(ax::parseCanFdStreamSingleFrame(wire.data(), wire.size()).has_value());
}

// ---------------------------------------------------------------------------
// Fuzzy round-trip (deterministic)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Heartbeat
// ---------------------------------------------------------------------------

TEST(Heartbeat, EncodeRoundTrip) {
  ax::HeartbeatV2 src{};
  src.message_number = 0x12345678u;
  src.time_interval_ms = 1000u;
  src.health_data = 0x05559556u;  // matches what the device sent us
  src.converter_type = 0;
  src.supported_features =
      ax::supported_features::kCanFdStream | ax::supported_features::kOneFramePerMessage;
  src.filter_channel_group = 0;
  src.filter_channel_id_set = 0;

  const auto wire = ax::encodeHeartbeatV2(src);
  ASSERT_EQ(wire.size(), ax::kEnvelopeHeaderSize + ax::kHeartbeatV2DataSize);

  // Envelope sanity.
  EXPECT_EQ(wire[0], 'A');
  EXPECT_EQ(wire[6], 0x04u);  // Heartbeat msg id LSB
  EXPECT_EQ(wire[8], ax::kHeartbeatVersion);
  EXPECT_EQ(wire[9], 22u);    // data length LSB

  const auto got = ax::parseHeartbeat(wire.data(), wire.size());
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->message_number, src.message_number);
  EXPECT_EQ(got->time_interval_ms, src.time_interval_ms);
  EXPECT_EQ(got->health_data, src.health_data);
  EXPECT_EQ(got->converter_type, src.converter_type);
  EXPECT_EQ(got->supported_features, src.supported_features);
  EXPECT_EQ(got->filter_channel_group, src.filter_channel_group);
  EXPECT_EQ(got->filter_channel_id_set, src.filter_channel_id_set);
}

// Parity test: bytes captured from real device (S/N 0010525274, FW V1.00) by
// `axiomatic_probe` on 2026-05-06. Heartbeat #3 from the probe transcript:
//
//   ← 33 bytes from 10.74.28.133:4000
//   envelope: AXIO ba 36 04 00 02 16 00   (Heartbeat v2, data_len=22)
//   data:     f6 2a 01 00 e8 03 00 00 56 95 55 05
//             00 01 00 00 00 00 00 00 00 00
//
// This guards against any regression where our codec stops matching real
// device wire bytes.
TEST(Heartbeat, ParseCapturedDeviceFixture) {
  const std::array<uint8_t, 33> wire{
      // Envelope
      0x41, 0x58, 0x49, 0x4F,  // AXIO
      0xBA, 0x36,              // Protocol ID = 14010
      0x04, 0x00,              // Message ID = 4 (Heartbeat)
      0x02,                    // Message Version = 2
      0x16, 0x00,              // Data Length = 22
      // Data
      0xF6, 0x2A, 0x01, 0x00,  // Message Number = 0x00012AF6 (76534)
      0xE8, 0x03, 0x00, 0x00,  // Time Interval  = 0x000003E8 (1000 ms)
      0x56, 0x95, 0x55, 0x05,  // Health Data
      0x00,                    // Converter Type = 0 (Ethernet to CAN w/ Voltage Out)
      0x01, 0x00, 0x00, 0x00,  // Supported Features = 0x00000001 (CAN FD Stream)
      0x00,                    // Filter CGMB = 0
      0x00, 0x00, 0x00, 0x00,  // Filter CIDSMB = 0 (NULL filter, accept all)
  };
  const auto hb = ax::parseHeartbeat(wire.data(), wire.size());
  ASSERT_TRUE(hb.has_value());
  EXPECT_EQ(hb->message_number, 0x00012AF6u);
  EXPECT_EQ(hb->time_interval_ms, 1000u);
  EXPECT_EQ(hb->health_data, 0x05559556u);
  EXPECT_EQ(hb->converter_type, 0u);
  EXPECT_EQ(hb->supported_features, ax::supported_features::kCanFdStream);
  EXPECT_EQ(hb->filter_channel_group, 0u);
  EXPECT_EQ(hb->filter_channel_id_set, 0u);
}

TEST(Heartbeat, RejectsTruncated) {
  ax::HeartbeatV2 src{};
  src.supported_features = ax::supported_features::kCanFdStream;
  const auto wire = ax::encodeHeartbeatV2(src);
  for (std::size_t n = 0; n < wire.size(); ++n) {
    EXPECT_FALSE(ax::parseHeartbeat(wire.data(), n).has_value())
        << "n=" << n;
  }
}

TEST(Heartbeat, RejectsWrongMessageId) {
  ax::HeartbeatV2 src{};
  auto wire = ax::encodeHeartbeatV2(src);
  wire[6] = 0x05;  // change Heartbeat → CanFdStream id
  EXPECT_FALSE(ax::parseHeartbeat(wire.data(), wire.size()).has_value());
}

TEST(Heartbeat, RejectsVersionBelowTwo) {
  ax::HeartbeatV2 src{};
  auto wire = ax::encodeHeartbeatV2(src);
  wire[8] = 1;  // claim Message Version 1
  EXPECT_FALSE(ax::parseHeartbeat(wire.data(), wire.size()).has_value());
}

TEST(Heartbeat, AcceptsFutureVersion) {
  // Forward-compat: Message Version > 2 with a longer data field is accepted;
  // we read only the v2-defined bytes and ignore the rest.
  ax::HeartbeatV2 src{};
  src.message_number = 42;
  auto wire = ax::encodeHeartbeatV2(src);
  // Pretend it's version 3 with extra bytes appended.
  wire[8] = 3;
  // data_length needs to grow to include the extra padding.
  wire[9] = static_cast<uint8_t>((ax::kHeartbeatV2DataSize + 5u) & 0xFFu);
  wire[10] = 0;
  for (int i = 0; i < 5; ++i) wire.push_back(0xAB);
  const auto hb = ax::parseHeartbeat(wire.data(), wire.size());
  ASSERT_TRUE(hb.has_value());
  EXPECT_EQ(hb->message_number, 42u);  // v2 fields still decoded correctly
}

TEST(Heartbeat, DataFieldOnlyParse) {
  // `parseHeartbeatV2DataField` is for callers that have already sliced out
  // the data region (e.g., a streaming parser that handled the envelope upstream).
  ax::HeartbeatV2 src{};
  src.message_number = 1;
  src.time_interval_ms = 1000;
  src.supported_features = ax::supported_features::kCanFdStream;
  const auto wire = ax::encodeHeartbeatV2(src);
  const uint8_t * data = wire.data() + ax::kEnvelopeHeaderSize;
  const auto hb = ax::parseHeartbeatV2DataField(data, ax::kHeartbeatV2DataSize);
  ASSERT_TRUE(hb.has_value());
  EXPECT_EQ(hb->message_number, 1u);
  EXPECT_EQ(hb->supported_features, ax::supported_features::kCanFdStream);
}

// ---------------------------------------------------------------------------
// Fuzzy round-trip
// ---------------------------------------------------------------------------

TEST(Fuzzy, RandomFramesRoundTrip) {
  std::mt19937 rng{0xC0FFEE};
  const uint8_t valid_lens[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,
                                12, 16, 20, 24, 32, 48, 64};
  for (int iter = 0; iter < 500; ++iter) {
    ax::CanFdFrameRecord f{};
    f.physical_channel =
        static_cast<uint16_t>(std::uniform_int_distribution<uint32_t>(0u, 0x1FFFu)(rng));
    f.address.channel_group =
        static_cast<uint8_t>(std::uniform_int_distribution<uint32_t>(0u, 255u)(rng));
    f.address.channel_id_set = std::uniform_int_distribution<uint32_t>{}(rng);
    f.timestamp_ms = std::uniform_int_distribution<uint32_t>{}(rng);

    const bool fd = (rng() & 1u) == 1u;
    const bool ext = (rng() & 1u) == 1u;
    uint8_t flags = 0;
    if (fd) flags |= ax::can_flag::kFd;
    if (ext) flags |= ax::can_flag::kExtId;
    if (fd && (rng() & 1u)) flags |= ax::can_flag::kBrs;
    if (fd && (rng() & 1u)) flags |= ax::can_flag::kEsi;
    f.flags = flags;
    f.length = fd
        ? valid_lens[std::uniform_int_distribution<std::size_t>(0u, 15u)(rng)]
        : static_cast<uint8_t>(std::uniform_int_distribution<uint32_t>(0u, 8u)(rng));
    f.can_id = ext
        ? std::uniform_int_distribution<uint32_t>(0u, ax::kExtIdMax)(rng)
        : std::uniform_int_distribution<uint32_t>(0u, ax::kStdIdMax)(rng);
    for (uint8_t i = 0; i < f.length; ++i) {
      f.data[i] = static_cast<uint8_t>(rng() & 0xFFu);
    }

    std::vector<uint8_t> wire;
    const std::size_t n = ax::writeCanFdFrame(wire, f);
    ASSERT_GT(n, 0u);
    auto got = ax::parseCanFdFrame(wire.data(), wire.size());
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got->physical_channel, f.physical_channel);
    EXPECT_EQ(got->address.channel_group, f.address.channel_group);
    EXPECT_EQ(got->address.channel_id_set, f.address.channel_id_set);
    EXPECT_EQ(got->timestamp_ms, f.timestamp_ms);
    EXPECT_EQ(got->flags, f.flags);
    EXPECT_EQ(got->length, f.length);
    EXPECT_EQ(got->can_id, f.can_id);
    for (uint8_t i = 0; i < f.length; ++i) {
      EXPECT_EQ(got->data[i], f.data[i]) << "iter=" << iter << " byte=" << +i;
    }
  }
}
