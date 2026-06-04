#include "axiomatic_adapter/axiomatic_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace polymath::axiomatic {

// ---------------------------------------------------------------------------
// Endianness helpers
// ---------------------------------------------------------------------------

uint16_t readLE16(const uint8_t * src) noexcept {
  return static_cast<uint16_t>(src[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(src[1]) << 8);
}

uint32_t readLE32(const uint8_t * src) noexcept {
  return static_cast<uint32_t>(src[0]) |
         (static_cast<uint32_t>(src[1]) << 8) |
         (static_cast<uint32_t>(src[2]) << 16) |
         (static_cast<uint32_t>(src[3]) << 24);
}

void writeLE16(uint8_t * dst, uint16_t v) noexcept {
  dst[0] = static_cast<uint8_t>(v & 0xFF);
  dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void writeLE32(uint8_t * dst, uint32_t v) noexcept {
  dst[0] = static_cast<uint8_t>(v & 0xFF);
  dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

// ---------------------------------------------------------------------------
// Length validation
// ---------------------------------------------------------------------------

bool isValidCanFdLength(uint8_t len) noexcept {
  // Per spec: 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64.
  if (len <= 8u) {
    return true;
  }
  switch (len) {
    case 12: case 16: case 20: case 24: case 32: case 48: case 64:
      return true;
    default:
      return false;
  }
}

// ---------------------------------------------------------------------------
// Envelope codec
// ---------------------------------------------------------------------------

std::size_t writeMessageHeader(uint8_t * out,
                               MessageId message_id,
                               uint8_t message_version,
                               uint16_t data_length) noexcept {
  std::memcpy(out, kAxiomaticTag.data(), 4);
  writeLE16(out + 4, kProtocolId);
  writeLE16(out + 6, static_cast<uint16_t>(message_id));
  out[8] = message_version;
  writeLE16(out + 9, data_length);
  return kEnvelopeHeaderSize;
}

std::optional<MessageHeader> parseMessageHeader(const uint8_t * data,
                                                std::size_t len) noexcept {
  if (data == nullptr || len < kEnvelopeHeaderSize) {
    return std::nullopt;
  }
  if (!std::equal(kAxiomaticTag.begin(), kAxiomaticTag.end(), data)) {
    return std::nullopt;
  }
  MessageHeader h{};
  h.protocol_id = readLE16(data + 4);
  if (h.protocol_id != kProtocolId) {
    return std::nullopt;
  }
  h.message_id = readLE16(data + 6);
  h.message_version = data[8];
  h.data_length = readLE16(data + 9);
  if (h.data_length > kMaxMessageDataLen) {
    return std::nullopt;
  }
  return h;
}

// ---------------------------------------------------------------------------
// CAN FD Frame codec
// ---------------------------------------------------------------------------

namespace {

// Returns true iff the length field is consistent with the flag bits.
bool lengthIsConsistentWithFlags(uint8_t length, uint8_t flags) noexcept {
  const bool is_error = (flags & can_flag::kError) != 0u;
  const bool is_fd    = (flags & can_flag::kFd) != 0u;
  const bool is_rtr   = (flags & can_flag::kRemote) != 0u;

  if (is_error) {
    // Error/notification messages: 1..64.
    return length >= 1u && length <= 64u;
  }
  if (is_fd) {
    // CAN FD data frame: 0,1..8,12,16,20,24,32,48,64.
    return isValidCanFdLength(length);
  }
  if (is_rtr) {
    // Classical CAN remote frame request: 0..8 (data section is empty regardless,
    // but length carries the requested DLC).
    return length <= 8u;
  }
  // Classical CAN data frame: 0..8.
  return length <= 8u;
}

// Returns the number of payload bytes that should follow the 17-byte header.
// For RTR frames, payload is empty regardless of `length`.
uint8_t payloadBytesFromLengthAndFlags(uint8_t length, uint8_t flags) noexcept {
  if ((flags & can_flag::kRemote) != 0u && (flags & can_flag::kError) == 0u) {
    return 0u;
  }
  return length;
}

bool canIdInRangeForFlags(uint32_t id, uint8_t flags) noexcept {
  return ((flags & can_flag::kExtId) != 0u) ? (id <= kExtIdMax) : (id <= kStdIdMax);
}

}  // namespace

std::size_t writeCanFdFrame(std::vector<uint8_t> & out,
                            const CanFdFrameRecord & f) {
  // Validate before emitting any bytes.
  if (!lengthIsConsistentWithFlags(f.length, f.flags)) {
    return 0u;
  }
  if (!canIdInRangeForFlags(f.can_id, f.flags)) {
    return 0u;
  }
  if (f.physical_channel > 0x1FFFu) {
    return 0u;
  }
  if ((f.reserved_flags & 0xF8u) != 0u) {
    // reserved_flags only has 3 meaningful bits (high bits in PCNFB2)
    return 0u;
  }

  const uint8_t payload_bytes = payloadBytesFromLengthAndFlags(f.length, f.flags);
  const std::size_t prev_size = out.size();
  out.resize(prev_size + kCanFdFrameHeaderSize + payload_bytes);
  uint8_t * p = out.data() + prev_size;

  // PCNFB1, PCNFB2: low 13 bits = physical channel, high 3 bits = reserved flags.
  const uint16_t pcn_word =
      static_cast<uint16_t>(f.physical_channel & 0x1FFFu) |
      static_cast<uint16_t>((f.reserved_flags & 0x07u) << 13);
  writeLE16(p, pcn_word);
  // CGB
  p[2] = f.address.channel_group;
  // CIDSB1..4
  writeLE32(p + 3, f.address.channel_id_set);
  // ATB1..4
  writeLE32(p + 7, f.timestamp_ms);
  // CANFB
  p[11] = f.flags;
  // CANLB
  p[12] = f.length;
  // CANID1..4
  writeLE32(p + 13, f.can_id);
  // Payload
  if (payload_bytes > 0u) {
    std::memcpy(p + kCanFdFrameHeaderSize, f.data.data(), payload_bytes);
  }
  return kCanFdFrameHeaderSize + payload_bytes;
}

std::optional<CanFdFrameRecord> parseCanFdFrame(const uint8_t * data,
                                                std::size_t len) noexcept {
  if (data == nullptr || len < kCanFdFrameHeaderSize) {
    return std::nullopt;
  }
  CanFdFrameRecord f{};
  const uint16_t pcn_word = readLE16(data);
  f.physical_channel = static_cast<uint16_t>(pcn_word & 0x1FFFu);
  f.reserved_flags = static_cast<uint8_t>((pcn_word >> 13) & 0x07u);
  f.address.channel_group = data[2];
  f.address.channel_id_set = readLE32(data + 3);
  f.timestamp_ms = readLE32(data + 7);
  f.flags = data[11];
  f.length = data[12];
  f.can_id = readLE32(data + 13);

  if (!lengthIsConsistentWithFlags(f.length, f.flags)) {
    return std::nullopt;
  }
  if (!canIdInRangeForFlags(f.can_id, f.flags)) {
    return std::nullopt;
  }
  // Reject reserved CAN ID bits set: standard ID must have bits [11..31] = 0;
  // extended ID must have bits [29..31] = 0.
  if ((f.flags & can_flag::kExtId) == 0u) {
    if ((f.can_id & ~uint32_t{kStdIdMax}) != 0u) {
      return std::nullopt;
    }
  } else {
    if ((f.can_id & ~uint32_t{kExtIdMax}) != 0u) {
      return std::nullopt;
    }
  }

  const uint8_t payload_bytes = payloadBytesFromLengthAndFlags(f.length, f.flags);
  if (len < kCanFdFrameHeaderSize + payload_bytes) {
    return std::nullopt;
  }
  if (payload_bytes > 0u) {
    std::memcpy(f.data.data(), data + kCanFdFrameHeaderSize, payload_bytes);
  }
  return f;
}

// ---------------------------------------------------------------------------
// Single-frame CAN FD Stream convenience
// ---------------------------------------------------------------------------

std::vector<uint8_t> encodeCanFdStreamSingleFrame(const CanFdFrameRecord & frame) {
  std::vector<uint8_t> buf;
  buf.reserve(kEnvelopeHeaderSize + kCanFdFrameHeaderSize + kCanFdMaxDataLen);

  // Reserve the envelope header; we'll backfill data_length once we know it.
  buf.resize(kEnvelopeHeaderSize);
  const std::size_t frame_size = writeCanFdFrame(buf, frame);
  if (frame_size == 0u) {
    return {};
  }
  if (frame_size > kMaxMessageDataLen) {
    return {};  // not reachable today — header(17) + max data(64) = 81 < 245
  }
  writeMessageHeader(buf.data(),
                     MessageId::CanFdStream,
                     /*message_version=*/0,
                     static_cast<uint16_t>(frame_size));
  return buf;
}

std::optional<CanFdFrameRecord> parseCanFdStreamSingleFrame(const uint8_t * data,
                                                            std::size_t len) noexcept {
  const auto header = parseMessageHeader(data, len);
  if (!header) {
    return std::nullopt;
  }
  if (header->message_id != static_cast<uint16_t>(MessageId::CanFdStream)) {
    return std::nullopt;
  }
  if (len < kEnvelopeHeaderSize + header->data_length) {
    return std::nullopt;
  }
  // We require exactly one CAN FD Frame fills the data region.
  const auto frame = parseCanFdFrame(data + kEnvelopeHeaderSize, header->data_length);
  if (!frame) {
    return std::nullopt;
  }
  // Sanity: declared data_length should match the frame's full size.
  const uint8_t payload_bytes = payloadBytesFromLengthAndFlags(frame->length, frame->flags);
  if (header->data_length != kCanFdFrameHeaderSize + payload_bytes) {
    return std::nullopt;
  }
  return frame;
}

// ---------------------------------------------------------------------------
// Heartbeat codec (Message ID 4, Message Version 2)
// ---------------------------------------------------------------------------
//
// Wire layout of the 22-byte data field (LSB-first throughout):
//   offset 0   MNB[1..4]    4   Message Number
//   offset 4   TIB[1..4]    4   Time Interval (ms)
//   offset 8   HDB[1..4]    4   Health Data
//   offset 12  CTB          1   Converter Type
//   offset 13  SFB[1..4]    4   Supported Features
//   offset 17  CGMB         1   Channel Group of Main CAN Port Input Filter
//   offset 18  CIDSMB[1..4] 4   Channel ID Set (4-byte CIDS for the Main filter)
//   total                  22

std::vector<uint8_t> encodeHeartbeatV2(const HeartbeatV2 & hb) {
  std::vector<uint8_t> buf(kEnvelopeHeaderSize + kHeartbeatV2DataSize);
  writeMessageHeader(buf.data(),
                     MessageId::Heartbeat,
                     /*message_version=*/kHeartbeatVersion,
                     static_cast<uint16_t>(kHeartbeatV2DataSize));
  uint8_t * p = buf.data() + kEnvelopeHeaderSize;
  writeLE32(p +  0, hb.message_number);
  writeLE32(p +  4, hb.time_interval_ms);
  writeLE32(p +  8, hb.health_data);
  p[12] = hb.converter_type;
  writeLE32(p + 13, hb.supported_features);
  p[17] = hb.filter_channel_group;
  writeLE32(p + 18, hb.filter_channel_id_set);
  return buf;
}

std::optional<HeartbeatV2> parseHeartbeatV2DataField(const uint8_t * data,
                                                     std::size_t len) noexcept {
  if (data == nullptr || len < kHeartbeatV2DataSize) {
    return std::nullopt;
  }
  HeartbeatV2 hb{};
  hb.message_number       = readLE32(data +  0);
  hb.time_interval_ms     = readLE32(data +  4);
  hb.health_data          = readLE32(data +  8);
  hb.converter_type       = data[12];
  hb.supported_features   = readLE32(data + 13);
  hb.filter_channel_group = data[17];
  hb.filter_channel_id_set= readLE32(data + 18);
  return hb;
}

std::optional<HeartbeatV2> parseHeartbeat(const uint8_t * data,
                                          std::size_t len) noexcept {
  const auto header = parseMessageHeader(data, len);
  if (!header) {
    return std::nullopt;
  }
  if (header->message_id != static_cast<uint16_t>(MessageId::Heartbeat)) {
    return std::nullopt;
  }
  if (header->message_version < kHeartbeatVersion) {
    // v1 lacks SFB/CGMB/CIDSMB; we don't try to upcast.
    return std::nullopt;
  }
  if (header->data_length < kHeartbeatV2DataSize) {
    return std::nullopt;
  }
  if (len < kEnvelopeHeaderSize + header->data_length) {
    return std::nullopt;
  }
  // Forward-compat: extra bytes after the v2 region (e.g. v3 NAFB + 10 filters)
  // are tolerated; we ignore them and return only the v2-defined fields.
  return parseHeartbeatV2DataField(data + kEnvelopeHeaderSize, kHeartbeatV2DataSize);
}

}  // namespace polymath::axiomatic
