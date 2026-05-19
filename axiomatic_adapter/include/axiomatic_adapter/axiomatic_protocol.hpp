// Axiomatic AX140970 Dual CAN FD to Ethernet Converter — wire protocol codec.
//
// Pure data-in / data-out: no sockets, no threads, no ROS, no logging.
// Every parse function returns std::optional and rejects malformed input.
// Every encode function appends to a caller-provided buffer.
//
// Reference: "Ethernet to CAN Converter Communication Protocol", v6, Apr 2025.

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace polymath::axiomatic {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

inline constexpr std::array<uint8_t, 4> kAxiomaticTag{'A', 'X', 'I', 'O'};
inline constexpr uint16_t kProtocolId = 14010;       // 0x36BA, LSB-first on the wire
inline constexpr std::size_t kEnvelopeHeaderSize = 11;
inline constexpr std::size_t kMaxMessageSize = 256;  // envelope cap
inline constexpr std::size_t kMaxMessageDataLen = kMaxMessageSize - kEnvelopeHeaderSize;
inline constexpr std::size_t kCanFdFrameHeaderSize = 17;
inline constexpr std::size_t kCanFdMaxDataLen = 64;

// ---------------------------------------------------------------------------
// Message IDs (Protocol ID 14010, current versions per spec table)
// ---------------------------------------------------------------------------

enum class MessageId : uint16_t {
  Undefined = 0,
  CanNotificationStream = 1,  // deprecated; do not emit
  StatusRequest = 2,
  StatusResponse = 3,
  Heartbeat = 4,
  CanFdStream = 5,
};

// ---------------------------------------------------------------------------
// CAN flag bits (CANFB byte in the CAN FD Frame)
// ---------------------------------------------------------------------------

namespace can_flag {
inline constexpr uint8_t kError = 0x80;     // ERR_Bit: error/notification message
inline constexpr uint8_t kExtId = 0x40;     // EID_Bit: 29-bit identifier
inline constexpr uint8_t kRemote = 0x20;    // RF_Bit:  remote transmission request
inline constexpr uint8_t kFd = 0x10;        // FDF_Bit: CAN FD frame
inline constexpr uint8_t kBrs = 0x08;       // BRS_Bit: bit rate switch (data phase)
inline constexpr uint8_t kEsi = 0x04;       // ESI_Bit: error state indicator
}  // namespace can_flag

// ---------------------------------------------------------------------------
// CAN ID limits
// ---------------------------------------------------------------------------

inline constexpr uint32_t kStdIdMax = 0x7FF;        // 11-bit
inline constexpr uint32_t kExtIdMax = 0x1FFFFFFF;   // 29-bit

// ---------------------------------------------------------------------------
// Supported Features bitmasks (32-bit, LSB-first on the wire)
// ---------------------------------------------------------------------------

namespace supported_features {
inline constexpr uint32_t kCanFdStream = 0x00000001;          // node supports Msg ID 5
inline constexpr uint32_t kOneFramePerMessage = 0x00000002;   // request: 1 frame per stream message
}  // namespace supported_features

// ---------------------------------------------------------------------------
// Heartbeat (Message ID 4) — Message Version 2
// ---------------------------------------------------------------------------
//
// 22-byte data field per spec. Used host→device for keepalive (UDP idles out
// at 10 s of inactivity device-side) and device→host for status (1 Hz).
//
// Versions 1 and 3 exist (v1 has no SFB/CGMB/CIDSMB; v3 adds variable-length
// additional filter addresses). We standardize on v2 as the universal common
// denominator: the device speaks it natively (verified by probe), and v2 is
// long-lived — newer protocols stay backward-compatible per spec.

inline constexpr uint8_t  kHeartbeatVersion = 2;
inline constexpr std::size_t kHeartbeatV2DataSize = 22;

struct HeartbeatV2 {
  uint32_t message_number = 0;       // free-running counter, increments per outbound HB
  uint32_t time_interval_ms = 1000;  // ms since previous heartbeat (≈1000 for 1 Hz)
  uint32_t health_data = 0;          // 4-byte Health Data field; host: 0 (no health to report)
  uint8_t  converter_type = 0;       // we are a host, not a converter; spec says 0 in that case
  uint32_t supported_features = 0;   // bitmask from supported_features::*
  uint8_t  filter_channel_group = 0; // Main CAN Port Input Filter Address — Channel Group
  uint32_t filter_channel_id_set = 0;// …and Channel ID Set; (0,0) means NULL filter (accept all)
};

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

// Envelope header (11 bytes on the wire).
struct MessageHeader {
  uint16_t protocol_id;
  uint16_t message_id;
  uint8_t  message_version;
  uint16_t data_length;
};

// CAN routing address: (Channel Group, Channel ID Set bitmap).
// The CIDS is a 32-bit OR-able bitmap; channel N is selected by bit (N - 1).
struct ChannelAddress {
  uint8_t  channel_group;
  uint32_t channel_id_set;
};

inline constexpr bool addressesMatch(ChannelAddress a, ChannelAddress b) {
  return a.channel_group == b.channel_group &&
         (a.channel_id_set & b.channel_id_set) != 0u;
}

// Decoded CAN FD frame (the 17-byte header + payload).
struct CanFdFrameRecord {
  uint16_t physical_channel = 0;        // 0..8191; 0 means "unused/undefined"
  uint8_t  reserved_flags = 0;          // top 3 bits of PCNFB; spec-reserved, kept for fidelity
  ChannelAddress address{};
  uint32_t timestamp_ms = 0;            // device free-running ms counter
  uint8_t  flags = 0;                   // bitmask of can_flag::*
  uint8_t  length = 0;                  // payload length (validated against flags)
  uint32_t can_id = 0;                  // std (≤0x7FF) or ext (≤0x1FFFFFFF) per kExtId flag
  std::array<uint8_t, kCanFdMaxDataLen> data{};

  bool isError() const { return (flags & can_flag::kError) != 0u; }
  bool isExtId() const { return (flags & can_flag::kExtId) != 0u; }
  bool isRemote() const { return (flags & can_flag::kRemote) != 0u; }
  bool isFd() const { return (flags & can_flag::kFd) != 0u; }
  bool isBrs() const { return (flags & can_flag::kBrs) != 0u; }
  bool isEsi() const { return (flags & can_flag::kEsi) != 0u; }
};

// ---------------------------------------------------------------------------
// Endianness helpers (LSB-first, the protocol default)
// ---------------------------------------------------------------------------

uint16_t readLE16(const uint8_t * src) noexcept;
uint32_t readLE32(const uint8_t * src) noexcept;
void writeLE16(uint8_t * dst, uint16_t v) noexcept;
void writeLE32(uint8_t * dst, uint32_t v) noexcept;

// ---------------------------------------------------------------------------
// CAN FD length validation
// ---------------------------------------------------------------------------

// Returns true iff `len` is a valid CAN FD payload length (0,1..8,12,16,20,24,32,48,64).
bool isValidCanFdLength(uint8_t len) noexcept;

// ---------------------------------------------------------------------------
// Envelope codec
// ---------------------------------------------------------------------------

// Writes the 11-byte envelope header into `out` (which must have ≥ 11 bytes).
// Returns the number of bytes written (always 11).
std::size_t writeMessageHeader(uint8_t * out,
                               MessageId message_id,
                               uint8_t message_version,
                               uint16_t data_length) noexcept;

// Parses an envelope header. Validates AXIO tag + Protocol ID. Returns nullopt if:
//   - buffer < 11 bytes
//   - tag mismatch
//   - protocol_id != 14010
//   - declared data_length would exceed kMaxMessageDataLen (245)
std::optional<MessageHeader> parseMessageHeader(const uint8_t * data,
                                                std::size_t len) noexcept;

// ---------------------------------------------------------------------------
// CAN FD Frame codec (one record within a CAN FD Stream message)
// ---------------------------------------------------------------------------

// Encodes a CAN FD Frame record (17-byte header + length data bytes) into `out`.
// Returns the number of bytes written, or 0 on validation failure (bad ID range,
// bad DLC, etc.) — caller MUST check and handle.
std::size_t writeCanFdFrame(std::vector<uint8_t> & out,
                            const CanFdFrameRecord & frame);

// Parses a CAN FD Frame record. Returns nullopt if buffer too short, length
// inconsistent with flags, or reserved bits set.
std::optional<CanFdFrameRecord> parseCanFdFrame(const uint8_t * data,
                                                std::size_t len) noexcept;

// ---------------------------------------------------------------------------
// Convenience: full single-frame CAN FD Stream message
// ---------------------------------------------------------------------------

// Encodes a complete CAN FD Stream message (envelope + exactly one CAN FD Frame).
// Sets the One-Frame-per-Message scenario implied by Q1-locked design choices.
// Returns the encoded bytes, or empty vector on encode failure.
std::vector<uint8_t> encodeCanFdStreamSingleFrame(const CanFdFrameRecord & frame);

// Parses a CAN FD Stream message containing exactly one CAN FD Frame.
// Returns nullopt if envelope is malformed, message_id is not CanFdStream, or the
// embedded frame is malformed.
std::optional<CanFdFrameRecord> parseCanFdStreamSingleFrame(const uint8_t * data,
                                                            std::size_t len) noexcept;

// ---------------------------------------------------------------------------
// Heartbeat codec
// ---------------------------------------------------------------------------

// Encodes a complete Heartbeat (Msg ID 4, Version 2) message: envelope + 22-byte
// data field. Returns the 33-byte wire buffer. Never fails (no validation hooks).
std::vector<uint8_t> encodeHeartbeatV2(const HeartbeatV2 & hb);

// Parses just the 22-byte Heartbeat v2 data field (caller has already validated
// envelope + sliced out the data section). Returns nullopt if `len` < 22.
std::optional<HeartbeatV2> parseHeartbeatV2DataField(const uint8_t * data,
                                                     std::size_t len) noexcept;

// Convenience: validates envelope, requires Msg ID = 4 and Version ≥ 2, and
// returns the parsed Heartbeat. Returns nullopt on any failure. Versions > 2
// are accepted (forward compatibility); only the v2-defined fields are decoded.
std::optional<HeartbeatV2> parseHeartbeat(const uint8_t * data,
                                          std::size_t len) noexcept;

}  // namespace polymath::axiomatic

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_PROTOCOL_HPP_
