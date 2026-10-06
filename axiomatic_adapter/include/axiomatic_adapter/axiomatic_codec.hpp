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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_CODEC_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_CODEC_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::can
{

/// @brief Supported Axiomatic Ethernet/CAN converters
enum class AxiomaticModel
{
  AX140900,
  AX142100A,
};

/// @class polymath::can::AxiomaticCodec
/// @brief Wire format of one converter model.
/// A model overrides protocolId, encode, and decodeMessage.
/// Methods are const and keep no state between calls; one instance may be shared across threads.
class AxiomaticCodec
{
public:
  virtual ~AxiomaticCodec() = default;

  /// @brief Encode one CAN frame as one complete protocol message
  virtual std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame) const = 0;

  /// @brief Decode every CAN frame in a buffer of back-to-back protocol messages.
  /// Messages with a Message ID that carries no CAN frames are skipped.
  /// @param data buffer start
  /// @param size buffer length in bytes
  protocol::DecodeResult decode(const uint8_t * data, size_t size) const;

protected:
  /// @brief Protocol ID every message of this model carries
  virtual uint16_t protocolId() const = 0;

  /// @brief Append the CAN frames in one message to result
  /// @return false when the Message ID carries no CAN frames
  virtual bool decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const = 0;
};

/// @brief Construct the codec for a model
std::unique_ptr<const AxiomaticCodec> makeCodec(AxiomaticModel model);

/// @brief Lower-case model names, e.g. "ax142100a", for command line and configuration parsing
std::map<std::string, AxiomaticModel> modelNames();

}  // namespace polymath::can

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_CODEC_HPP_
