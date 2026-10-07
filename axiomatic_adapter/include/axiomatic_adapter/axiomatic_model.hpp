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

#ifndef AXIOMATIC_ADAPTER__AXIOMATIC_MODEL_HPP_
#define AXIOMATIC_ADAPTER__AXIOMATIC_MODEL_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath::can
{

/// @class polymath::can::AxiomaticModel
/// @brief One Axiomatic converter model and its wire format.
/// Subclasses override protocolId, encode, and decodeMessage, and encodeRawData when the model forwards raw data.
/// Methods are const and keep no state between calls; one instance may be shared across threads.
class AxiomaticModel
{
public:
  virtual ~AxiomaticModel() = default;

  /// @brief Encode one CAN frame as one complete protocol message
  virtual std::vector<uint8_t> encode(const polymath::socketcan::CanFrame & frame) const = 0;

  /// @brief Encode raw (serial) data as one complete protocol message
  /// @return std::nullopt when the model has no raw data channel
  virtual std::optional<std::vector<uint8_t>> encodeRawData(const std::vector<uint8_t> & data) const;

  /// @brief Whether encodeRawData and decode carry raw data
  bool supportsRawData() const;

  /// @brief Decode every CAN frame and raw data payload in a buffer of back-to-back protocol messages.
  /// Messages with a Message ID that carries neither are skipped.
  /// A trailing incomplete message is left unconsumed; pass it again with the bytes that follow.
  /// @param data buffer start
  /// @param size buffer length in bytes
  protocol::DecodeResult decode(const uint8_t * data, size_t size) const;

protected:
  /// @brief Protocol ID every message of this model carries
  virtual uint16_t protocolId() const = 0;

  /// @brief Append the CAN frames and raw data in one message to result
  /// @return false when the Message ID carries neither
  virtual bool decodeMessage(const protocol::MessageView & message, protocol::DecodeResult & result) const = 0;
};

/// @brief Construct the model with a lower-case name, e.g. "ax142100a"
/// @throws std::invalid_argument for a name not in modelNames()
std::unique_ptr<const AxiomaticModel> makeModel(const std::string & model);

/// @brief Every model name makeModel accepts
std::vector<std::string> modelNames();

}  // namespace polymath::can

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_MODEL_HPP_
