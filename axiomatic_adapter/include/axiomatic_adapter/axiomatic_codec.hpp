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
#include <string>
#include <vector>

#include "axiomatic_adapter/axiomatic_protocol.hpp"
#include "socketcan_adapter/can_frame.hpp"

namespace polymath
{
namespace can
{

/// @brief Supported Axiomatic Ethernet/CAN converters
enum class AxiomaticModel
{
  AX140900,
  AX142100A,
};

/// @brief Wire format of one converter model
struct AxiomaticCodec
{
  /// @brief Encode one CAN frame as one complete protocol message
  std::vector<uint8_t> (*encode)(const polymath::socketcan::CanFrame & frame);

  /// @brief Decode every CAN frame in a buffer of back-to-back protocol messages
  protocol::DecodeResult (*decode)(const uint8_t * data, size_t size);
};

/// @brief Look up the codec for a model
const AxiomaticCodec & getCodec(AxiomaticModel model);

/// @brief Lower-case model names, e.g. "ax142100a", for command line and configuration parsing
std::map<std::string, AxiomaticModel> modelNames();

}  // namespace can
}  // namespace polymath

#endif  // AXIOMATIC_ADAPTER__AXIOMATIC_CODEC_HPP_
