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

#include "axiomatic_adapter/axiomatic_codec.hpp"

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "axiomatic_adapter/models/ax140900.hpp"
#include "axiomatic_adapter/models/ax142100a.hpp"

namespace polymath::can
{

protocol::DecodeResult AxiomaticCodec::decode(const uint8_t * data, size_t size) const
{
  protocol::ParsedMessages parsed = protocol::parseMessages(protocolId(), data, size);
  protocol::DecodeResult result{{}, std::move(parsed.diagnostics)};
  for (const auto & message : parsed.messages) {
    if (!decodeMessage(message, result)) {
      result.diagnostics.push_back(
        "SKIP: Message ID " + std::to_string(message.message_id) + " (" + std::to_string(message.body_size) +
        "-byte body) carries no CAN frames");
    }
  }
  return result;
}

namespace
{
using CodecFactory = std::unique_ptr<const AxiomaticCodec> (*)();

template <typename CodecT>
std::unique_ptr<const AxiomaticCodec> make()
{
  return std::make_unique<CodecT>();
}

std::map<std::string, CodecFactory> codecFactories()
{
  return {
    {"ax140900", &make<ax140900::Codec>},
    {"ax142100a", &make<ax142100a::Codec>},
  };
}
}  // namespace

std::unique_ptr<const AxiomaticCodec> makeCodec(const std::string & model)
{
  const auto factories = codecFactories();
  const auto factory = factories.find(model);
  if (factories.end() == factory) {
    throw std::invalid_argument("Unknown Axiomatic model \"" + model + "\"");
  }
  return factory->second();
}

std::vector<std::string> modelNames()
{
  std::vector<std::string> names;
  for (const auto & [name, factory] : codecFactories()) {
    names.push_back(name);
  }
  return names;
}

}  // namespace polymath::can
