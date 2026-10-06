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
#include <stdexcept>
#include <string>

#include "axiomatic_adapter/models/ax140900.hpp"
#include "axiomatic_adapter/models/ax142100a.hpp"

namespace polymath
{
namespace can
{

namespace
{
constexpr AxiomaticCodec AX140900_CODEC{&ax140900::encode, &ax140900::decode};
constexpr AxiomaticCodec AX142100A_CODEC{&ax142100a::encode, &ax142100a::decode};
}  // namespace

const AxiomaticCodec & getCodec(AxiomaticModel model)
{
  switch (model) {
    case AxiomaticModel::AX140900:
      return AX140900_CODEC;
    case AxiomaticModel::AX142100A:
      return AX142100A_CODEC;
  }
  throw std::invalid_argument("Unknown AxiomaticModel " + std::to_string(static_cast<int>(model)));
}

std::map<std::string, AxiomaticModel> modelNames()
{
  return {
    {"ax140900", AxiomaticModel::AX140900},
    {"ax142100a", AxiomaticModel::AX142100A},
  };
}

}  // namespace can
}  // namespace polymath
