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

#include "socketcan_adapter/can_frame.hpp"

#include <linux/can/error.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace polymath
{
namespace socketcan
{

CanFrame::CanFrame()
: frame_{}
{}

CanFrame::CanFrame(const struct can_frame & frame)
: frame_(frame)
{}

CanFrame::CanFrame(
  const canid_t raw_id, const std::array<unsigned char, CAN_MAX_DLC> & data, const uint64_t & timestamp, uint8_t len)
: timestamp_(timestamp)
{
  set_can_id(raw_id);
  std::copy(data.begin(), data.end(), frame_.data);
  set_len(len);
}

CanFrame::CanFrame(
  const canid_t id,
  const std::array<unsigned char, CAN_MAX_DLC> & data,
  const uint64_t & timestamp,
  FrameType & frame_type,
  IdType & frame_id_type,
  uint8_t len)
: timestamp_(timestamp)
{
  set_can_id(id);
  set_data(data);
  set_len(len);

  switch (frame_type) {
    case FrameType::DATA:
      set_type_data();
      break;
    case FrameType::ERROR:
      set_type_error();
      break;
    case FrameType::REMOTE:
      set_type_remote();
      break;
    default:
      throw std::logic_error("CanID: No Such Type");
  }

  switch (frame_id_type) {
    case IdType::EXTENDED:
      set_id_as_extended();
      break;
    case IdType::STANDARD:
      set_id_as_standard();
      break;
    default:
      throw std::logic_error("CanIdType: No Such Type");
  }
}

void CanFrame::set_id_as_extended()
{
  frame_.can_id |= CAN_EFF_FLAG;
}

void CanFrame::set_id_as_standard()
{
  frame_.can_id &= ~CAN_EFF_FLAG;
}

void CanFrame::set_type_error()
{
  // Clear RTR Flag
  frame_.can_id &= ~CAN_RTR_FLAG;
  // Set ERR Flag
  frame_.can_id |= CAN_ERR_FLAG;
}

void CanFrame::set_type_remote()
{
  // Clear Error Flag
  frame_.can_id &= ~CAN_ERR_FLAG;
  // Set RTR flag
  frame_.can_id |= CAN_RTR_FLAG;
}

void CanFrame::set_type_data()
{
  frame_.can_id &= ~CAN_ERR_FLAG;
  frame_.can_id &= ~CAN_RTR_FLAG;
}

void CanFrame::set_can_id(const canid_t & id)
{
  /// TODO: How do we separate extended and sff here?
  /// Assuming EFF will always work for now
  // Clear and save
  frame_.can_id &= ~CAN_EFF_MASK;
  frame_.can_id |= (CAN_EFF_MASK & id);
}

void CanFrame::set_len(unsigned char len)
{
  frame_.len = len;
}

void CanFrame::set_data(const std::array<unsigned char, CAN_MAX_DLC> & data)
{
  std::copy(data.begin(), data.end(), frame_.data);
}

void CanFrame::set_timestamp(const uint64_t & timestamp)
{
  timestamp_ = timestamp;
}

IdType CanFrame::get_id_type() const
{
  // Frame id type is defined as bit 31
  return IdType((frame_.can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG);
}

FrameType CanFrame::get_frame_type() const
{
  // Frame type is defined as bits 29 and 30
  return FrameType((frame_.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)) >> CAN_EFF_ID_BITS);
}

canid_t CanFrame::get_id() const
{
  canid_t mask = get_id_type() == IdType::EXTENDED ? CAN_EFF_MASK : CAN_SFF_MASK;

  return frame_.can_id & mask;
}

canid_t CanFrame::get_masked_id(uint8_t bit_length, uint8_t bit_offset) const
{
  // Set the bit length to all be passthrough and then shift by bit offset
  canid_t mask = ((1 << bit_length) - 1) << bit_offset;

  return frame_.can_id & mask;
}

struct can_frame CanFrame::get_frame() const
{
  // Return the frame as a copy, all work done to the underlying frame should be through functions
  return frame_;
}

unsigned char CanFrame::get_len() const
{
  // If len is set, return len, otherwise return dlc assuming one of the two is correct
  return static_cast<uint8_t>(frame_.len) > 0 ? frame_.len : frame_.can_dlc;
}

bool CanFrame::set_frame(const struct can_frame & frame)
{
  frame_ = frame;
  return true;
}

std::string CanFrame::get_error() const
{
  if (get_frame_type() != FrameType::ERROR) {
    return std::string("");
  }

  // These are a small subset of errors from linux/can/error.h
  // Add aditional errors as needed
  if (frame_.can_id & CAN_ERR_BUSERROR) {
    return std::string("CAN bus error detected.");
  }
  if (frame_.can_id & CAN_ERR_CRTL) {
    return std::string("CAN controller error.");
  }
  if (frame_.can_id & CAN_ERR_PROT) {
    return std::string("CAN protocol error.");
  }
  if (frame_.can_id & CAN_ERR_TRX) {
    return std::string("CAN transceiver error.");
  }

  // Empty string if none
  return std::string("Error Frame Id: ") + std::to_string(frame_.can_id);
}

const std::array<unsigned char, CAN_MAX_DLC> CanFrame::get_data() const
{
  std::array<unsigned char, CAN_MAX_DLC> data;
  std::copy(std::begin(frame_.data), std::end(frame_.data), data.begin());

  return data;
}

}  // namespace socketcan
}  // namespace polymath
