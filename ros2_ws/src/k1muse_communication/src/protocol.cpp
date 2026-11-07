#include "k1muse_communication/protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iterator>

namespace k1muse_communication::protocol
{
namespace
{

void append_float(std::vector<uint8_t> * bytes, float value)
{
  uint8_t raw[sizeof(float)];
  std::memcpy(raw, &value, sizeof(value));
  bytes->insert(bytes->end(), std::begin(raw), std::end(raw));
}

float read_float(const std::vector<uint8_t> & bytes, size_t offset)
{
  float value{};
  std::memcpy(&value, bytes.data() + offset, sizeof(value));
  return value;
}

uint32_t read_u32(const std::vector<uint8_t> & bytes, size_t offset)
{
  return static_cast<uint32_t>(bytes[offset]) |
         (static_cast<uint32_t>(bytes[offset + 1]) << 8U) |
         (static_cast<uint32_t>(bytes[offset + 2]) << 16U) |
         (static_cast<uint32_t>(bytes[offset + 3]) << 24U);
}

}  // namespace

uint8_t crc8_atm(const uint8_t * data, size_t size)
{
  uint8_t crc = 0x00;
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ? static_cast<uint8_t>((crc << 1U) ^ 0x07U) :
        static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

std::vector<uint8_t> encode_frame(const Frame & frame)
{
  std::vector<uint8_t> bytes;
  if (frame.payload.size() > kMaxPayloadLength) {
    return bytes;
  }

  bytes.reserve(8U + frame.payload.size());
  bytes.push_back(kSof1);
  bytes.push_back(kSof2);
  bytes.push_back(frame.source);
  bytes.push_back(frame.target);
  bytes.push_back(frame.command);
  bytes.push_back(frame.sequence);
  bytes.push_back(static_cast<uint8_t>(frame.payload.size()));
  bytes.insert(bytes.end(), frame.payload.begin(), frame.payload.end());
  bytes.push_back(crc8_atm(bytes.data() + 2, bytes.size() - 2));
  return bytes;
}

Frame make_chassis_mov(uint8_t sequence, double vx, double vy, double omega)
{
  Frame frame;
  frame.source = kSourceHost;
  frame.target = kTargetChassis;
  frame.command = kChassisMov;
  frame.sequence = sequence;
  frame.payload.reserve(13);
  frame.payload.push_back(0x00);
  append_float(&frame.payload, static_cast<float>(std::atan2(vy, vx)));
  append_float(&frame.payload, static_cast<float>(std::hypot(vx, vy)));
  append_float(&frame.payload, static_cast<float>(omega));
  return frame;
}

Frame make_chassis_stop(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetChassis, kChassisStop, sequence, {}};
}

Frame make_lift_home(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetLift, kLiftHome, sequence, {}};
}

Frame make_lift_move_z(uint8_t sequence, float z)
{
  Frame frame{kSourceHost, kTargetLift, kLiftMoveZ, sequence, {}};
  frame.payload.reserve(sizeof(float));
  append_float(&frame.payload, z);
  return frame;
}

Frame make_lift_stop(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetLift, kLiftStop, sequence, {}};
}

Frame make_arm_home(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetArm, kArmHome, sequence, {}};
}

Frame make_arm_move_pose(
  uint8_t sequence, float x, float y, float z, float roll, float pitch)
{
  Frame frame{kSourceHost, kTargetArm, kArmMovePose, sequence, {}};
  frame.payload.reserve(5U * sizeof(float));
  append_float(&frame.payload, x);
  append_float(&frame.payload, y);
  append_float(&frame.payload, z);
  append_float(&frame.payload, roll);
  append_float(&frame.payload, pitch);
  return frame;
}

Frame make_arm_gripper(uint8_t sequence, float gripper_rad)
{
  Frame frame{kSourceHost, kTargetArm, kArmGripper, sequence, {}};
  frame.payload.reserve(sizeof(float));
  append_float(&frame.payload, gripper_rad);
  return frame;
}

Frame make_arm_grip(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetArm, kArmGrip, sequence, {}};
}

Frame make_arm_stop(uint8_t sequence)
{
  return Frame{kSourceHost, kTargetArm, kArmStop, sequence, {}};
}

Frame make_arm_demo(uint8_t sequence, uint8_t command, float x, float y, float z, float lift_z)
{
  Frame frame{kSourceHost, kTargetArm, command, sequence, {}};
  if(command==kArmPlaceLeft || command==kArmPlaceRight) {
    append_float(&frame.payload, lift_z);
  } else {
    frame.payload.reserve(16);
    for (const float value : {x, y, z, lift_z}) {append_float(&frame.payload, value);}
  }
  return frame;
}

bool decode_arm_demo(const Frame & frame, ArmDemo * demo)
{
  if (!demo || frame.source != kSourceMcu || frame.target != kTargetArm ||
    frame.command != kArmDemo || frame.payload.size() != 16) {return false;}
  demo->tick_ms = read_u32(frame.payload, 0);
  demo->request_seq = frame.payload[4]; demo->request_cmd = frame.payload[5];
  demo->request_source = frame.payload[6]; demo->state = frame.payload[7];
  demo->step = frame.payload[8]; demo->step_count = frame.payload[9];
  demo->holding = frame.payload[10]; demo->reason = frame.payload[11];
  demo->progress = read_float(frame.payload, 12);
  return true;
}

bool decode_chassis_status(const Frame & frame, ChassisStatus * status)
{
  if (status == nullptr || frame.source != kSourceMcu || frame.target != kTargetChassis ||
    frame.command != kChassisStatus || frame.payload.size() != 32U)
  {
    return false;
  }

  status->tick_ms = read_u32(frame.payload, 0);
  status->state = frame.payload[4];
  status->move_cs = frame.payload[5];
  status->motor_block_flags = frame.payload[6];
  status->wcs_vx = read_float(frame.payload, 8);
  status->wcs_vy = read_float(frame.payload, 12);
  status->omega = read_float(frame.payload, 16);
  status->wcs_x = read_float(frame.payload, 20);
  status->wcs_y = read_float(frame.payload, 24);
  status->wcs_direction = read_float(frame.payload, 28);
  return true;
}

bool decode_lift_status(const Frame & frame, LiftStatus * status)
{
  if (status == nullptr || frame.source != kSourceMcu || frame.target != kTargetLift ||
    frame.command != kLiftStatus || frame.payload.size() != 12U)
  {
    return false;
  }

  status->is_homed = frame.payload[0];
  status->is_busy = frame.payload[1];
  status->has_fault = frame.payload[2];
  status->fault_reason = frame.payload[3];
  status->current_z = read_float(frame.payload, 4);
  status->target_z = read_float(frame.payload, 8);
  return true;
}

bool decode_lift_result(const Frame & frame, LiftResult * result)
{
  if (result == nullptr || frame.source != kSourceMcu || frame.target != kTargetLift ||
    frame.command != kLiftResult || frame.payload.size() != 6U)
  {
    return false;
  }

  result->request_seq = frame.payload[0];
  result->request_cmd = frame.payload[1];
  result->request_source = frame.payload[2];
  result->result = frame.payload[3];
  result->reject_reason = frame.payload[4];
  result->fault_reason = frame.payload[5];
  return true;
}

bool decode_arm_status(const Frame & frame, ArmStatus * status)
{
  if (status == nullptr || frame.source != kSourceMcu || frame.target != kTargetArm ||
    frame.command != kArmStatus || frame.payload.size() != 48U)
  {
    return false;
  }

  status->tick_ms = read_u32(frame.payload, 0);
  status->state = frame.payload[4];
  status->status = frame.payload[5];
  status->is_busy = frame.payload[6];
  status->has_fault = frame.payload[7];
  status->active_cmd = frame.payload[8];
  status->active_source = frame.payload[9];
  status->active_seq = frame.payload[10];
  status->diag_code = frame.payload[11];
  status->current_x = read_float(frame.payload, 12);
  status->current_y = read_float(frame.payload, 16);
  status->current_z = read_float(frame.payload, 20);
  status->target_x = read_float(frame.payload, 24);
  status->target_y = read_float(frame.payload, 28);
  status->target_z = read_float(frame.payload, 32);
  status->current_gripper_rad = read_float(frame.payload, 36);
  status->target_gripper_rad = read_float(frame.payload, 40);
  status->fault_flags = read_u32(frame.payload, 44);
  return true;
}

bool decode_arm_result(const Frame & frame, ArmResult * result)
{
  if (result == nullptr || frame.source != kSourceMcu || frame.target != kTargetArm ||
    frame.command != kArmResult || frame.payload.size() != 60U)
  {
    return false;
  }

  result->tick_ms = read_u32(frame.payload, 0);
  result->request_seq = frame.payload[4];
  result->request_cmd = frame.payload[5];
  result->request_source = frame.payload[6];
  result->result = frame.payload[7];
  result->reason = frame.payload[8];
  result->fault_source = frame.payload[9];
  result->state_after = frame.payload[10];
  result->diag_code = frame.payload[11];
  return true;
}

ParseResult StreamParser::consume(const uint8_t * data, size_t size)
{
  ParseResult result;
  constexpr std::array<uint8_t, 2> sof_pattern{kSof1, kSof2};
  buffer_.insert(buffer_.end(), data, data + size);

  while (true) {
    const auto sof = std::search(
      buffer_.begin(), buffer_.end(), sof_pattern.begin(), sof_pattern.end());
    if (sof == buffer_.end()) {
      if (!buffer_.empty() && buffer_.back() == kSof1) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 1);
      } else {
        buffer_.clear();
      }
      return result;
    }
    if (sof != buffer_.begin()) {
      buffer_.erase(buffer_.begin(), sof);
    }
    if (buffer_.size() < 8U) {
      return result;
    }

    const uint8_t payload_length = buffer_[6];
    if (payload_length > kMaxPayloadLength) {
      ++result.rejected_frames;
      buffer_.erase(buffer_.begin());
      continue;
    }

    const size_t frame_length = 8U + payload_length;
    if (buffer_.size() < frame_length) {
      return result;
    }
    const uint8_t expected_crc = crc8_atm(buffer_.data() + 2, 5U + payload_length);
    if (buffer_[frame_length - 1U] != expected_crc) {
      ++result.rejected_frames;
      buffer_.erase(buffer_.begin());
      continue;
    }

    Frame frame;
    frame.source = buffer_[2];
    frame.target = buffer_[3];
    frame.command = buffer_[4];
    frame.sequence = buffer_[5];
    frame.payload.assign(buffer_.begin() + 7, buffer_.begin() + 7 + payload_length);
    result.frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_length);
  }
}

}  // namespace k1muse_communication::protocol
