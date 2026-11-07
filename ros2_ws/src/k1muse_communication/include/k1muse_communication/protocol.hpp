#ifndef K1MUSE_COMMUNICATION__PROTOCOL_HPP_
#define K1MUSE_COMMUNICATION__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace k1muse_communication::protocol
{

constexpr uint8_t kSof1 = 0xA5;
constexpr uint8_t kSof2 = 0x5A;
constexpr uint8_t kSourceHost = 0x02;
constexpr uint8_t kSourceMcu = 0x10;
constexpr uint8_t kTargetChassis = 0x01;
constexpr uint8_t kTargetArm = 0x02;
constexpr uint8_t kTargetLift = 0x03;
constexpr uint8_t kChassisStop = 0x00;
constexpr uint8_t kChassisMov = 0x01;
constexpr uint8_t kChassisOdom = 0x02;
constexpr uint8_t kChassisStatus = 0x80;
constexpr uint8_t kLiftStop = 0x00;
constexpr uint8_t kLiftHome = 0x01;
constexpr uint8_t kLiftMoveZ = 0x02;
constexpr uint8_t kLiftStatus = 0x80;
constexpr uint8_t kLiftResult = 0x81;
constexpr uint8_t kArmStop = 0x00;
constexpr uint8_t kArmHome = 0x01;
constexpr uint8_t kArmMovePose = 0x03;
constexpr uint8_t kArmGripper = 0x04;
constexpr uint8_t kArmGrip = 0x07;
constexpr uint8_t kArmTopPickHold = 0x0B;
constexpr uint8_t kArmTopPickPlaceLeft = 0x0C;
constexpr uint8_t kArmTopPickPlaceRight = 0x0D;
constexpr uint8_t kArmPlaceLeft = 0x0E;
constexpr uint8_t kArmPlaceRight = 0x0F;
constexpr uint8_t kArmDemo = 0x83;
constexpr uint8_t kArmStatus = 0x80;
constexpr uint8_t kArmResult = 0x81;
constexpr uint8_t kResultAccepted = 0x01;
constexpr uint8_t kResultRejected = 0x02;
constexpr uint8_t kResultCompleted = 0x03;
constexpr uint8_t kResultAborted = 0x04;
constexpr uint8_t kResultFailed = 0x05;
constexpr uint8_t kResultSuperseded = 0x06;
constexpr uint8_t kMaxPayloadLength = 64;

struct Frame
{
  uint8_t source{};
  uint8_t target{};
  uint8_t command{};
  uint8_t sequence{};
  std::vector<uint8_t> payload;
};

struct ChassisStatus
{
  uint32_t tick_ms{};
  uint8_t state{};
  uint8_t move_cs{};
  uint8_t motor_block_flags{};
  float wcs_vx{};
  float wcs_vy{};
  float omega{};
  float wcs_x{};
  float wcs_y{};
  float wcs_direction{};
};

struct LiftStatus
{
  uint8_t is_homed{};
  uint8_t is_busy{};
  uint8_t has_fault{};
  uint8_t fault_reason{};
  float current_z{};
  float target_z{};
};

struct LiftResult
{
  uint8_t request_seq{};
  uint8_t request_cmd{};
  uint8_t request_source{};
  uint8_t result{};
  uint8_t reject_reason{};
  uint8_t fault_reason{};
};

struct ArmStatus
{
  uint32_t tick_ms{};
  uint8_t state{};
  uint8_t status{};
  uint8_t is_busy{};
  uint8_t has_fault{};
  uint8_t active_cmd{};
  uint8_t active_source{};
  uint8_t active_seq{};
  uint8_t diag_code{};
  float current_x{};
  float current_y{};
  float current_z{};
  float target_x{};
  float target_y{};
  float target_z{};
  float current_gripper_rad{};
  float target_gripper_rad{};
  uint32_t fault_flags{};
};

struct ArmResult
{
  uint32_t tick_ms{};
  uint8_t request_seq{};
  uint8_t request_cmd{};
  uint8_t request_source{};
  uint8_t result{};
  uint8_t reason{};
  uint8_t fault_source{};
  uint8_t state_after{};
  uint8_t diag_code{};
};

struct ParseResult
{
  std::vector<Frame> frames;
  size_t rejected_frames{};
};

struct ArmDemo
{
  uint32_t tick_ms{};
  uint8_t request_seq{}, request_cmd{}, request_source{}, state{};
  uint8_t step{}, step_count{}, holding{}, reason{};
  float progress{};
};

uint8_t crc8_atm(const uint8_t * data, size_t size);
std::vector<uint8_t> encode_frame(const Frame & frame);
Frame make_chassis_mov(uint8_t sequence, double vx, double vy, double omega);
Frame make_chassis_stop(uint8_t sequence);
Frame make_lift_home(uint8_t sequence);
Frame make_lift_move_z(uint8_t sequence, float z);
Frame make_lift_stop(uint8_t sequence);
Frame make_arm_home(uint8_t sequence);
Frame make_arm_move_pose(
  uint8_t sequence, float x, float y, float z, float roll, float pitch);
Frame make_arm_gripper(uint8_t sequence, float gripper_rad);
Frame make_arm_grip(uint8_t sequence);
// TOP_PICK commands carry XYZ+lift (16B); PLACE_LEFT/RIGHT carry only lift (4B).
Frame make_arm_demo(uint8_t sequence, uint8_t command, float x, float y, float z, float lift_z);
Frame make_arm_stop(uint8_t sequence);
bool decode_chassis_status(const Frame & frame, ChassisStatus * status);
bool decode_lift_status(const Frame & frame, LiftStatus * status);
bool decode_lift_result(const Frame & frame, LiftResult * result);
bool decode_arm_status(const Frame & frame, ArmStatus * status);
bool decode_arm_result(const Frame & frame, ArmResult * result);
bool decode_arm_demo(const Frame & frame, ArmDemo * demo);

class StreamParser
{
public:
  ParseResult consume(const uint8_t * data, size_t size);

private:
  std::vector<uint8_t> buffer_;
};

}  // namespace k1muse_communication::protocol

#endif  // K1MUSE_COMMUNICATION__PROTOCOL_HPP_
