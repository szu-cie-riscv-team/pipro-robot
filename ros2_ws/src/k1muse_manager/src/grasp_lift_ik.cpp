#include "k1muse_manager/grasp_plan.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

extern "C" {
#include "so101_algorithm.h"
#include "mdl_so101_arm_config.h"
}

namespace k1muse_manager
{
namespace
{
using Joints = std::array<float, SO101_ACTIVE_JOINT_COUNT>;
// MCU mdl_lift_config.h::LIFT_REACHED_EPS_M; corrections inside this band may not move.
constexpr double kLiftReachedToleranceM = .002;

bool solve_pose(
  const robot_pose_target_t & pose, Joints & seed, so101_pose_branch_t & branch)
{
  // Match mdl_so101_arm.c's gross bounds and solve_pose acceptance.
  const auto & p = pose.position;
  const float radius = std::hypot(p.x, p.y);
  if (p.x < SO101_ARM_WORKSPACE_X_MIN_M || p.x > SO101_ARM_WORKSPACE_X_MAX_M ||
    p.y < SO101_ARM_WORKSPACE_Y_MIN_M || p.y > SO101_ARM_WORKSPACE_Y_MAX_M ||
    p.z < SO101_ARM_WORKSPACE_Z_MIN_M || p.z > SO101_ARM_WORKSPACE_Z_MAX_M ||
    radius < SO101_ARM_WORKSPACE_R_MIN_M || radius > SO101_ARM_WORKSPACE_R_MAX_M)
  {return false;}
  Joints result{};
  robot_ik_info_t info{};
  if (so101_pose_ik_branch(&pose, seed.data(), nullptr, branch, result.data(), &info) != ROBOT_OK ||
    info.position_error_m > SO101_ARM_REACH_IK_TOLERANCE_M ||
    info.joint_limit_margin_rad < SO101_ARM_REACH_HARD_LIMIT_MARGIN_RAD)
  {return false;}
  seed = result;
  branch = static_cast<so101_pose_branch_t>(info.solution_branch);
  return true;
}

bool reachable_plan(const GraspPlan & plan, const Pose5 & ready, std::string & error)
{
  Joints seed{SO101_ARM_HOME_JOINT_1_RAD, SO101_ARM_HOME_JOINT_2_RAD,
    SO101_ARM_HOME_JOINT_3_RAD, SO101_ARM_HOME_JOINT_4_RAD, SO101_ARM_HOME_JOINT_5_RAD};
  robot_pose_target_t previous{{float(ready[0]), float(ready[1]), float(ready[2])},
    float(ready[3]), float(ready[4])};
  auto branch = SO101_POSE_BRANCH_ANY;
  if (!solve_pose(previous, seed, branch)) {error = "IK unreachable ready pose"; return false;}
  for (const auto * sequence : {&plan.pick, &plan.exit}) {
    for (const auto & step : *sequence) {
      const auto & g = step.goal;
      if (g.command != ArmGoal::MOVE_POSE) {continue;}
      const robot_pose_target_t target{{g.x, g.y, g.z}, g.roll, g.pitch};
      mat4f_t fk{};
      if (so101_fk_compute(seed.data(), &fk) != ROBOT_OK) {error = "IK seed FK failed"; return false;}
      const robot_pose_target_t start{{fk.m[0][3], fk.m[1][3], fk.m[2][3]},
        seed[SO101_JOINT_WRIST_ROLL], seed[SO101_JOINT_SHOULDER_LIFT] +
        seed[SO101_JOINT_ELBOW_FLEX] + seed[SO101_JOINT_WRIST_FLEX]};
      const float dx = g.x - start.position.x, dy = g.y - start.position.y;
      const float dz = g.z - start.position.z;
      const float horizontal = std::hypot(dx, dy), distance = std::hypot(horizontal, dz);
      constexpr float tau = 6.2831853071795864769f;
      const bool cartesian = horizontal >= SO101_ARM_HORIZONTAL_MIN_DISTANCE_M &&
        distance <= SO101_ARM_HORIZONTAL_MAX_DISTANCE_M &&
        std::abs(dz) <= horizontal * SO101_ARM_HORIZONTAL_MAX_VERTICAL_RATIO &&
        std::abs(std::remainder(g.roll - previous.roll_rad, tau)) <= SO101_ARM_HORIZONTAL_ORIENTATION_EPS_RAD &&
        std::abs(std::remainder(g.pitch - previous.pitch_rad, tau)) <= SO101_ARM_HORIZONTAL_ORIENTATION_EPS_RAD;
      branch = SO101_POSE_BRANCH_ANY;
      if (cartesian) {
        const unsigned samples = std::max(unsigned(SO101_ARM_HORIZONTAL_PREFLIGHT_MIN_SAMPLES),
          unsigned(std::ceil(distance / SO101_ARM_HORIZONTAL_PREFLIGHT_STEP_M)));
        // Firmware fixes the elbow branch within each short Cartesian leg.
        for (unsigned i = 0; i <= samples; ++i) {
          const float ratio = float(i) / samples;
          const robot_pose_target_t sample{{start.position.x + ratio * dx,
            start.position.y + ratio * dy, start.position.z + ratio * dz},
            start.roll_rad + ratio * (g.roll - start.roll_rad),
            start.pitch_rad + ratio * (g.pitch - start.pitch_rad)};
          if (!solve_pose(sample, seed, branch)) {
            error = "IK unreachable Cartesian step=" + step.id; return false;
          }
        }
      } else if (!solve_pose(target, seed, branch)) {
        error = "IK unreachable pose step=" + step.id; return false;
      }
      previous = target;
    }
  }
  return true;
}
}  // namespace

bool choose_grasp_lift(
  const Point3 & tcp, const GraspProfile & profile, double current_lift,
  double & lift_goal, GraspPlan & plan, std::string & error)
{
  if (!finite_point(tcp) || !std::isfinite(current_lift) ||
    current_lift < .003 || current_lift > .497)
  {error = "invalid target/current lift for IK selection"; return false;}
  // Preserve measured TCP and firmware limits. Only relax the old reference
  // rectangle; every accepted pose/path must still pass reachable_plan().
  auto observed_profile = profile;
  for (size_t i = 0; i < 3; ++i) {
    observed_profile.minimum[i] -= kGraspObservationToleranceM;
    observed_profile.maximum[i] += kGraspObservationToleranceM;
  }
  // The measured height is always tested first. Remaining candidates are legal
  // protocol heights, ordered by movement; corrections must exceed MCU arrival tolerance.
  std::array<double, 100> heights{};
  heights[0] = current_lift;
  for (unsigned i = 0; i <= 98; ++i) {heights[i + 1] = double(float(.005 + i * .005));}
  std::sort(heights.begin() + 1, heights.end(), [current_lift](double a, double b) {
    const double da = std::abs(a - current_lift), db = std::abs(b - current_lift);
    return da == db ? a < b : da < db;
  });
  std::string current_error;
  for (size_t i=0;i<heights.size();++i) {
    const double height=heights[i];
    // Test the actual measured height unchanged first. Never plan a correction
    // whose assumed Z change the MCU can accept without actually moving.
    if(i>0 && std::abs(height-current_lift)<=kLiftReachedToleranceM) {continue;}
    const Point3 shifted{tcp[0], tcp[1], tcp[2] + (current_lift - height)};
    GraspPlan candidate;
    if (make_front_grasp_plan(shifted, observed_profile, candidate, error) &&
      reachable_plan(candidate, profile.poses.at("ready"), error))
    {
      lift_goal = height; plan = std::move(candidate); error.clear(); return true;
    }
    if (current_error.empty()) {current_error = error;}
  }
  error = "no IK-feasible lift in 0.005..0.495m (5mm grid); current: " + current_error;
  return false;
}
}  // namespace k1muse_manager
