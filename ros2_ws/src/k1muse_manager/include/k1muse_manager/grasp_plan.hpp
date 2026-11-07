#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>
#include <k1muse_interfaces/action/arm_command.hpp>

namespace k1muse_manager
{
using Point3 = std::array<double, 3>;
using Pose5 = std::array<double, 5>;
using ArmGoal = k1muse_interfaces::action::ArmCommand::Goal;
// Observation allowance for the old rectangular prefilter, only with full IK.
constexpr double kGraspObservationToleranceM = .002;
enum class ArmStepKind {Pose, Open, Grip};
struct ArmStepSpec
{
  std::string id;
  ArmStepKind kind;
  std::string target;
};
struct ArmStep
{
  std::string id;
  ArmGoal goal;
};
struct GraspProfile
{
  bool calibrated{false};
  Point3 surface_to_tcp{}, minimum{}, maximum{};
  Point3 preferred_tcp{};  // Required from the selected top profile; no installation fallback.
  // Positive approach distance: front uses -X; top uses +Z.
  double approach_m{0.05}, lift_m{0.05}, roll{0}, pitch{0}, open_rad{1.3};
  bool align_lift{true};
  std::map<std::string, Pose5> poses;
  std::vector<ArmStepSpec> pick, exit;
};
struct GraspPlan
{
  Point3 tcp{}, approach{};
  std::vector<ArmStep> pick, exit;
};
bool finite_point(const Point3 & point);
bool within_grasp_region(const Point3 & point, const GraspProfile & profile);
bool surface_to_tcp(const Point3 & point, const GraspProfile & profile, Point3 & tcp);
// Check the current lift first, then nearest legal 5mm-grid heights (.005..495m).
// Different heights must be outside the MCU's existing 2mm arrival tolerance.
// Pure local IK preflight; the caller moves/reobserves before executing a changed height.
bool choose_grasp_lift(
  const Point3 & tcp, const GraspProfile & profile, double current_lift,
  double & lift_goal, GraspPlan & plan, std::string & error);
bool make_front_grasp_plan(
  const Point3 & tcp, const GraspProfile & profile, GraspPlan & plan, std::string & error);
bool make_top_grasp_plan(
  const Point3 & tcp, const GraspProfile & profile, GraspPlan & plan, std::string & error);
// Fixed profile validation runs once at load/reload; expansion assumes it succeeded.
bool validate_grasp_profile(const GraspProfile & profile, std::string & error);
// Shared pure sequence expansion. No ROS entities or motion execution.
bool expand_grasp_plan(
  const Point3 & tcp, const Point3 & approach, const Point3 & lifted,
  const GraspProfile & profile, GraspPlan & plan, std::string & error);
}  // namespace k1muse_manager
