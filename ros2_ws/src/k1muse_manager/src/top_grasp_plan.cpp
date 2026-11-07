#include "k1muse_manager/grasp_plan.hpp"

#include <cmath>

namespace k1muse_manager
{
bool make_top_grasp_plan(
  const Point3 & tcp, const GraspProfile & profile, GraspPlan & plan, std::string & error)
{
  if (!std::isfinite(profile.approach_m) || profile.approach_m <= 0 ||
    !std::isfinite(profile.lift_m) || profile.lift_m <= 0)
  {error = "top approach/lift distances must be positive"; return false;}
  return expand_grasp_plan(tcp, {tcp[0], tcp[1], tcp[2] + profile.approach_m},
    {tcp[0], tcp[1], tcp[2] + profile.lift_m}, profile, plan, error);
}
}  // namespace k1muse_manager
