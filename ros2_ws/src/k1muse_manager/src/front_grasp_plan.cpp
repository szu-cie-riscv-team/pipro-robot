#include "k1muse_manager/grasp_plan.hpp"

#include <cmath>
#include <set>
#include <sstream>

namespace k1muse_manager
{
bool finite_point(const Point3 & p)
{
  for (double v : p) {if (!std::isfinite(v) || !std::isfinite(static_cast<float>(v))) {return false;}}
  return true;
}
bool within_grasp_region(const Point3 & p, const GraspProfile & profile)
{
  if (!finite_point(p)) {return false;}
  for (size_t i = 0; i < 3; ++i) {
    if (p[i] < profile.minimum[i] || p[i] > profile.maximum[i]) {return false;}
  }
  return true;
}
bool surface_to_tcp(const Point3 & p, const GraspProfile & profile, Point3 & tcp)
{
  for (size_t i = 0; i < 3; ++i) {tcp[i] = p[i] + profile.surface_to_tcp[i];}
  return finite_point(p) && finite_point(tcp);
}
bool validate_grasp_profile(const GraspProfile & profile, std::string & error)
{
  auto reject = [&](const std::string & why) {error=why;return false;};
  if (!std::isfinite(profile.approach_m) || profile.approach_m<=0 ||
    !std::isfinite(profile.lift_m) || !std::isfinite(static_cast<float>(profile.roll)) ||
    !std::isfinite(static_cast<float>(profile.pitch)) ||
    !std::isfinite(profile.open_rad) || profile.open_rad<0 || profile.open_rad>2)
  {return reject("invalid grasp profile distances/angle");}
  if (!profile.poses.count("ready") || !profile.poses.count("carry") ||
    profile.pick.empty() || profile.exit.empty() || profile.pick.size()>32 || profile.exit.size()>32)
  {return reject("missing ready/carry pose or invalid sequence length");}
  if (profile.calibrated) {
    for (size_t i=0;i<3;++i) {
      if (profile.minimum[i]>=profile.maximum[i]) {return reject("invalid calibrated grasp region");}
    }
  }
  std::set<std::string> names;
  bool opened=false,contact=false;
  std::string last_pose;
  for (const auto * sequence : {&profile.pick,&profile.exit}) {
    const bool exiting=sequence==&profile.exit;
    for (size_t i=0;i<sequence->size();++i) {
      const auto & spec=(*sequence)[i];
      if (spec.id.empty() || !names.insert(spec.id).second) {return reject("invalid/duplicate ARM step id: "+spec.id);}
      if (exiting && spec.kind!=ArmStepKind::Pose) {return reject("exit must preserve holding: "+spec.id);}
      if (!exiting && ((spec.kind==ArmStepKind::Grip)!=(i+1==sequence->size()))) {
        return reject("pick must end in GRIP: "+spec.id);
      }
      if (spec.kind==ArmStepKind::Pose) {
        if (spec.target!="contact" && spec.target!="approach" && spec.target!="lifted") {
          const auto found=profile.poses.find(spec.target);
          if (found==profile.poses.end()) {return reject("unknown pose: "+spec.target);}
          const auto & p=found->second;
          if (!finite_point({p[0],p[1],p[2]}) ||
            !std::isfinite(static_cast<float>(p[3])) || !std::isfinite(static_cast<float>(p[4])) ||
            (profile.calibrated && !within_grasp_region({p[0],p[1],p[2]},profile)))
          {return reject("invalid configured pose: "+spec.target);}
        }
        last_pose=spec.target;
        if (spec.target=="contact") {
          if (!opened || exiting) {return reject("contact requires open gripper in pick");}
          contact=true;
        }
      } else if (spec.kind==ArmStepKind::Open) {opened=true;}
      else if (!opened || !contact || last_pose!="contact") {return reject("GRIP must follow contact");}
    }
  }
  if (profile.exit.back().target!="carry") {return reject("exit must end in carry pose");}
  error.clear();return true;
}

bool expand_grasp_plan(
  const Point3 & tcp, const Point3 & approach, const Point3 & lifted,
  const GraspProfile & profile, GraspPlan & output, std::string & error)
{
  // The profile was validated at load/reload; only this observation varies.
  if (!within_grasp_region(tcp,profile) || !within_grasp_region(approach,profile)) {
    std::ostringstream detail;
    detail << "contact/approach outside grasp region; tcp=[" << tcp[0] << ',' << tcp[1] << ',' << tcp[2]
      << "] approach=[" << approach[0] << ',' << approach[1] << ',' << approach[2]
      << "] min=[" << profile.minimum[0] << ',' << profile.minimum[1] << ',' << profile.minimum[2]
      << "] max=[" << profile.maximum[0] << ',' << profile.maximum[1] << ',' << profile.maximum[2] << ']';
    error=detail.str();return false;
  }
  GraspPlan plan;plan.tcp=tcp;plan.approach=approach;
  plan.pick.reserve(profile.pick.size());plan.exit.reserve(profile.exit.size());
  for (const auto * sequence : {&profile.pick,&profile.exit}) {
    auto & steps=sequence==&profile.pick?plan.pick:plan.exit;
    for (const auto & spec : *sequence) {
      ArmGoal goal;
      if (spec.kind==ArmStepKind::Pose) {
        Pose5 pose;
        if (spec.target=="contact" || spec.target=="approach" || spec.target=="lifted") {
          const auto & point=spec.target=="contact"?tcp:spec.target=="approach"?approach:lifted;
          if (spec.target=="lifted" && !within_grasp_region(point,profile)) {
            error="lifted point outside grasp region";return false;
          }
          pose={point[0],point[1],point[2],profile.roll,profile.pitch};
        } else {pose=profile.poses.at(spec.target);}
        goal.command=ArmGoal::MOVE_POSE;
        goal.x=pose[0];goal.y=pose[1];goal.z=pose[2];goal.roll=pose[3];goal.pitch=pose[4];
      } else if (spec.kind==ArmStepKind::Open) {
        goal.command=ArmGoal::GRIPPER;goal.target_rad=profile.open_rad;
      } else {goal.command=ArmGoal::GRIP;}
      steps.push_back({spec.id,goal});
    }
  }
  output=std::move(plan);error.clear();return true;
}
bool make_front_grasp_plan(
  const Point3 & tcp, const GraspProfile & profile, GraspPlan & plan, std::string & error)
{
  return expand_grasp_plan(tcp, {tcp[0] - profile.approach_m, tcp[1], tcp[2]},
    {tcp[0], tcp[1], tcp[2] + profile.lift_m}, profile, plan, error);
}
}  // namespace k1muse_manager
