// Pure pick/place logic (no ROS): grasp geometry, straight-line IK, grasp verification.
// SPEC §11.4-11.5, DC-20 (grasp at top - 0.025), DC-27 (clearance rule lives in the node).
#pragma once

#include <string>
#include <vector>

#include "my_robot_manipulation/arm_ik.hpp"

namespace my_robot_manipulation
{

struct GraspPoints
{
  Point grasp;      ///< tool0 when the fingers close
  Point pregrasp;   ///< tool0 above it, start of the vertical descent
};

/// Object = upright cylinder, `centre` = axis point at mid-height (arm_base_link frame).
/// grasp = (axis x, axis y, top - below_top); pregrasp = grasp + (0, 0, approach).
GraspPoints grasp_points(const Point & centre, double height, double below_top, double approach);

/// tool0 straight line from `from` to `to` in `n` equal steps (n >= 1), top-down IK at each.
/// Every step must use the same IK branch (front/back, elbow up/down) as `start` and change no
/// joint by more than `max_step` rad, so the executed motion really is the straight line.
/// out = the n joint vectors (excluding `start`). Returns false + reason otherwise.
bool line_ik(
  const ArmKinematics & arm, const Point & from, const Point & to, int n, double max_step,
  const Joints & start, std::vector<Joints> & out, std::string * why);

struct FingerReading
{
  double left_q{0.0};
  double right_q{0.0};
  double left_contact_age{1e9};    ///< s since the last contact with the target (1e9 = never)
  double right_contact_age{1e9};
};

struct GraspCheck
{
  double q_min{0.015};       ///< stalled on a 0.05 m cylinder: q ~ 0.025
  double q_max{0.035};
  double missed_q{0.010};    ///< closed past this = nothing between the fingers
  double contact_window{0.5};
  bool require_contact{true};
};

enum class GraspVerdict { OK, MISSED, UNCERTAIN };

/// SPEC §11.5: OK needs both fingers stalled in [q_min, q_max] AND (if required) a contact with
/// the target on both fingers within the window; MISSED if either finger closed below missed_q.
GraspVerdict verify_grasp(const FingerReading & f, const GraspCheck & c, std::string * why);

const char * to_string(GraspVerdict v);

}  // namespace my_robot_manipulation
