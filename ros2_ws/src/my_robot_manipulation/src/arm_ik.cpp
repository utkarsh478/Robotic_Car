// Analytic kinematics for the 4-DOF arm. See arm_ik.hpp for frames and conventions.
#include "my_robot_manipulation/arm_ik.hpp"

#include <cmath>
#include <sstream>

namespace my_robot_manipulation
{

const char * to_string(IkStatus s)
{
  switch (s) {
    case IkStatus::OK: return "OK";
    case IkStatus::OUT_OF_REACH: return "OUT_OF_REACH";
    case IkStatus::JOINT_LIMIT: return "JOINT_LIMIT";
  }
  return "UNKNOWN";
}

ArmKinematics::ArmKinematics(const ArmParams & params)
: params_(params) {}

Point ArmKinematics::wrist(const Joints & q) const
{
  // Planar chain in the vertical plane at yaw q[0]: r = horizontal distance, z = height.
  const double a1 = q[1];            // upper arm angle from vertical
  const double a2 = q[1] + q[2];     // forearm angle from vertical
  const double r = params_.L2 * std::sin(a1) + params_.L3 * std::sin(a2);
  const double z = params_.h_s + params_.L2 * std::cos(a1) + params_.L3 * std::cos(a2);
  return {r * std::cos(q[0]), r * std::sin(q[0]), z};
}

ToolPose ArmKinematics::fk(const Joints & q) const
{
  const Point w = wrist(q);
  const double pitch = q[1] + q[2] + q[3];
  const double dr = params_.L_tool * std::sin(pitch);
  ToolPose out;
  out.position = {w.x + dr * std::cos(q[0]), w.y + dr * std::sin(q[0]),
    w.z + params_.L_tool * std::cos(pitch)};
  out.pitch_total = pitch;
  out.yaw = q[0];
  return out;
}

Point ArmKinematics::approach_dir(const ToolPose & t)
{
  return {std::sin(t.pitch_total) * std::cos(t.yaw), std::sin(t.pitch_total) * std::sin(t.yaw),
    std::cos(t.pitch_total)};
}

double ArmKinematics::outward_pitch(const ToolPose & t)
{
  const Point d = approach_dir(t);
  const double r = std::hypot(t.position.x, t.position.y);
  if (r < 1e-9) {return t.pitch_total;}   // on the yaw axis "outward" is undefined: keep the joint value
  const double outward = (d.x * t.position.x + d.y * t.position.y) / r;
  return std::atan2(outward, d.z);
}

bool ArmKinematics::within_limits(const Joints & q, int * violating_joint) const
{
  constexpr double eps = 1e-9;
  for (int i = 0; i < 4; ++i) {
    if (q[i] < params_.limits[i].lower - eps || q[i] > params_.limits[i].upper + eps) {
      if (violating_joint) {*violating_joint = i;}
      return false;
    }
  }
  return true;
}

IkResult ArmKinematics::ik(const Point & p, double pitch_total) const
{
  IkResult res;
  const double L2 = params_.L2;
  const double L3 = params_.L3;

  // 1. Yaw: the arm plane must contain the target. Two ways to do that:
  //    "front": yaw = atan2(y, x), the target is at horizontal distance +r in the arm plane;
  //    "back":  yaw turned by pi, the target is at -r (the arm reaches back over the yaw axis,
  //             shoulder pitched negative). Needed when "front" violates a limit, e.g. a point
  //             that only the backward-leaning configurations can reach.
  // On the yaw axis (r = 0) any yaw works; use 0.
  const double r = std::hypot(p.x, p.y);
  const double yaw_front = (r < 1e-9) ? 0.0 : std::atan2(p.y, p.x);
  const double yaw_back = yaw_front > 0.0 ? yaw_front - kPi : yaw_front + kPi;

  int bad_joint = -1;
  bool reachable = false;
  double last_d = 0.0;
  for (int branch = 0; branch < 2; ++branch) {
    const double q0 = branch == 0 ? yaw_front : yaw_back;
    const double r_signed = branch == 0 ? r : -r;

    // Turning the yaw by pi mirrors the arm plane, so the same physical approach direction
    // has pitch -pitch_total there (sin flips, cos stays).
    const double pitch = branch == 0 ? pitch_total : -pitch_total;

    // 2. Wrist joint = tool0 minus the tool segment along the approach direction.
    const double wr = r_signed - params_.L_tool * std::sin(pitch);
    const double wz = (p.z - params_.h_s) - params_.L_tool * std::cos(pitch);

    // 3. Planar 2-link IK for the wrist point (law of cosines).
    const double d2 = wr * wr + wz * wz;
    last_d = std::sqrt(d2);
    const double D = (d2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
    if (D > 1.0 + 1e-12 || D < -1.0 - 1e-12) {continue;}
    reachable = true;
    const double Dc = std::fmax(-1.0, std::fmin(1.0, D));

    // Two elbow solutions: elbow-up (elbow q >= 0, forearm folds forward/down) first.
    for (int k = 0; k < 2; ++k) {
      const bool elbow_up = (k == 0);
      const double q2 = (elbow_up ? 1.0 : -1.0) * std::acos(Dc);
      // Angle of the wrist point from vertical, minus the forearm's contribution.
      const double q1 = std::atan2(wr, wz) - std::atan2(L3 * std::sin(q2), L2 + L3 * std::cos(q2));
      // The wrist angle is defined up to 2*pi (same physical direction): try all three.
      for (double turn : {0.0, 2.0 * kPi, -2.0 * kPi}) {
        const double q3 = pitch + turn - q1 - q2;
        Joints q{{q0, q1, q2, q3}};
        if (within_limits(q, &bad_joint)) {
          res.status = IkStatus::OK;
          res.q = q;
          res.elbow_up = elbow_up;
          res.message = std::string(branch == 0 ? "front, " : "back, ") + (elbow_up ? "elbow-up" : "elbow-down");
          return res;
        }
      }
    }
  }
  if (!reachable) {
    std::ostringstream ss;
    ss << "wrist distance " << last_d << " m outside [" << std::fabs(L2 - L3) << ", " << (L2 + L3) << "]";
    res.status = IkStatus::OUT_OF_REACH;
    res.message = ss.str();
    return res;
  }
  static const char * names[4] = {"yaw", "shoulder", "elbow", "wrist"};
  res.status = IkStatus::JOINT_LIMIT;
  res.message = std::string("all 4 solutions (front/back x elbow up/down) violate a joint limit (last: ") +
    (bad_joint >= 0 ? names[bad_joint] : "?") + ")";
  return res;
}

}  // namespace my_robot_manipulation
