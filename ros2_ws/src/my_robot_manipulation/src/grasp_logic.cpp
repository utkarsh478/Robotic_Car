#include "my_robot_manipulation/grasp_logic.hpp"

#include <cmath>
#include <sstream>

namespace my_robot_manipulation
{

GraspPoints grasp_points(const Point & centre, double height, double below_top, double approach)
{
  GraspPoints g;
  g.grasp = {centre.x, centre.y, centre.z + height / 2.0 - below_top};
  g.pregrasp = {g.grasp.x, g.grasp.y, g.grasp.z + approach};
  return g;
}

// The IK branch is identified by IkResult::message ("front, elbow-up" etc.).
bool line_ik(
  const ArmKinematics & arm, const Point & from, const Point & to, int n, double max_step,
  const Joints & start, std::vector<Joints> & out, std::string * why)
{
  out.clear();
  if (n < 1) {
    if (why) {*why = "n must be >= 1";}
    return false;
  }
  Joints prev = start;
  std::string branch;
  for (int i = 1; i <= n; ++i) {
    const double s = static_cast<double>(i) / n;
    const Point p{from.x + s * (to.x - from.x), from.y + s * (to.y - from.y), from.z + s * (to.z - from.z)};
    const IkResult r = arm.ik_top_down(p);
    if (!r.ok()) {
      if (why) {
        std::ostringstream ss;
        ss << "waypoint " << i << "/" << n << " (" << p.x << ", " << p.y << ", " << p.z << "): " << r.message;
        *why = ss.str();
      }
      return false;
    }
    if (i == 1) {
      branch = r.message;
    } else if (r.message != branch) {
      if (why) {*why = "IK branch changes along the line (" + branch + " -> " + r.message + ")";}
      return false;
    }
    for (int j = 0; j < 4; ++j) {
      if (std::fabs(r.q[j] - prev[j]) > max_step) {
        if (why) {
          std::ostringstream ss;
          ss << "joint " << j << " jumps " << std::fabs(r.q[j] - prev[j]) << " rad at waypoint " << i
             << " (limit " << max_step << ")";
          *why = ss.str();
        }
        return false;
      }
    }
    out.push_back(r.q);
    prev = r.q;
  }
  return true;
}

GraspVerdict verify_grasp(const FingerReading & f, const GraspCheck & c, std::string * why)
{
  std::ostringstream ss;
  ss << "fingers L " << f.left_q << " R " << f.right_q << " m, last target contact L ";
  ss << (f.left_contact_age < 1e8 ? std::to_string(f.left_contact_age) + " s" : std::string("never"));
  ss << " R " << (f.right_contact_age < 1e8 ? std::to_string(f.right_contact_age) + " s" : std::string("never"));
  ss << " ago";
  if (why) {*why = ss.str();}
  if (f.left_q < c.missed_q || f.right_q < c.missed_q) {
    return GraspVerdict::MISSED;
  }
  const bool stalled = f.left_q >= c.q_min && f.left_q <= c.q_max && f.right_q >= c.q_min && f.right_q <= c.q_max;
  const bool touching = !c.require_contact ||
    (f.left_contact_age <= c.contact_window && f.right_contact_age <= c.contact_window);
  return (stalled && touching) ? GraspVerdict::OK : GraspVerdict::UNCERTAIN;
}

const char * to_string(GraspVerdict v)
{
  switch (v) {
    case GraspVerdict::OK: return "OK";
    case GraspVerdict::MISSED: return "MISSED";
    default: return "UNCERTAIN";
  }
}

}  // namespace my_robot_manipulation
