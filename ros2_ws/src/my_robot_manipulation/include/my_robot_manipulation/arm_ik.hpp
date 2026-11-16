// Analytic kinematics for the 4-DOF arm (yaw + 3 pitch joints). SPEC §3.3, §11.1.
//
// Pure C++17, no ROS: the same code runs in unit tests, the ik_cli tool and (Phase 9) the
// pick/place server.
//
// Frame: every position is in arm_base_link (base_link + (0.05, 0, 0.11), axes parallel to
// base_link). Joint order: q[0] yaw, q[1] shoulder, q[2] elbow, q[3] wrist.
//
// Geometry (zero configuration: all links point straight up, +z):
//   shoulder joint at height h_s = 0.10 above arm_base_link
//   upper arm L2 = 0.28, forearm L3 = 0.24, wrist joint -> tool0 L_tool = 0.11
// Pitch convention: a positive pitch rotates a link toward +x (forward). The tool's approach
// axis makes the angle  pitch_total = q1 + q2 + q3  with +z, so pitch_total = pi means the
// gripper points straight down (top-down grasp).
//
// Why a 4-DOF arm needs its own IK: it can place tool0 at any reachable point, but only with
// the tool axis in the vertical plane through the yaw axis, so a general 6-DOF solver (KDL)
// fails most full-pose goals. Given the point and the approach pitch, the solution is closed
// form: yaw from atan2, then a planar 2-link problem for the wrist joint.
#pragma once

#include <array>
#include <string>

namespace my_robot_manipulation
{

using Joints = std::array<double, 4>;

struct Point
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ToolPose
{
  Point position;            ///< tool0 origin in arm_base_link
  double pitch_total{0.0};   ///< q1 + q2 + q3: approach angle from +z in the yaw joint's plane; pi = down
  double yaw{0.0};           ///< q[0]
};

struct JointLimit
{
  double lower;
  double upper;
};

struct ArmParams
{
  double h_s{0.10};      ///< shoulder joint height above arm_base_link
  double L2{0.28};       ///< shoulder -> elbow
  double L3{0.24};       ///< elbow -> wrist
  double L_tool{0.11};   ///< wrist joint -> tool0
  std::array<JointLimit, 4> limits{{{-2.9, 2.9}, {-1.0, 2.0}, {-2.6, 2.6}, {-2.0, 2.0}}};
};

enum class IkStatus
{
  OK,
  OUT_OF_REACH,   ///< wrist point farther than L2 + L3 or closer than |L2 - L3|
  JOINT_LIMIT,    ///< geometrically reachable, but all four solutions violate a limit
};

const char * to_string(IkStatus s);

struct IkResult
{
  IkStatus status{IkStatus::OUT_OF_REACH};
  Joints q{{0.0, 0.0, 0.0, 0.0}};
  bool elbow_up{true};   ///< which of the two elbow solutions was returned
  std::string message;
  bool ok() const {return status == IkStatus::OK;}
};

class ArmKinematics
{
public:
  explicit ArmKinematics(const ArmParams & params = ArmParams());

  /// Forward kinematics: tool0 pose in arm_base_link.
  ToolPose fk(const Joints & q) const;

  /// Unit vector of the tool's approach axis (tool0 +z) in arm_base_link.
  static Point approach_dir(const ToolPose & t);

  /// Position of the wrist joint (start of the tool segment) in arm_base_link.
  Point wrist(const Joints & q) const;

  /// Approach pitch of a tool pose expressed the way ik() expects it (see ik()).
  static double outward_pitch(const ToolPose & t);

  /// Inverse kinematics for tool0 at `p` with approach pitch `pitch_total` (pi = top-down).
  /// `pitch_total` is PHYSICAL: the approach axis's angle from +z, measured in the vertical
  /// plane through the yaw axis and p, positive when the axis tilts outward (away from the
  /// yaw axis). This differs from ToolPose::pitch_total, which is relative to the yaw joint
  /// (the two differ in sign when the arm reaches back over the yaw axis); use outward_pitch()
  /// to convert. Top-down (pi) means the same thing in both.
  /// Candidates, in order: yaw toward the target ("front") with elbow-up (q[2] >= 0), then
  /// elbow-down; then yaw turned by pi ("back", reaching over the yaw axis) with elbow-up, then
  /// elbow-down. Returns the first candidate that respects every joint limit.
  IkResult ik(const Point & p, double pitch_total) const;

  /// Top-down grasp/place IK (pitch_total = pi), the case the pick/place pipeline uses.
  IkResult ik_top_down(const Point & p) const {return ik(p, kPi);}

  bool within_limits(const Joints & q, int * violating_joint = nullptr) const;

  const ArmParams & params() const {return params_;}

  static constexpr double kPi = 3.14159265358979323846;

private:
  ArmParams params_;
};

}  // namespace my_robot_manipulation
