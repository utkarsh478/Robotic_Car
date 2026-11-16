// Unit tests for the pure pick logic (Phase 9): grasp geometry, straight-line IK, verification.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "my_robot_manipulation/grasp_logic.hpp"

using namespace my_robot_manipulation;  // NOLINT

namespace
{
// target standing on the ground at base_link x 0.40: arm_base_link is (0.05, 0, 0.11) in
// base_link and base_link is 0.075 above the ground -> mid-height 0.075 - 0.185 = -0.11
const Point kCentre{0.35, 0.0, 0.075 - 0.185};
}  // namespace

TEST(GraspPoints, TopMinus25mmAndApproachAbove)
{
  const auto g = grasp_points(kCentre, 0.15, 0.025, 0.10);
  EXPECT_NEAR(g.grasp.x, 0.35, 1e-12);
  EXPECT_NEAR(g.grasp.z, -0.11 + 0.075 - 0.025, 1e-12);   // = -0.06, the P5-verified grasp
  EXPECT_NEAR(g.pregrasp.z - g.grasp.z, 0.10, 1e-12);
  EXPECT_NEAR(g.pregrasp.y, g.grasp.y, 1e-12);
}

TEST(LineIk, DescentToTheP5GraspIsSmoothAndExact)
{
  ArmKinematics arm;
  const auto g = grasp_points(kCentre, 0.15, 0.025, 0.10);
  const auto start = arm.ik_top_down(g.pregrasp);
  ASSERT_TRUE(start.ok());
  std::vector<Joints> wps;
  std::string why;
  ASSERT_TRUE(line_ik(arm, g.pregrasp, g.grasp, 20, 0.3, start.q, wps, &why)) << why;
  ASSERT_EQ(wps.size(), 20u);
  // every waypoint is on the vertical line, top-down
  for (size_t i = 0; i < wps.size(); ++i) {
    const auto t = arm.fk(wps[i]);
    EXPECT_NEAR(t.position.x, 0.35, 1e-9);
    EXPECT_NEAR(t.position.y, 0.0, 1e-9);
    EXPECT_NEAR(t.position.z, g.pregrasp.z - 0.10 * (i + 1) / 20.0, 1e-9);
  }
  // last waypoint = the joint values verified in Gazebo in Phase 5
  EXPECT_NEAR(wps.back()[1], 0.9696, 1e-4);
  EXPECT_NEAR(wps.back()[2], 1.6527, 1e-4);
  EXPECT_NEAR(wps.back()[3], 0.5192, 1e-4);
}

TEST(LineIk, LiftWithLateralOffsetWorksAcrossTheMissionBand)
{
  ArmKinematics arm;
  for (double bx : {0.375, 0.40, 0.425}) {
    for (double y : {-0.05, 0.0, 0.05}) {
      const Point c{bx - 0.05, y, -0.11};
      const auto g = grasp_points(c, 0.15, 0.025, 0.10);
      const auto s = arm.ik_top_down(g.grasp);
      ASSERT_TRUE(s.ok());
      std::vector<Joints> wps;
      std::string why;
      EXPECT_TRUE(line_ik(arm, g.grasp, g.pregrasp, 20, 0.3, s.q, wps, &why)) << bx << " " << y << ": " << why;
    }
  }
}

TEST(LineIk, UnreachableWaypointIsReported)
{
  ArmKinematics arm;
  const Point from{0.35, 0.0, -0.06};
  const Point to{0.70, 0.0, -0.06};         // beyond 0.52 m reach
  const auto s = arm.ik_top_down(from);
  std::vector<Joints> wps;
  std::string why;
  EXPECT_FALSE(line_ik(arm, from, to, 20, 0.3, s.q, wps, &why));
  EXPECT_NE(why.find("waypoint"), std::string::npos);
}

TEST(LineIk, JointJumpIsRejected)
{
  ArmKinematics arm;
  const Point from{0.35, 0.0, -0.06};
  const auto s = arm.ik_top_down(from);
  Joints wrong = s.q;
  wrong[0] += 1.0;                           // start state far from the line's first IK solution
  std::vector<Joints> wps;
  std::string why;
  EXPECT_FALSE(line_ik(arm, from, {0.35, 0.0, 0.04}, 20, 0.3, wrong, wps, &why));
  EXPECT_NE(why.find("jumps"), std::string::npos);
}

TEST(VerifyGrasp, StalledWithContactIsOk)
{
  GraspCheck c;
  FingerReading f{0.025, 0.026, 0.05, 0.10};
  std::string why;
  EXPECT_EQ(verify_grasp(f, c, &why), GraspVerdict::OK) << why;
}

TEST(VerifyGrasp, ClosedFingersAreMissed)
{
  GraspCheck c;
  EXPECT_EQ(verify_grasp({0.002, 0.003, 1e9, 1e9}, c, nullptr), GraspVerdict::MISSED);
  EXPECT_EQ(verify_grasp({0.025, 0.004, 0.0, 0.0}, c, nullptr), GraspVerdict::MISSED);  // one side slipped
}

TEST(VerifyGrasp, StalledWithoutContactIsNotOk)
{
  GraspCheck c;
  EXPECT_EQ(verify_grasp({0.025, 0.025, 2.0, 0.1}, c, nullptr), GraspVerdict::UNCERTAIN);
  c.require_contact = false;
  EXPECT_EQ(verify_grasp({0.025, 0.025, 2.0, 0.1}, c, nullptr), GraspVerdict::OK);
}

TEST(VerifyGrasp, WrongWidthIsNotOk)
{
  GraspCheck c;
  EXPECT_EQ(verify_grasp({0.039, 0.039, 0.0, 0.0}, c, nullptr), GraspVerdict::UNCERTAIN);   // still open
  EXPECT_EQ(verify_grasp({0.012, 0.012, 0.0, 0.0}, c, nullptr), GraspVerdict::UNCERTAIN);   // thinner object
}
