// Unit tests for the analytic arm kinematics (PHASES.md P6).
#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "my_robot_manipulation/arm_ik.hpp"

using namespace my_robot_manipulation;  // NOLINT

namespace
{
constexpr double kPi = ArmKinematics::kPi;
// arm_base_link sits at base_link (0.05, 0, 0.11); base_link is 0.075 above the ground.
constexpr double kArmBaseAboveGround = 0.075 + 0.11;

double dist(const Point & a, const Point & b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

}  // namespace

TEST(ArmFK, ZeroConfigurationPointsStraightUp)
{
  ArmKinematics arm;
  const auto t = arm.fk({{0, 0, 0, 0}});
  // 0.10 (shoulder) + 0.28 + 0.24 + 0.11 = 0.73 above arm_base_link (= base_link z 0.84)
  EXPECT_NEAR(t.position.x, 0.0, 1e-12);
  EXPECT_NEAR(t.position.y, 0.0, 1e-12);
  EXPECT_NEAR(t.position.z, 0.73, 1e-12);
  EXPECT_NEAR(t.pitch_total, 0.0, 1e-12);
}

TEST(ArmFK, ShoulderHorizontalReachesForward)
{
  ArmKinematics arm;
  const auto t = arm.fk({{0, kPi / 2, 0, 0}});
  EXPECT_NEAR(t.position.x, 0.63, 1e-12);   // 0.28 + 0.24 + 0.11
  EXPECT_NEAR(t.position.z, 0.10, 1e-12);
}

TEST(ArmFK, YawRotatesAboutVerticalAxis)
{
  ArmKinematics arm;
  const auto t = arm.fk({{kPi / 2, kPi / 2, 0, 0}});
  EXPECT_NEAR(t.position.x, 0.0, 1e-12);
  EXPECT_NEAR(t.position.y, 0.63, 1e-12);
}

// The Phase-5 gate grasp pose (verified in Gazebo: fingers stalled on the cylinder at 0.025).
TEST(ArmIK, Phase5GraspPoseIsReproduced)
{
  ArmKinematics arm;
  // tool0 at base_link (0.40, 0, 0.050) -> arm_base_link (0.35, 0, -0.06)
  const auto r = arm.ik_top_down({0.35, 0.0, -0.06});
  ASSERT_TRUE(r.ok()) << r.message;
  EXPECT_TRUE(r.elbow_up);
  EXPECT_NEAR(r.q[0], 0.0, 1e-4);
  EXPECT_NEAR(r.q[1], 0.9696, 1e-4);
  EXPECT_NEAR(r.q[2], 1.6527, 1e-4);
  EXPECT_NEAR(r.q[3], 0.5192, 1e-4);
}

// SPEC §11.1 + DC-20 worked examples: grasp with the target at base x 0.40 and the pre-place
// point above a box whose centre is at base x 0.45. Both must solve with margin.
TEST(ArmIK, SpecGraspAndPrePlaceExamplesSolve)
{
  ArmKinematics arm;
  // grasp: cylinder top 0.15 above ground, tool0 at top - 0.025
  const Point grasp{0.40 - 0.05, 0.0, 0.125 - kArmBaseAboveGround};
  // pre-place: object centre 0.105 above ground (floor 0.01 + half height 0.075 + 0.02),
  // tool0 at centre + 0.05, then +0.15 above the place point
  const Point preplace{0.45 - 0.05, 0.0, 0.105 + 0.05 + 0.15 - kArmBaseAboveGround};
  for (const auto & p : {grasp, preplace}) {
    const auto r = arm.ik_top_down(p);
    ASSERT_TRUE(r.ok()) << r.message;
    EXPECT_LT(dist(arm.fk(r.q).position, p), 1e-9);
    // wrist distance from the shoulder must leave margin to full reach (0.52)
    const Point w = arm.wrist(r.q);
    const double d = std::hypot(std::hypot(w.x, w.y), w.z - arm.params().h_s);
    EXPECT_LT(d, 0.50);
  }
}

// 2000 random reachable top-down targets: FK(IK(p)) == p and joints inside limits.
TEST(ArmIK, RandomTopDownRoundTrip)
{
  ArmKinematics arm;
  const auto & L = arm.params().limits;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto sample = [&](int i) {return L[i].lower + (L[i].upper - L[i].lower) * u(rng);};
  int tested = 0;
  int attempts = 0;
  while (tested < 2000 && attempts < 200000) {
    ++attempts;
    Joints q{{sample(0), sample(1), sample(2), 0.0}};
    q[3] = kPi - q[1] - q[2];                      // top-down
    if (!arm.within_limits(q)) {continue;}
    const Point p = arm.fk(q).position;
    const auto r = arm.ik_top_down(p);
    ASSERT_TRUE(r.ok()) << "target from valid joints must be solvable: " << r.message;
    const auto t = arm.fk(r.q);
    ASSERT_LT(dist(t.position, p), 1e-6) << "round trip error";
    // physical approach direction must be straight down, whatever branch IK picked
    ASSERT_LT(dist(ArmKinematics::approach_dir(t), {0.0, 0.0, -1.0}), 1e-9) << r.message;
    ASSERT_TRUE(arm.within_limits(r.q));
    ++tested;
  }
  EXPECT_EQ(tested, 2000);
}

// Same for arbitrary approach pitch (later phases may not always approach top-down).
TEST(ArmIK, RandomGeneralPitchRoundTrip)
{
  ArmKinematics arm;
  const auto & L = arm.params().limits;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  int tested = 0;
  int used_back = 0;
  for (int k = 0; k < 2000; ++k) {
    Joints q;
    for (int i = 0; i < 4; ++i) {q[i] = L[i].lower + (L[i].upper - L[i].lower) * u(rng);}
    const auto t0 = arm.fk(q);
    const auto r = arm.ik(t0.position, ArmKinematics::outward_pitch(t0));
    ASSERT_TRUE(r.ok()) << r.message;
    const auto t1 = arm.fk(r.q);
    ASSERT_LT(dist(t1.position, t0.position), 1e-6);
    // compare the PHYSICAL approach direction, not the pitch number (the back branch mirrors it)
    ASSERT_LT(dist(ArmKinematics::approach_dir(t1), ArmKinematics::approach_dir(t0)), 1e-9) << r.message;
    if (r.message.rfind("back", 0) == 0) {++used_back;}
    ++tested;
  }
  EXPECT_EQ(tested, 2000);
  std::printf("[ info ] general-pitch round trip: %d/2000 solved on the back (yaw + pi) branch\n", used_back);
}

TEST(ArmIK, OutOfReachIsRejected)
{
  ArmKinematics arm;
  // wrist farther than L2 + L3 = 0.52 from the shoulder (top-down: wrist = tool0 + 0.11 up)
  EXPECT_EQ(arm.ik_top_down({0.60, 0.0, 0.0}).status, IkStatus::OUT_OF_REACH);
  EXPECT_EQ(arm.ik_top_down({0.0, 0.0, -0.60}).status, IkStatus::OUT_OF_REACH);
  // wrist closer than |L2 - L3| = 0.04: wrist exactly at the shoulder
  EXPECT_EQ(arm.ik_top_down({0.0, 0.0, 0.10 - 0.11}).status, IkStatus::OUT_OF_REACH);
}

TEST(ArmIK, JointLimitViolationIsRejected)
{
  ArmKinematics arm;
  // Straight below the arm base: geometrically reachable (wrist distance 0.49) but the
  // shoulder would have to point down (~2.83 or ~3.46 rad, limit 2.0) in both elbow solutions.
  const auto r = arm.ik_top_down({0.0, 0.0, -0.50});
  EXPECT_EQ(r.status, IkStatus::JOINT_LIMIT) << r.message;
}

TEST(ArmIK, WithinLimitsReportsTheViolatingJoint)
{
  ArmKinematics arm;
  int j = -1;
  EXPECT_TRUE(arm.within_limits({{0, 0, 0, 0}}));
  EXPECT_FALSE(arm.within_limits({{0, 2.5, 0, 0}}, &j));
  EXPECT_EQ(j, 1);
  EXPECT_FALSE(arm.within_limits({{0, 0, 0, -2.1}}, &j));
  EXPECT_EQ(j, 3);
}

// Reachable top-down workspace on the floor in front of the robot, for the report.
TEST(ArmIK, FloorGraspReachInFront)
{
  ArmKinematics arm;
  // tool0 0.125 above ground (cylinder grasp height), straight ahead: find the x range
  double x_min = 1e9;
  double x_max = -1e9;
  for (double xb = 0.0; xb <= 0.80; xb += 0.005) {
    if (arm.ik_top_down({xb - 0.05, 0.0, 0.125 - kArmBaseAboveGround}).ok()) {
      x_min = std::fmin(x_min, xb);
      x_max = std::fmax(x_max, xb);
    }
  }
  std::printf("[ info ] top-down grasp at cylinder height reachable for base_link x in [%.3f, %.3f]\n",
    x_min, x_max);
  EXPECT_LE(x_min, 0.30);   // the planned grasp at x 0.40 sits inside the range with margin
  EXPECT_GE(x_max, 0.50);
}
