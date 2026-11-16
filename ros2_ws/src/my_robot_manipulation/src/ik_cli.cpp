// Command-line front end to ArmKinematics, used by tools/gates/fk_vs_tf.py so the gate checks
// THIS library against the URDF/Gazebo TF tree (not a Python re-implementation).
//
//   ros2 run my_robot_manipulation ik_cli ik  X Y Z [PITCH]   # tool0 in arm_base_link, PITCH default pi
//       -> "OK q0 q1 q2 q3 elbow-up"  or  "OUT_OF_REACH ..." / "JOINT_LIMIT ..."   (exit 0 / 2)
//   ros2 run my_robot_manipulation ik_cli fk  Q0 Q1 Q2 Q3
//       -> "x y z pitch_total"
#include <cstdio>
#include <cstdlib>
#include <string>

#include "my_robot_manipulation/arm_ik.hpp"

using my_robot_manipulation::ArmKinematics;
using my_robot_manipulation::Joints;
using my_robot_manipulation::Point;

static int usage()
{
  std::fprintf(stderr, "usage: ik_cli ik X Y Z [PITCH] | ik_cli fk Q0 Q1 Q2 Q3\n");
  return 1;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {return usage();}
  const std::string mode = argv[1];
  ArmKinematics arm;
  if (mode == "ik" && (argc == 5 || argc == 6)) {
    const Point p{std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4])};
    const double pitch = (argc == 6) ? std::atof(argv[5]) : ArmKinematics::kPi;
    const auto r = arm.ik(p, pitch);
    if (!r.ok()) {
      std::printf("%s %s\n", to_string(r.status), r.message.c_str());
      return 2;
    }
    std::printf("OK %.9f %.9f %.9f %.9f %s\n", r.q[0], r.q[1], r.q[2], r.q[3], r.message.c_str());
    return 0;
  }
  if (mode == "fk" && argc == 6) {
    const Joints q{{std::atof(argv[2]), std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5])}};
    const auto t = arm.fk(q);
    std::printf("%.9f %.9f %.9f %.9f\n", t.position.x, t.position.y, t.position.z, t.pitch_total);
    return 0;
  }
  return usage();
}
