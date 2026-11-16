// pick_place_server: /pick_object action (SPEC §11.2-11.5, §11.7; Phase 9).  /place_object: Phase 10.
//
// Pick, top-down, base stationary:
//   1  goal (odom) -> arm_base_link; grasp tool0 = axis (x, y), top - 0.025 (DC-20); pre-grasp +0.10
//   2  analytic IK: pre-grasp, grasp and every descent waypoint; both end states must keep
//      >= min_clearance from the robot itself and the world (DC-27)          -> UNREACHABLE
//   3  scene: floor + target cylinder; open gripper; MoveIt plan+execute to pre-grasp
//   4  remove target from the scene; straight vertical descent (20 IK waypoints, every state
//      collision-checked, TOTG, MoveGroupInterface::execute)
//   5  close (q 0.0, 1.5 s), wait until both fingers stop (< 0.002 m/s for 0.3 s, max 2 s)
//   6  verify (fingers stalled in [0.015, 0.035] + contact with the target on both fingers in
//      the last 0.5 s)       miss -> open, retract, stow, GRASP_NOT_DETECTED (no attach)
//   7  attach (/gripper/set_attached, Gazebo-confirmed) + attached collision object; lift 0.10
//   8  re-verify                                                             -> OBJECT_LOST
//   9  MoveIt to "transport"; success
// Threads: the action runs in its own thread; sensors and the gripper/attach clients live in
// separate callback groups spun by a MultiThreadedExecutor; MoveGroupInterface spins its own.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include "my_robot_interfaces/action/pick_object.hpp"
#include "my_robot_interfaces/action/place_object.hpp"
#include "my_robot_interfaces/msg/detection.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "my_robot_manipulation/arm_ik.hpp"
#include "my_robot_manipulation/grasp_logic.hpp"

namespace mrm = my_robot_manipulation;
using namespace std::chrono_literals;

namespace
{
using PickObject = my_robot_interfaces::action::PickObject;
using GoalHandlePick = rclcpp_action::ServerGoalHandle<PickObject>;
using PlaceObject = my_robot_interfaces::action::PlaceObject;
using GoalHandlePlace = rclcpp_action::ServerGoalHandle<PlaceObject>;
// The shared error codes have the same values in both actions (UNREACHABLE 1, PLANNING_FAILED 2,
// EXECUTION_FAILED 3, TIMEOUT 6); the helpers below use the PickObject names.
static_assert(PickObject::Result::UNREACHABLE == PlaceObject::Result::UNREACHABLE, "codes differ");
static_assert(PickObject::Result::PLANNING_FAILED == PlaceObject::Result::PLANNING_FAILED, "codes differ");
static_assert(PickObject::Result::EXECUTION_FAILED == PlaceObject::Result::EXECUTION_FAILED, "codes differ");
static_assert(PickObject::Result::TIMEOUT == PlaceObject::Result::TIMEOUT, "codes differ");
using FJT = control_msgs::action::FollowJointTrajectory;

const std::vector<std::string> kArmJoints = {"arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint",
  "arm_wrist_joint"};
const std::vector<std::string> kFingerJoints = {"left_finger_joint", "right_finger_joint"};

/// Outcome of one stage. code uses the PickObject error constants.
struct Step
{
  bool ok{true};
  uint8_t code{PickObject::Result::NONE};
  std::string msg;
  static Step fail(uint8_t c, const std::string & m) {return Step{false, c, m};}
};
}  // namespace

class PickPlaceServer : public rclcpp::Node
{
public:
  PickPlaceServer()
  : Node("pick_place_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Parameters (defaults here; manipulation_params.yaml overrides). Declared only if not
    // already declared from overrides (robot_description etc. come in as overrides too).
    p_planning_time_ = param("planning_time", 2.0);
    p_attempts_ = param("planning_attempts", 2);
    p_free_vel_ = param("free_velocity_scaling", 0.5);
    p_free_acc_ = param("free_acceleration_scaling", 0.5);
    p_line_vel_ = param("line_velocity_scaling", 0.2);
    p_line_acc_ = param("line_acceleration_scaling", 0.2);
    p_waypoints_ = param("line_waypoints", 20);
    p_max_step_ = param("line_max_joint_step", 0.3);
    p_below_top_ = param("grasp_below_top", 0.025);
    p_approach_ = param("pregrasp_height", 0.10);
    p_lift_ = param("lift_height", 0.10);
    p_min_clear_ = param("min_clearance", 0.01);
    p_open_q_ = param("gripper_open", 0.04);
    p_closed_q_ = param("gripper_closed", 0.0);
    p_open_time_ = param("gripper_open_time", 1.5);
    p_close_time_ = param("gripper_close_time", 1.5);
    p_settle_vel_ = param("finger_settle_velocity", 0.002);
    p_settle_time_ = param("finger_settle_time", 0.3);
    p_settle_timeout_ = param("finger_settle_timeout", 2.0);
    check_.q_min = param("grasp_q_min", 0.015);
    check_.q_max = param("grasp_q_max", 0.035);
    check_.missed_q = param("missed_q", 0.010);
    check_.contact_window = param("contact_window", 0.5);
    check_.require_contact = param("require_contact", true);
    p_reverify_window_ = param("reverify_contact_window", 1.0);
    p_reverify_contact_ = param("reverify_require_contact", false);
    p_hold_squeeze_ = param("hold_squeeze", 0.002);
    p_hold_tol_ = param("hold_tolerance", 0.004);
    p_target_model_ = param("target_model", std::string("target_cylinder"));
    p_arm_base_ = param("arm_base_frame", std::string("arm_base_link"));
    p_timeout_ = param("pick_timeout", 90.0);
    p_floor_z_ = param("floor_z", -0.012);
    // place (SPEC 11.6, DC-38)
    p_place_clear_ = param("place_floor_clearance", 0.02);
    p_preplace_ = param("preplace_height", 0.15);
    p_retract_ = param("retract_height", 0.15);
    p_box_size_ = param("box_outer_size", 0.30);
    p_box_h_ = param("box_height", 0.15);
    p_box_wall_ = param("box_wall", 0.01);
    p_released_q_ = param("released_q_min", 0.035);
    p_no_contact_ = param("release_no_contact_time", 0.5);
    p_check_percept_ = param("release_check_perception", true);
    p_det_topic_ = param("release_detection_topic", std::string("/perception/target_raw"));
    p_det_reset_ = param("release_detector_reset", std::string("/target_detector/reset"));
    p_det_radius_ = param("release_max_offset", 0.10);
    p_det_needed_ = param("release_detections", 3);
    p_det_timeout_ = param("release_detection_timeout", 5.0);
    p_place_timeout_ = param("place_timeout", 90.0);

    cb_action_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_sensors_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_clients_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions so;
    so.callback_group = cb_sensors_;
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {onJointState(*m);}, so);
    left_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/gripper/left_contact", 20, [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr m) {onContact(*m, 0);}, so);
    right_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "/gripper/right_contact", 20, [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr m) {onContact(*m, 1);}, so);
    // Gazebo-confirmed attach state from grasp_attach_manager (latched)
    attached_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/gripper/attached", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {attached_ = m->data;}, so);

    gripper_ = rclcpp_action::create_client<FJT>(this, "/gripper_controller/follow_joint_trajectory", cb_clients_);
    attach_ = create_client<std_srvs::srv::SetBool>("/gripper/set_attached", rmw_qos_profile_services_default,
        cb_clients_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    pick_server_ = rclcpp_action::create_server<PickObject>(
      this, "/pick_object",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PickObject::Goal>) {
        if (busy_.load()) {
          RCLCPP_WARN(get_logger(), "pick rejected: another goal is running");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandlePick>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandlePick> gh) {
        busy_ = true;
        std::thread([this, gh]() {executePick(gh); busy_ = false;}).detach();
      },
      rcl_action_server_get_default_options(), cb_action_);

    place_server_ = rclcpp_action::create_server<PlaceObject>(
      this, "/place_object",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PlaceObject::Goal>) {
        if (busy_.load()) {
          RCLCPP_WARN(get_logger(), "place rejected: another goal is running");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandlePlace>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<GoalHandlePlace> gh) {
        busy_ = true;
        std::thread([this, gh]() {executePlace(gh); busy_ = false;}).detach();
      },
      rcl_action_server_get_default_options(), cb_action_);

    // release verification by perception (SPEC 11.6 step 4)
    det_sub_ = create_subscription<my_robot_interfaces::msg::Detection>(
      p_det_topic_, 20, [this](my_robot_interfaces::msg::Detection::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        dets_.push_back(*m);
        if (dets_.size() > 50) {dets_.erase(dets_.begin());}
      }, so);
    det_reset_ = create_client<std_srvs::srv::Trigger>(p_det_reset_, rmw_qos_profile_services_default, cb_clients_);
  }

  /// MoveIt objects need shared_from_this(), so they are created after construction.
  void init()
  {
    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
    arm_->setPlanningTime(p_planning_time_);
    arm_->setNumPlanningAttempts(static_cast<unsigned int>(p_attempts_));
    arm_->setMaxVelocityScalingFactor(p_free_vel_);
    arm_->setMaxAccelerationScalingFactor(p_free_acc_);
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
    if (!psm_->getPlanningScene()) {
      throw std::runtime_error("PlanningSceneMonitor: no planning scene (robot_description missing?)");
    }
    psm_->requestPlanningSceneState("/get_planning_scene");
    psm_->startSceneMonitor("/monitored_planning_scene");
    psm_->startStateMonitor("/joint_states");
    for (const char * l : {"shoulder_link", "upper_arm_link", "forearm_link", "wrist_link", "gripper_base_link",
        "left_finger_link", "right_finger_link"})
    {
      const moveit::core::LinkModel * lm = psm_->getRobotModel()->getLinkModel(l);
      if (!lm) {throw std::runtime_error(std::string("robot model has no link ") + l);}
      arm_links_.insert(lm);
    }
    scene_iface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    addFloor();
    RCLCPP_INFO(get_logger(), "pick_place_server ready: /pick_object (planning frame %s)",
      arm_->getPlanningFrame().c_str());
  }

private:
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, def);
    }
    return get_parameter(name).get_value<T>();
  }

  // ------------------------------------------------------------------ sensors
  void onJointState(const sensor_msgs::msg::JointState & m)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < m.name.size(); ++i) {
      if (i < m.position.size()) {pos_[m.name[i]] = m.position[i];}
      if (i < m.velocity.size()) {vel_[m.name[i]] = m.velocity[i];}
    }
  }

  void onContact(const ros_gz_interfaces::msg::Contacts & m, int side)
  {
    for (const auto & c : m.contacts) {
      if (c.collision1.name.find(p_target_model_) != std::string::npos ||
        c.collision2.name.find(p_target_model_) != std::string::npos)
      {
        std::lock_guard<std::mutex> lk(mtx_);
        last_contact_[side] = now().seconds();
        return;
      }
    }
  }

  mrm::FingerReading fingers()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mrm::FingerReading f;
    f.left_q = pos_.count(kFingerJoints[0]) ? pos_[kFingerJoints[0]] : -1.0;
    f.right_q = pos_.count(kFingerJoints[1]) ? pos_[kFingerJoints[1]] : -1.0;
    const double t = now().seconds();
    f.left_contact_age = last_contact_[0] > 0 ? t - last_contact_[0] : 1e9;
    f.right_contact_age = last_contact_[1] > 0 ? t - last_contact_[1] : 1e9;
    return f;
  }

  bool armJoints(mrm::Joints & q)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    for (int i = 0; i < 4; ++i) {
      auto it = pos_.find(kArmJoints[i]);
      if (it == pos_.end()) {return false;}
      q[i] = it->second;
    }
    return true;
  }

  /// wait (sim time) without spinning: the executor threads keep the subscriptions alive
  void sleepSim(double s)
  {
    const double t0 = now().seconds();
    const auto w0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() && now().seconds() - t0 < s &&
      std::chrono::steady_clock::now() - w0 < std::chrono::duration<double>(4.0 * s + 5.0))
    {
      std::this_thread::sleep_for(10ms);
    }
  }

  // ------------------------------------------------------------------ scene
  void addFloor()
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = arm_->getPlanningFrame();
    co.id = "floor";
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {3.0, 3.0, 0.02};
    geometry_msgs::msg::Pose p;
    p.position.z = p_floor_z_;   // top 2 mm below the ground (DC-24)
    p.orientation.w = 1.0;
    co.primitives.push_back(box);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;
    scene_iface_->applyCollisionObject(co);
  }

  void addTarget(const geometry_msgs::msg::PoseStamped & pose, double d, double h)
  {
    moveit_msgs::msg::CollisionObject co;
    co.header = pose.header;
    co.id = "target";
    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = {h, d / 2.0};
    geometry_msgs::msg::Pose p;
    p.position = pose.pose.position;
    p.orientation.w = 1.0;
    co.primitives.push_back(cyl);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;
    scene_iface_->applyCollisionObject(co);
    waitScene(true, false);
  }

  /// Wait until OUR monitored scene shows the change (the scene services return when move_group
  /// has applied it; our PlanningSceneMonitor hears about it a moment later). DC-36.
  bool waitScene(bool want_world, bool want_attached, double timeout = 3.0)
  {
    const auto w0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - w0 < std::chrono::duration<double>(timeout)) {
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const bool in_world = scene->getWorld()->hasObject("target");
        const bool attached = scene->getCurrentState().hasAttachedBody("target");
        if (in_world == want_world && attached == want_attached) {return true;}
      }
      std::this_thread::sleep_for(20ms);
    }
    RCLCPP_WARN(get_logger(), "planning scene did not reach (world target %d, attached target %d) in %.1f s",
      want_world, want_attached, timeout);
    return false;
  }

  /// Remove the world object "target" synchronously (service, not the fire-and-forget topic).
  void removeTarget()
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = arm_->getPlanningFrame();
    co.id = "target";
    co.operation = co.REMOVE;
    scene_iface_->applyCollisionObject(co);
    waitScene(false, false);
  }

  void attachTargetInScene(double d, double h)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "gripper_base_link";
    aco.object.header.frame_id = "tool0";
    aco.object.id = "target";
    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = {h, d / 2.0};
    geometry_msgs::msg::Pose p;
    p.position.z = h / 2.0 - p_below_top_;   // tool0 +z points down along the object axis
    p.orientation.w = 1.0;
    aco.object.primitives.push_back(cyl);
    aco.object.primitive_poses.push_back(p);
    aco.object.operation = aco.object.ADD;
    aco.touch_links = {"gripper_base_link", "left_finger_link", "right_finger_link", "wrist_link"};
    scene_iface_->applyAttachedCollisionObject(aco);
    waitScene(false, true);
  }

  /// Remove "target" wherever it is (still attached from an earlier pick, or in the world),
  /// synchronously. Detaching in MoveIt puts the object back into the world, so detach first,
  /// then remove the world object, then wait until our own monitored scene shows neither (DC-36).
  void clearTarget()
  {
    bool attached = false;
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
      attached = scene->getCurrentState().hasAttachedBody("target");
    }
    if (attached) {
      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.link_name = "gripper_base_link";
      aco.object.id = "target";
      aco.object.operation = aco.object.REMOVE;
      scene_iface_->applyAttachedCollisionObject(aco);
    }
    removeTarget();
  }

  /// Joint state -> RobotState from the monitored scene, with arm (and optionally finger) values.
  moveit::core::RobotState stateWith(const mrm::Joints & q, double fingers_q = -1.0)
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    moveit::core::RobotState st = scene->getCurrentState();
    st.setJointGroupPositions("arm", std::vector<double>(q.begin(), q.end()));
    if (fingers_q >= 0.0) {
      st.setVariablePosition(kFingerJoints[0], fingers_q);
      st.setVariablePosition(kFingerJoints[1], fingers_q);
    }
    st.update();
    return st;
  }

  /// DC-27: smallest distance from the ARM and GRIPPER links to anything (the robot's other
  /// links, using the SRDF ACM, and world objects). Only the moving links count: the wheels and
  /// casters rest 2 mm above the floor box (DC-24) and would always report 0.002 m (DC-34).
  double clearance(const mrm::Joints & q, double fingers_q, std::string & pair)
  {
    moveit::core::RobotState st = stateWith(q, fingers_q);
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    collision_detection::DistanceRequest req;
    req.type = collision_detection::DistanceRequestType::GLOBAL;
    req.enable_signed_distance = true;
    req.acm = &scene->getAllowedCollisionMatrix();
    req.active_components_only = &arm_links_;
    collision_detection::DistanceResult self_res, world_res;
    scene->getCollisionEnv()->distanceSelf(req, self_res, st);
    scene->getCollisionEnv()->distanceRobot(req, world_res, st);
    const auto & m = self_res.minimum_distance.distance <= world_res.minimum_distance.distance ?
      self_res.minimum_distance : world_res.minimum_distance;
    pair = (m.link_names[0].empty() ? std::string("world") : m.link_names[0]) + " - " +
      (m.link_names[1].empty() ? std::string("world") : m.link_names[1]);
    return m.distance;
  }

  bool colliding(const mrm::Joints & q, std::string & what)
  {
    moveit::core::RobotState st = stateWith(q);
    planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 1;
    collision_detection::CollisionResult res;
    scene->checkCollision(req, res, st);
    if (res.collision && !res.contacts.empty()) {
      what = res.contacts.begin()->first.first + " - " + res.contacts.begin()->first.second;
    }
    return res.collision;
  }

  // ------------------------------------------------------------------ motion
  Step moveJoints(const mrm::Joints & q, const std::string & label)
  {
    arm_->setStartStateToCurrentState();
    arm_->setMaxVelocityScalingFactor(p_free_vel_);
    arm_->setMaxAccelerationScalingFactor(p_free_acc_);
    arm_->setJointValueTarget(std::vector<double>(q.begin(), q.end()));
    return planAndExecute(label);
  }

  Step moveNamed(const std::string & name)
  {
    arm_->setStartStateToCurrentState();
    arm_->setMaxVelocityScalingFactor(p_free_vel_);
    arm_->setMaxAccelerationScalingFactor(p_free_acc_);
    if (!arm_->setNamedTarget(name)) {
      return Step::fail(PickObject::Result::PLANNING_FAILED, "unknown named state " + name);
    }
    return planAndExecute(name);
  }

  Step planAndExecute(const std::string & label)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto t0 = now().seconds();
    if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return Step::fail(PickObject::Result::PLANNING_FAILED, "MoveIt could not plan to " + label);
    }
    if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "execution to " + label + " failed");
    }
    RCLCPP_INFO(get_logger(), "  %s: planned+executed in %.1f s", label.c_str(), now().seconds() - t0);
    return Step{};
  }

  /// Straight vertical tool0 line from the CURRENT pose to `to` (arm_base_link), SPEC §11.3.
  Step moveLine(const mrm::Point & to, const std::string & label)
  {
    mrm::Joints q0;
    if (!armJoints(q0)) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "no joint states");
    }
    const mrm::Point from = kin_.fk(q0).position;
    std::vector<mrm::Joints> wps;
    std::string why;
    if (!mrm::line_ik(kin_, from, to, p_waypoints_, p_max_step_, q0, wps, &why)) {
      return Step::fail(PickObject::Result::UNREACHABLE, label + ": " + why);
    }
    for (size_t i = 0; i < wps.size(); ++i) {
      std::string what;
      if (colliding(wps[i], what)) {
        return Step::fail(PickObject::Result::PLANNING_FAILED,
                 label + ": waypoint " + std::to_string(i + 1) + " in collision (" + what + ")");
      }
    }
    robot_trajectory::RobotTrajectory traj(psm_->getRobotModel(), "arm");
    traj.addSuffixWayPoint(stateWith(q0), 0.0);
    for (const auto & q : wps) {
      traj.addSuffixWayPoint(stateWith(q), 0.1);
    }
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(traj, p_line_vel_, p_line_acc_)) {
      return Step::fail(PickObject::Result::PLANNING_FAILED, label + ": time parameterisation failed");
    }
    moveit_msgs::msg::RobotTrajectory msg;
    traj.getRobotTrajectoryMsg(msg);
    const auto t0 = now().seconds();
    if (arm_->execute(msg) != moveit::core::MoveItErrorCode::SUCCESS) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, label + ": execution failed");
    }
    lines_checked_ += static_cast<int>(wps.size());
    RCLCPP_INFO(get_logger(), "  %s: %zu collision-free waypoints, %.1f s", label.c_str(), wps.size(),
      now().seconds() - t0);
    return Step{};
  }

  /// Command both fingers to q. wait_settle: block until both stop (stall or target reached).
  Step gripperTo(double q, double duration, bool wait_settle)
  {
    if (!gripper_->wait_for_action_server(2s)) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "gripper action server not available");
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = kFingerJoints;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {q, q};
    pt.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(pt);
    auto fut = gripper_->async_send_goal(goal);
    if (fut.wait_for(5s) != std::future_status::ready || !fut.get()) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "gripper goal not accepted");
    }
    sleepSim(duration);
    if (!wait_settle) {
      return Step{};
    }
    // both finger speeds below the threshold for settle_time (stalled on the object, or arrived)
    const double t0 = now().seconds();
    double still_since = -1.0;
    while (now().seconds() - t0 < p_settle_timeout_) {
      double vl = 1.0, vr = 1.0;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        if (vel_.count(kFingerJoints[0])) {vl = std::fabs(vel_[kFingerJoints[0]]);}
        if (vel_.count(kFingerJoints[1])) {vr = std::fabs(vel_[kFingerJoints[1]]);}
      }
      const double t = now().seconds();
      if (vl < p_settle_vel_ && vr < p_settle_vel_) {
        if (still_since < 0) {still_since = t;}
        if (t - still_since >= p_settle_time_) {return Step{};}
      } else {
        still_since = -1.0;
      }
      std::this_thread::sleep_for(10ms);
    }
    return Step{};   // not settled within the timeout: verification decides
  }

  /// Hold each finger where it is (after the attach, DC-35): commanding 0.0 would sweep the
  /// fingers through the welded object, which no longer collides with the robot.
  Step gripperHold(double ql, double qr)
  {
    if (!gripper_->wait_for_action_server(2s)) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "gripper action server not available");
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = kFingerJoints;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {ql, qr};
    pt.time_from_start = rclcpp::Duration::from_seconds(0.2);
    goal.trajectory.points.push_back(pt);
    auto fut = gripper_->async_send_goal(goal);
    if (fut.wait_for(5s) != std::future_status::ready || !fut.get()) {
      return Step::fail(PickObject::Result::EXECUTION_FAILED, "gripper hold goal not accepted");
    }
    sleepSim(0.3);
    return Step{};
  }

  bool setAttached(bool on, std::string & why)
  {
    if (!attach_->wait_for_service(2s)) {
      why = "/gripper/set_attached not available";
      return false;
    }
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = on;
    auto fut = attach_->async_send_request(req);
    if (fut.wait_for(6s) != std::future_status::ready) {
      why = "/gripper/set_attached timed out";
      return false;
    }
    auto res = fut.get();
    why = res->message;
    return res->success;
  }

  // ------------------------------------------------------------------ pick
  void executePick(const std::shared_ptr<GoalHandlePick> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<PickObject::Result>();
    auto feedback = std::make_shared<PickObject::Feedback>();
    const double t_start = now().seconds();
    lines_checked_ = 0;
    auto stage = [&](const std::string & s) {
        feedback->stage = s;
        gh->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "pick: %s", s.c_str());
      };
    auto finish = [&](const Step & st) {
        result->success = st.ok;
        result->error_code = st.code;
        result->message = st.msg;
        RCLCPP_INFO(get_logger(), "pick %s after %.1f s: %s (%d line waypoints collision-checked)",
          st.ok ? "SUCCEEDED" : "FAILED", now().seconds() - t_start, st.msg.c_str(), lines_checked_);
        if (st.ok) {gh->succeed(result);} else {gh->abort(result);}
      };
    auto out_of_time = [&]() {return now().seconds() - t_start > p_timeout_;};
    auto canceled = [&]() {
        if (gh->is_canceling()) {
          result->success = false;
          result->message = "canceled";
          gh->canceled(result);
          return true;
        }
        return false;
      };

    const double d = goal->object_diameter > 0 ? goal->object_diameter : 0.05;
    const double h = goal->object_height > 0 ? goal->object_height : 0.15;

    // 1. goal -> arm_base_link
    geometry_msgs::msg::PointStamped in, c;
    in.header = goal->object_pose.header;
    in.header.stamp = rclcpp::Time(0);   // latest: the base is stationary during the pick
    in.point = goal->object_pose.pose.position;
    try {
      c = tf_buffer_->transform(in, p_arm_base_, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      return finish(Step::fail(PickObject::Result::UNREACHABLE, std::string("TF: ") + e.what()));
    }
    const mrm::GraspPoints gp = mrm::grasp_points({c.point.x, c.point.y, c.point.z}, h, p_below_top_, p_approach_);
    RCLCPP_INFO(get_logger(), "pick: object at arm_base (%.3f, %.3f, %.3f), grasp tool0 z %.3f",
      c.point.x, c.point.y, c.point.z, gp.grasp.z);

    // 2. IK + clearance (before the target is in the scene: the fingers will surround it)
    clearTarget();
    const mrm::IkResult ik_pre = kin_.ik_top_down(gp.pregrasp);
    const mrm::IkResult ik_grasp = kin_.ik_top_down(gp.grasp);
    if (!ik_pre.ok() || !ik_grasp.ok()) {
      return finish(Step::fail(PickObject::Result::UNREACHABLE, std::string("IK: pre-grasp ") +
        ik_pre.message + ", grasp " + ik_grasp.message));
    }
    {
      std::vector<mrm::Joints> wps;
      std::string why;
      if (!mrm::line_ik(kin_, gp.pregrasp, gp.grasp, p_waypoints_, p_max_step_, ik_pre.q, wps, &why)) {
        return finish(Step::fail(PickObject::Result::UNREACHABLE, "descent: " + why));
      }
    }
    for (const auto & [label, q] : {std::make_pair(std::string("pre-grasp"), ik_pre.q),
        std::make_pair(std::string("grasp"), ik_grasp.q)})
    {
      std::string pair;
      const double cl = clearance(q, p_open_q_, pair);
      RCLCPP_INFO(get_logger(), "pick: %s clearance %.3f m (%s)", label.c_str(), cl, pair.c_str());
      if (cl < p_min_clear_) {
        std::ostringstream ss;
        ss << label << " clearance " << cl << " m (" << pair << ") < " << p_min_clear_ << " m (DC-27)";
        return finish(Step::fail(PickObject::Result::UNREACHABLE, ss.str()));
      }
    }

    // 3. scene + open + pre-grasp
    addTarget(goal->object_pose, d, h);
    stage("open");
    Step st = gripperTo(p_open_q_, p_open_time_, true);
    if (!st.ok) {return finish(st);}
    if (canceled()) {return;}
    stage("pregrasp");
    st = moveJoints(ik_pre.q, "pre-grasp");
    if (!st.ok) {return finish(st);}
    if (canceled()) {return;}

    // 4. descend
    removeTarget();
    stage("descend");
    st = moveLine(gp.grasp, "descend");
    if (!st.ok) {return finish(st);}

    // 5. close
    stage("close");
    st = gripperTo(p_closed_q_, p_close_time_, true);
    if (!st.ok) {return finish(st);}

    // 6. verify
    stage("verify");
    std::string why;
    const mrm::GraspVerdict v = mrm::verify_grasp(fingers(), check_, &why);
    RCLCPP_INFO(get_logger(), "pick: grasp %s: %s", mrm::to_string(v), why.c_str());
    if (v != mrm::GraspVerdict::OK) {
      gripperTo(p_open_q_, p_open_time_, true);
      moveLine(gp.pregrasp, "retract");
      moveNamed("stow");
      return finish(Step::fail(PickObject::Result::GRASP_NOT_DETECTED,
        std::string("grasp ") + mrm::to_string(v) + ": " + why));
    }

    // 7. hold, attach, lift. The fingers are still commanded to 0.0; the moment the weld exists
    // they would sweep through the (no longer colliding) object, so FIRST hold them 2 mm inside
    // the stall width (a light squeeze while the object still collides), THEN attach (DC-37).
    const mrm::FingerReading f_grip = fingers();
    const double hold_l = std::max(0.0, f_grip.left_q - p_hold_squeeze_);
    const double hold_r = std::max(0.0, f_grip.right_q - p_hold_squeeze_);
    st = gripperHold(hold_l, hold_r);
    if (!st.ok) {return finish(st);}
    std::string att;
    if (!setAttached(true, att)) {
      gripperTo(p_open_q_, p_open_time_, true);
      moveLine(gp.pregrasp, "retract");
      moveNamed("stow");
      return finish(Step::fail(PickObject::Result::EXECUTION_FAILED, "attach failed: " + att));
    }
    attachTargetInScene(d, h);
    stage("lift");
    st = moveLine({gp.grasp.x, gp.grasp.y, gp.grasp.z + p_lift_}, "lift");
    if (!st.ok) {return finish(st);}

    // 8. re-verify. In simulation the attach welds the target to the wrist; the robot model has
    // no self-collision, so the fingers stop touching it and the contact sensors go quiet (P9
    // trace, DC-35). Evidence here = fingers still at the grasp width + Gazebo confirms the weld.
    // On hardware set reverify_require_contact: true (contact / motor current stays meaningful).
    mrm::GraspCheck re = check_;
    re.contact_window = p_reverify_window_;
    re.require_contact = p_reverify_contact_;
    const mrm::FingerReading f_now = fingers();
    mrm::GraspVerdict v2 = mrm::verify_grasp(f_now, re, &why);
    // in simulation the fingers stay where they were held: compare with the hold command, not
    // with the stall window (the hold is 2 mm inside it)
    const bool at_hold = std::fabs(f_now.left_q - hold_l) < p_hold_tol_ && std::fabs(f_now.right_q - hold_r) < p_hold_tol_;
    if (!p_reverify_contact_ && v2 == mrm::GraspVerdict::UNCERTAIN && at_hold) {v2 = mrm::GraspVerdict::OK;}
    if (v2 == mrm::GraspVerdict::OK && !p_reverify_contact_ && !at_hold) {v2 = mrm::GraspVerdict::UNCERTAIN;}
    const bool welded = attached_.load();
    if (v2 == mrm::GraspVerdict::OK && !welded) {v2 = mrm::GraspVerdict::UNCERTAIN;}
    {
      std::ostringstream ss;
      ss << ", held at L " << hold_l << " R " << hold_r << (at_hold ? " (in place)" : " (MOVED)")
         << ", Gazebo attach " << (welded ? "confirmed" : "NOT confirmed");
      why += ss.str();
    }
    RCLCPP_INFO(get_logger(), "pick: after lift %s: %s", mrm::to_string(v2), why.c_str());
    if (v2 != mrm::GraspVerdict::OK) {
      std::string tmp;
      setAttached(false, tmp);
      clearTarget();
      gripperTo(p_open_q_, p_open_time_, true);
      moveNamed("stow");
      return finish(Step::fail(PickObject::Result::OBJECT_LOST, std::string("after lift: ") + why));
    }
    if (out_of_time()) {
      return finish(Step::fail(PickObject::Result::TIMEOUT, "pick took longer than pick_timeout"));
    }

    // 9. transport pose
    stage("transport");
    st = moveNamed("transport");
    if (!st.ok) {return finish(st);}
    finish(Step{true, PickObject::Result::NONE, "picked: " + why});
  }

  // ------------------------------------------------------------------ place
  /// Box walls + floor (SPEC 11.2) around the place point; `floor_top` = z of the inside floor.
  void addBox(const std_msgs::msg::Header & hdr, double cx, double cy, double floor_top, double yaw)
  {
    moveit_msgs::msg::CollisionObject co;
    co.header = hdr;
    co.id = "box";
    const double o = p_box_size_, h = p_box_h_, t = p_box_wall_;
    const double z0 = floor_top - t;                 // bottom of the box
    const double c = std::cos(yaw), sn = std::sin(yaw);
    auto part = [&](double dx, double dy, double sx, double sy, double sz, double zc) {
        shape_msgs::msg::SolidPrimitive b;
        b.type = shape_msgs::msg::SolidPrimitive::BOX;
        b.dimensions = {sx, sy, sz};
        geometry_msgs::msg::Pose p;
        p.position.x = cx + c * dx - sn * dy;
        p.position.y = cy + sn * dx + c * dy;
        p.position.z = zc;
        p.orientation.z = std::sin(yaw / 2);
        p.orientation.w = std::cos(yaw / 2);
        co.primitives.push_back(b);
        co.primitive_poses.push_back(p);
      };
    part(0, 0, o, o, t, z0 + t / 2);                                 // floor
    part(o / 2 - t / 2, 0, t, o, h, z0 + h / 2);                     // +x wall
    part(-(o / 2 - t / 2), 0, t, o, h, z0 + h / 2);                  // -x wall
    part(0, o / 2 - t / 2, o - 2 * t, t, h, z0 + h / 2);             // +y wall
    part(0, -(o / 2 - t / 2), o - 2 * t, t, h, z0 + h / 2);          // -y wall
    co.operation = co.ADD;
    scene_iface_->applyCollisionObject(co);
    const auto w0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - w0 < 3s) {
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        if (scene->getWorld()->hasObject("box")) {return;}
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  void removeBox()
  {
    moveit_msgs::msg::CollisionObject co;
    co.header.frame_id = arm_->getPlanningFrame();
    co.id = "box";
    co.operation = co.REMOVE;
    scene_iface_->applyCollisionObject(co);
    const auto w0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - w0 < 3s) {
      {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        if (!scene->getWorld()->hasObject("box")) {return;}
      }
      std::this_thread::sleep_for(20ms);
    }
  }

  void executePlace(const std::shared_ptr<GoalHandlePlace> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<PlaceObject::Result>();
    auto feedback = std::make_shared<PlaceObject::Feedback>();
    const double t_start = now().seconds();
    lines_checked_ = 0;
    bool box_added = false;
    auto stage = [&](const std::string & s) {
        feedback->stage = s;
        gh->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "place: %s", s.c_str());
      };
    auto finish = [&](const Step & st) {
        if (box_added) {removeBox();}
        result->success = st.ok;
        result->error_code = st.code;
        result->message = st.msg;
        RCLCPP_INFO(get_logger(), "place %s after %.1f s: %s (%d line waypoints collision-checked)",
          st.ok ? "SUCCEEDED" : "FAILED", now().seconds() - t_start, st.msg.c_str(), lines_checked_);
        if (st.ok) {gh->succeed(result);} else {gh->abort(result);}
      };
    if (!attached_.load()) {
      return finish(Step::fail(PlaceObject::Result::EXECUTION_FAILED, "not holding an object (Gazebo attach is off)"));
    }
    const double h = 0.15;     // object height (the pick's; PlaceObject carries no size)

    // 1. goal (object centre when released) -> arm_base_link; box yaw from the goal orientation
    geometry_msgs::msg::PointStamped in, c;
    in.header = goal->place_pose.header;
    in.header.stamp = rclcpp::Time(0);
    in.point = goal->place_pose.pose.position;
    try {
      c = tf_buffer_->transform(in, p_arm_base_, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      return finish(Step::fail(PlaceObject::Result::UNREACHABLE, std::string("TF: ") + e.what()));
    }
    const auto & q = goal->place_pose.pose.orientation;
    const double box_yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    // tool0 = object centre + (h/2 - below_top) (same grasp as the pick, DC-20)
    const mrm::Point place{c.point.x, c.point.y, c.point.z + h / 2 - p_below_top_};
    const mrm::Point pre{place.x, place.y, place.z + p_preplace_};
    const double floor_top = goal->place_pose.pose.position.z - h / 2 - p_place_clear_;
    RCLCPP_INFO(get_logger(), "place: object centre at arm_base (%.3f, %.3f, %.3f), box floor z %.3f (goal frame)",
      c.point.x, c.point.y, c.point.z, floor_top);

    // 2. IK, box in the scene, clearance
    const mrm::IkResult ik_pre = kin_.ik_top_down(pre);
    const mrm::IkResult ik_place = kin_.ik_top_down(place);
    if (!ik_pre.ok() || !ik_place.ok()) {
      return finish(Step::fail(PlaceObject::Result::UNREACHABLE, std::string("IK: pre-place ") + ik_pre.message +
        ", place " + ik_place.message));
    }
    {
      std::vector<mrm::Joints> wps;
      std::string why;
      if (!mrm::line_ik(kin_, pre, place, p_waypoints_, p_max_step_, ik_pre.q, wps, &why)) {
        return finish(Step::fail(PlaceObject::Result::UNREACHABLE, "descent: " + why));
      }
    }
    addBox(goal->place_pose.header, goal->place_pose.pose.position.x, goal->place_pose.pose.position.y, floor_top,
      box_yaw);
    box_added = true;
    const mrm::FingerReading f0 = fingers();
    for (const auto & [label, qq] : {std::make_pair(std::string("pre-place"), ik_pre.q),
        std::make_pair(std::string("place"), ik_place.q)})
    {
      std::string pair;
      const double cl = clearance(qq, std::min(f0.left_q, f0.right_q), pair);
      RCLCPP_INFO(get_logger(), "place: %s clearance %.3f m (%s)", label.c_str(), cl, pair.c_str());
      if (cl < p_min_clear_) {
        std::ostringstream ss;
        ss << label << " clearance " << cl << " m (" << pair << ") < " << p_min_clear_ << " m (DC-27)";
        return finish(Step::fail(PlaceObject::Result::UNREACHABLE, ss.str()));
      }
    }

    // 3. pre-place (MoveIt, avoids the box with the held object)
    stage("preplace");
    Step st = moveJoints(ik_pre.q, "pre-place");
    if (!st.ok) {return finish(st);}
    if (gh->is_canceling()) {
      result->message = "canceled";
      removeBox();
      gh->canceled(result);
      return;
    }

    // 4. straight down
    stage("descend");
    st = moveLine(place, "place descend");
    if (!st.ok) {moveNamed("transport"); return finish(st);}

    // 5. release: OPEN FIRST, then detach (DC-38). While welded the fingers pass through the
    //    object; detaching with the fingers still 2 mm inside it would make Gazebo push it out.
    stage("release");
    st = gripperTo(p_open_q_, p_open_time_, true);
    if (!st.ok) {return finish(st);}
    std::string att;
    if (!setAttached(false, att)) {
      return finish(Step::fail(PlaceObject::Result::EXECUTION_FAILED, "detach failed: " + att));
    }
    clearTarget();      // MoveIt: detach + remove (the object now stands in the box)

    // 6. verify the gripper let go
    stage("verify");
    sleepSim(p_no_contact_);
    std::string why;
    {
      const mrm::FingerReading f = fingers();
      std::ostringstream ss;
      ss << "fingers L " << f.left_q << " R " << f.right_q << ", last contact L " << f.left_contact_age << " R "
         << f.right_contact_age << " s ago";
      why = ss.str();
      if (f.left_q < p_released_q_ || f.right_q < p_released_q_ ||
        f.left_contact_age < p_no_contact_ || f.right_contact_age < p_no_contact_)
      {
        moveLine(pre, "retract");
        moveNamed("stow");
        return finish(Step::fail(PlaceObject::Result::RELEASE_NOT_CONFIRMED, "gripper did not let go: " + why));
      }
    }

    // 7. retract and stow, THEN let perception confirm the object is in the box: in stow the
    //    whole arm is out of the camera's view (P7, DC-25), so nothing of the robot can hide it
    //    (P10 run 2: 0 detections with the arm still above the box, DC-39)
    stage("retract");
    st = moveLine({place.x, place.y, place.z + p_retract_}, "retract");
    if (!st.ok) {moveNamed("stow"); return finish(st);}
    st = moveNamed("stow");
    if (!st.ok) {return finish(st);}
    if (p_check_percept_) {
      stage("verify");
      std::string pwhy;
      const bool seen = targetSeenAt(goal->place_pose, pwhy);
      why += "; " + pwhy;
      if (!seen) {
        return finish(Step::fail(PlaceObject::Result::RELEASE_NOT_CONFIRMED, "object not confirmed in the box: " + why));
      }
    }
    if (now().seconds() - t_start > p_place_timeout_) {
      return finish(Step::fail(PlaceObject::Result::TIMEOUT, "place took longer than place_timeout: " + why));
    }
    finish(Step{true, PlaceObject::Result::NONE, "placed: " + why});
  }

  /// >= release_detections fresh target detections, median within release_max_offset (xy) of
  /// the place point, all compared in the place goal's frame.
  bool targetSeenAt(const geometry_msgs::msg::PoseStamped & where, std::string & why)
  {
    if (det_reset_->wait_for_service(1s)) {
      auto fut = det_reset_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      fut.wait_for(2s);
    }
    {
      std::lock_guard<std::mutex> lk(mtx_);
      dets_.clear();
    }
    const double t0 = now().seconds();
    std::vector<double> d;
    while (now().seconds() - t0 < p_det_timeout_) {
      std::vector<my_robot_interfaces::msg::Detection> got;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        got = dets_;
      }
      d.clear();
      for (const auto & m : got) {
        geometry_msgs::msg::PointStamped pin, pout;
        pin.header = m.header;
        pin.header.stamp = rclcpp::Time(0);
        pin.point = m.pose.position;
        try {
          pout = tf_buffer_->transform(pin, where.header.frame_id, tf2::durationFromSec(0.2));
        } catch (const tf2::TransformException &) {
          continue;
        }
        d.push_back(std::hypot(pout.point.x - where.pose.position.x, pout.point.y - where.pose.position.y));
      }
      if (static_cast<int>(d.size()) >= p_det_needed_) {break;}
      std::this_thread::sleep_for(50ms);
    }
    std::ostringstream ss;
    if (static_cast<int>(d.size()) < p_det_needed_) {
      ss << "perception: " << d.size() << " target detections in " << p_det_timeout_ << " s";
      why = ss.str();
      return false;
    }
    std::sort(d.begin(), d.end());
    const double med = d[d.size() / 2];
    ss << "perception: target " << med * 1000 << " mm from the place point (" << d.size() << " detections)";
    why = ss.str();
    return med <= p_det_radius_;
  }

  // members
  mrm::ArmKinematics kin_;
  mrm::GraspCheck check_;
  double p_planning_time_, p_free_vel_, p_free_acc_, p_line_vel_, p_line_acc_, p_max_step_, p_below_top_;
  double p_approach_, p_lift_, p_min_clear_, p_open_q_, p_closed_q_, p_open_time_, p_close_time_;
  double p_settle_vel_, p_settle_time_, p_settle_timeout_, p_reverify_window_, p_timeout_, p_floor_z_;
  bool p_reverify_contact_{false};
  double p_hold_squeeze_{0.002}, p_hold_tol_{0.004};
  int p_attempts_, p_waypoints_;
  std::string p_target_model_, p_arm_base_;
  int lines_checked_{0};
  std::set<const moveit::core::LinkModel *> arm_links_;

  rclcpp::CallbackGroup::SharedPtr cb_action_, cb_sensors_, cb_clients_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attached_sub_;
  rclcpp_action::Client<FJT>::SharedPtr gripper_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr attach_;
  rclcpp_action::Server<PickObject>::SharedPtr pick_server_;
  rclcpp_action::Server<PlaceObject>::SharedPtr place_server_;
  rclcpp::Subscription<my_robot_interfaces::msg::Detection>::SharedPtr det_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr det_reset_;
  std::vector<my_robot_interfaces::msg::Detection> dets_;
  double p_place_clear_, p_preplace_, p_retract_, p_box_size_, p_box_h_, p_box_wall_, p_released_q_, p_no_contact_;
  double p_det_radius_, p_det_timeout_, p_place_timeout_;
  bool p_check_percept_{true};
  int p_det_needed_{3};
  std::string p_det_topic_, p_det_reset_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> scene_iface_;

  std::mutex mtx_;
  std::map<std::string, double> pos_, vel_;
  double last_contact_[2]{-1.0, -1.0};
  std::atomic<bool> busy_{false};
  std::atomic<bool> attached_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceServer>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});
  try {
    node->init();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "init failed: %s", e.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
