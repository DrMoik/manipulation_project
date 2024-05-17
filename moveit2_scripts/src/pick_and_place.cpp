#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // Initialize the planning scene interface for collision checking
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define a collision object for the table
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();
  collision_object.id = "table";

  // Define a box for the table surface
  shape_msgs::msg::SolidPrimitive table_surface;
  table_surface.type = table_surface.BOX;
  table_surface.dimensions.resize(3);
  table_surface.dimensions[0] = 2.0;  // x dimension
  table_surface.dimensions[1] = 2.0;  // y dimension
  table_surface.dimensions[2] = 0.01; // z dimension, very thin

  // Define the pose of the table
  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.005; // Position it slightly below Z=0

  collision_object.primitives.push_back(table_surface);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Add the collision object to the world
  planning_scene_interface.addCollisionObjects(collision_objects);
  rclcpp::sleep_for(std::chrono::seconds(1));

  ///////////////////////////////////////////////////////////////////
  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  // joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -2.50; // Shoulder Lift
  joint_group_positions_arm[2] = 1.50;  // Elbow
  joint_group_positions_arm[3] = -1.50; // Wrist 1
  joint_group_positions_arm[4] = -1.55; // Wrist 2
  // joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to home position");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = -0.02;
  target_pose1.position.z = 0.264;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to pregrasp position");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("gripper_open");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.05;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.05;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  if (fraction < 0.9) {
    RCLCPP_ERROR(LOGGER, "Failed to plan a sufficient approach path");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(trajectory_approach);

  // Close Gripper

  RCLCPP_INFO(LOGGER, "Close Gripper!");
  move_group_gripper.setNamedTarget("gripper_middle1");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to close gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(my_plan_gripper);

  move_group_gripper.setNamedTarget("gripper_middle2");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to close gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(my_plan_gripper);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.04;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  if (fraction < 0.9) {
    RCLCPP_ERROR(LOGGER, "Failed to plan a sufficient approach path");
    rclcpp::shutdown();
    return -1;
  }

  move_group_arm.execute(trajectory_retreat);

  // Place

  RCLCPP_INFO(LOGGER, "Release Position");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] += 3.141592; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  if (!success_arm) {
    RCLCPP_ERROR(LOGGER, "Failed to plan to release position");
    rclcpp::shutdown();
    return -1;
  }
  move_group_arm.execute(my_plan_arm);

  // Open Gripper

  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("gripper_open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
  if (!success_gripper) {
    RCLCPP_ERROR(LOGGER, "Failed to open gripper");
    rclcpp::shutdown();
    return -1;
  }

  move_group_gripper.execute(my_plan_gripper);

  rclcpp::shutdown();
  return 0;
}