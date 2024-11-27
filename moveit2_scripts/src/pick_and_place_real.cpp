//pick_and_place_real.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <chrono>
#include <thread>

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

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
      move_group_arm.setPlanningTime(10.0);
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

  // Close Gripper at the Beginning
  RCLCPP_INFO(LOGGER, "Close Gripper to start");
  move_group_gripper.setNamedTarget("gripper_close");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;
  bool success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  if (success_gripper) {
    move_group_gripper.execute(my_plan_gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  } else {
    RCLCPP_ERROR(LOGGER, "Failed to plan gripper close.");
  }
  current_state_arm = move_group_arm.getCurrentState(10);
  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.28;
  move_group_arm.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);

  current_state_gripper = move_group_gripper.getCurrentState(10);
  // Open Gripper
  RCLCPP_INFO(LOGGER, "Open Gripper!");
  move_group_gripper.setNamedTarget("gripper_partially_open");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);

  current_state_arm = move_group_arm.getCurrentState(10);
  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.position.z -= 0.038;
  approach_waypoints.push_back(target_pose1);
  target_pose1.position.z -= 0.038;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.0002;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));


  current_state_gripper = move_group_gripper.getCurrentState(10);
  // Close Gripper
  RCLCPP_INFO(LOGGER, "Close Gripper!");
  move_group_gripper.setNamedTarget("gripper_partially_close");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));


  current_state_arm = move_group_arm.getCurrentState(10);
  // Retreat
  RCLCPP_INFO(LOGGER, "Retreat from opbject!");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.038;
  retreat_waypoints.push_back(target_pose1);
  target_pose1.position.z += 0.038;
  retreat_waypoints.push_back(target_pose1);
  moveit_msgs::msg::RobotTrajectory trajectory_retreat;
  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);
  move_group_arm.execute(trajectory_retreat);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

//   // Place
//   RCLCPP_INFO(LOGGER, "Rotating Arm");
//   current_state_arm = move_group_arm.getCurrentState(10);
//   current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                              joint_group_positions_arm);
//   joint_group_positions_arm[0] = 3.14; // Shoulder Pan
//   move_group_arm.setJointValueTarget(joint_group_positions_arm);
//   success_arm = (move_group_arm.plan(my_plan_arm) ==
//                  moveit::core::MoveItErrorCode::SUCCESS);
//   move_group_arm.execute(my_plan_arm);

// Define the increment in radians (60 degrees = π/3 radians)
double increment = M_PI; // 180 degrees in radians

// Get the current position of shoulder_pan_joint
current_state_arm = move_group_arm.getCurrentState(10);
current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                           joint_group_positions_arm);
double current_position = joint_group_positions_arm[0];

// Target position is 180 degrees (π radians)
double target_position = M_PI; // 180 degrees in radians

// Determine the direction of rotation
double direction = (target_position > current_position) ? 1.0 : -1.0;

// Calculate the number of steps required
int steps = 1; //std::ceil(std::abs(target_position - current_position) / increment);

// Create a vector to hold the intermediate positions
std::vector<double> intermediate_positions;

// Generate the intermediate positions
for (int i = 1; i <= steps; ++i) {
    double position = current_position + direction * increment * i;
    // Clamp to the target position to avoid overshooting
    if ((direction > 0 && position > target_position) ||
        (direction < 0 && position < target_position)) {
        position = target_position;
    }
    intermediate_positions.push_back(position);
    if (position == target_position) {
        break;
    }
}

for (double position : intermediate_positions) {
    // Copy the current joint positions
    current_state_arm = move_group_arm.getCurrentState(10);
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                               joint_group_positions_arm);

    // Set the new position for the shoulder_pan_joint
    joint_group_positions_arm[0] = position; // Shoulder Pan

    // Set the joint target
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::core::MoveItErrorCode::SUCCESS);

    if (success_arm) {
        move_group_arm.execute(my_plan_arm);
        // Optionally, add a small delay between steps
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } else {
        RCLCPP_WARN(LOGGER, "Failed to plan for position: %f radians", position);
        break;
    }
}


  // Open Gripper
  RCLCPP_INFO(LOGGER, "Release Object!");
  move_group_gripper.setNamedTarget("gripper_open");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");
  joint_group_positions_arm[0] = 0.0;  // Shoulder Pan
  joint_group_positions_arm[1] = -1.5708; // Shoulder Lift
  joint_group_positions_arm[2] = 0.0;  // Elbow
  joint_group_positions_arm[3] = -1.5708; // Wrist 1
  joint_group_positions_arm[4] = 0.0; // Wrist 2
  joint_group_positions_arm[5] = -1.5708;  // Wrist 3
  move_group_arm.setJointValueTarget(joint_group_positions_arm);
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  rclcpp::shutdown();
  return 0;
}
