#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("move_down_cartesian_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    // Create the MoveGroupInterface for the UR3e manipulator
    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);



    // Set maximum velocity and acceleration scaling factors (optional)
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // Get the current pose of the end-effector
    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    // Define the target pose at z = 0.26
    geometry_msgs::msg::Pose target_pose = start_pose;
    target_pose.position.x = 0.337;
    target_pose.position.y = -0.02;
    target_pose.position.z = 0.26;

    // Set orientation (quaternion)
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    // Move to the starting position at z = 0.26
    move_group.setPoseTarget(target_pose);

    // Plan and execute the motion to the starting position
    moveit::planning_interface::MoveGroupInterface::Plan plan_to_start;
    bool success = (move_group.plan(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Planning to start position successful. Executing...");
        move_group.execute(plan_to_start);
        RCLCPP_INFO(node->get_logger(), "Reached starting position.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning to start position failed.");
        rclcpp::shutdown();
        return 1;
    }

    // Create a vector of waypoints for the Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Starting pose for the Cartesian path (should be the same as target_pose)
    geometry_msgs::msg::Pose cartesian_start_pose = target_pose;

    // Ending pose: move down by 0.08 meters along the Z-axis
    geometry_msgs::msg::Pose cartesian_target_pose = cartesian_start_pose;
    cartesian_target_pose.position.z -= 0.08; // Move down by 0.08 meters to z = 0.18

    // Add the waypoints
    waypoints.push_back(cartesian_start_pose);
    waypoints.push_back(cartesian_target_pose);

    // Set orientation constraint to maintain the end-effector's orientation
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "base_link";
    ocm.orientation = target_pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.01;
    ocm.absolute_y_axis_tolerance = 0.01;
    ocm.absolute_z_axis_tolerance = 0.01;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(path_constraints);

    // Plan the Cartesian path connecting the waypoints
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.99)
    {
        RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);

        // Optionally, time-parameterize the trajectory for smoother execution
        // For example, using IterativeParabolicTimeParameterization (requires additional setup)

        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group.execute(cartesian_plan);
        RCLCPP_INFO(node->get_logger(), "Motion execution complete.");
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Could only compute %.2f%% of the path. Planning failed.", fraction * 100.0);
    }

    // Clear path constraints after execution
    move_group.clearPathConstraints();

    // Shutdown ROS
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}

