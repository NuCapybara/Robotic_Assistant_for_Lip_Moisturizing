#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
// // Create the MoveIt MoveGroup Interface
///CAUSING PROBLEMS
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "interbotix_arm");

geometry_msgs::msg::Pose target_pose;
target_pose.position.y = 0.0;
target_pose.position.x = 0.0;
target_pose.position.z = 0.48; ///0.48 for the maximum height singlely on z
double z_angle = std::atan2(target_pose.position.y, target_pose.position.x);
tf2::Quaternion target_q;
// Here is the magic, the description of setEuler is mis-leading! 
target_q.setEuler(0.0, 0.0, z_angle);

target_pose.orientation.w = target_q.w();
target_pose.orientation.x = target_q.x();
target_pose.orientation.y = target_q.y();
target_pose.orientation.z = target_q.z();
// Set a target Pose

move_group_interface.setPoseTarget(target_pose);


// // Set a target Position
// double x = 0.21;
// double y = 0.0;
// double z = 0.15803736261075846;
// move_group_interface.setPositionTarget(x ,y ,z);

// // Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}