#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using moveit::planning_interface::MoveGroupInterface;

class CareMoveIt {
public:
    CareMoveIt(rclcpp::Node::SharedPtr node_ptr)
    : node_ptr(node_ptr),
      logger(node_ptr->get_logger()),
      move_group_interface(MoveGroupInterface(node_ptr, "interbotix_arm"))
    {
         RCLCPP_INFO_STREAM(logger, "I am in the constructor");
        // Parameter declaration
        node_ptr->declare_parameter<double>("rate", 100.);
        rate_hz = node_ptr->get_parameter("rate").as_double();
        std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
        lip_pose_subscriber = node_ptr->create_subscription<geometry_msgs::msg::Point>(
            "lip_pose", 10,
            std::bind(&CareMoveIt::plan_and_execute_cb, this,
            std::placeholders::_1));
    }

private:
    // void timer_callback() {
    //     RCLCPP_INFO(logger, "Timer callback triggered");
    //     plan_and_execute();
    // }

    void plan_and_execute_cb(const geometry_msgs::msg::Point &point_msg) {
        RCLCPP_INFO_STREAM(logger, "I RECEIVED MSG" << " "<< point_msg.x << " " << point_msg.y << " " << point_msg.z);
        geometry_msgs::msg::Pose target_pose;

        // target_pose.position.x = point_msg.x;
        // target_pose.position.y = point_msg.y;
        // target_pose.position.z = point_msg.z;
        if(point_msg.z > 0.4){
            target_pose.position.x = 0.35;
        }
        else{
            target_pose.position.x = point_msg.z/1000 + 0.015;
        }
        target_pose.position.y = -point_msg.x/1000 - 0.165;
        target_pose.position.z = -point_msg.y/1000 + 0.175;
        RCLCPP_INFO_STREAM(logger, "robot command" << " x "<< target_pose.position.x << " y " << target_pose.position.y << " z " << target_pose.position.z);

        // target_pose.position.x = 0.0;
        // target_pose.position.y = 0.0;
        // target_pose.position.z = 0.48; // Maximum height on zour

        double z_angle = std::atan2(target_pose.position.y, target_pose.position.x);
        tf2::Quaternion target_q;
        target_q.setRPY(0.0, 0.0, z_angle); // Set Euler angles

        target_pose.orientation.x = target_q.x();
        target_pose.orientation.y = target_q.y();
        target_pose.orientation.z = target_q.z();
        target_pose.orientation.w = target_q.w();

        move_group_interface.setPoseTarget(target_pose);

        // Planning and execution
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const success = static_cast<bool>(move_group_interface.plan(msg));
        auto const plan = msg;

        // Execute the plan
        if(success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        // geometry_msgs::msg::TransformStamped t;

        // t.header.stamp = node_ptr->get_clock()->now();
        // t.header.frame_id = "wx200/base_link";
        // t.child_frame_id = "motion_target";
        // t.transform.translation.x = target_pose.position.x;
        // t.transform.translation.y = target_pose.position.y;
        // t.transform.translation.z = target_pose.position.z;
        // t.transform.rotation.x = target_pose.orientation.x;
        // t.transform.rotation.y = target_pose.orientation.y;
        // t.transform.rotation.z = target_pose.orientation.z;
        // t.transform.rotation.w = target_pose.orientation.w;
        
    }

    rclcpp::Node::SharedPtr node_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_hz;
    rclcpp::Logger logger;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr lip_pose_subscriber;
    
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
            true));
    auto const hello_moveit = std::make_shared<CareMoveIt>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
