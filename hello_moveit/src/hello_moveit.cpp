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
#include "geometry_msgs/msg/pose_array.hpp"
#include <interbotix_xs_msgs/msg/joint_group_command.hpp>
#include <interbotix_xs_msgs/srv/motor_gains.hpp>
#include <interbotix_xs_msgs/srv/register_values.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>


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
        lip_pose_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PoseArray>(
            "lip_pose", 10,
            std::bind(&CareMoveIt::plan_and_execute_cb, this,
            std::placeholders::_1));

        // Let's try to do some dynamixal register fun

        get_motor_reg_cli =
            node_ptr->create_client<interbotix_xs_msgs::srv::RegisterValues>(
                "/wx200/get_motor_registers", 10);
        set_motor_reg_cli =
            node_ptr->create_client<interbotix_xs_msgs::srv::RegisterValues>(
                "/wx200/set_motor_registers", 10);
        set_motor_pid_cli =
            node_ptr->create_client<interbotix_xs_msgs::srv::MotorGains>(
                "/wx200/set_motor_pid_gains", 10);

        while (!get_motor_reg_cli->wait_for_service(std::chrono::seconds{1})) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger,
                        "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(logger, "service not available, waiting again...");
        }


        GetDynamixelReg("Position_D_Gain");
        // RCLCPP_INFO_STREAM(logger, "AAAAA");
        GetDynamixelReg("Position_I_Gain");
        RCLCPP_INFO_STREAM(logger, "BBBBB");
        GetDynamixelReg("Position_P_Gain");
        RCLCPP_INFO_STREAM(logger, "CCCCC");


        // Now this is useable, but we must set all gains together, which I don'
        // want to. SetMotorPID("waist", 1920, 100, 200); SetMotorPID("shoulder",
        // 1920, 100, 200); SetMotorPID("elbow", 1920, 100, 200);
        // SetMotorPID("wrist_angle", 1920, 100, 200);

        // Clean these up later.

        SetDynamixelReg("waist", "Position_P_Gain", 1920);
        SetDynamixelReg("shoulder", "Position_P_Gain", 1920);
        SetDynamixelReg("elbow", "Position_P_Gain", 1920);
        SetDynamixelReg("wrist_angle", "Position_P_Gain", 1920);

        SetDynamixelReg("waist", "Position_I_Gain", 100);
        SetDynamixelReg("shoulder", "Position_I_Gain", 100);
        SetDynamixelReg("elbow", "Position_I_Gain", 100);
        SetDynamixelReg("wrist_angle", "Position_I_Gain", 100);

        SetDynamixelReg("waist", "Position_D_Gain", 200);
        SetDynamixelReg("shoulder", "Position_D_Gain", 200);
        SetDynamixelReg("elbow", "Position_D_Gain", 200);
        SetDynamixelReg("wrist_angle", "Position_D_Gain", 200);

        GetDynamixelReg("Position_D_Gain");
        GetDynamixelReg("Position_I_Gain");
        GetDynamixelReg("Position_P_Gain");
    }

private:


    void plan_and_execute_cb(const geometry_msgs::msg::PoseArray &point_msg) {

        ///the target pose initialization
        RCLCPP_INFO_STREAM(logger, "START PLAN AND EXECUTE");
        for(size_t i = 0; i < point_msg.poses.size(); i++){
            geometry_msgs::msg::Pose eachPose;
            eachPose.position.x = point_msg.poses[i].position.x;
            eachPose.position.y = point_msg.poses[i].position.y;
            eachPose.position.z = point_msg.poses[i].position.z;
            
            geometry_msgs::msg::Pose target_pose;
            // target_pose.position.x = point_msg.x;
            // target_pose.position.y = point_msg.y;
            // target_pose.position.z = point_msg.z;
            if(eachPose.position.z > 0.4){
                target_pose.position.x = 0.35;
            }
            else{
                target_pose.position.x = eachPose.position.z/1000 + 0.015;
            }
            target_pose.position.y = -eachPose.position.x/1000 - 0.165;
            target_pose.position.z = -eachPose.position.y/1000 + 0.175;
            RCLCPP_INFO_STREAM(logger, "robot command 111111" << " x "<< target_pose.position.x << " y " << target_pose.position.y << " z " << target_pose.position.z);



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
                RCLCPP_INFO_STREAM(logger, "SUCCESS PLAN1");
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
            geometry_msgs::msg::Pose reset_pose;
            reset_pose.position.x = 0.35;
            reset_pose.position.y = -0.00600772;
            reset_pose.position.z = 0.109082;
            double z_rst_angle = std::atan2(reset_pose.position.y, reset_pose.position.x);
            tf2::Quaternion target_q_reset;
            target_q_reset.setRPY(0.0, 0.0, z_rst_angle); // Set Euler angles

            reset_pose.orientation.x = target_q_reset.x();
            reset_pose.orientation.y = target_q_reset.y();
            reset_pose.orientation.z = target_q_reset.z();
            reset_pose.orientation.w = target_q_reset.w();

            move_group_interface.setPoseTarget(reset_pose);

            // Planning and execution
            moveit::planning_interface::MoveGroupInterface::Plan msg2;
            auto const success2 = static_cast<bool>(move_group_interface.plan(msg2));
            auto const plan2 = msg2;
            RCLCPP_INFO_STREAM(logger, "RECEIVE PLAN2");
            // Execute the plan
            if(success2) {
                move_group_interface.execute(plan2);
                RCLCPP_INFO_STREAM(logger, "SUCCESS PLAN2 in msg index" << i);
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        exit(1);
    }



    void GetDynamixelReg(std::string reg_name) {

    // clang-format off
    // string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
    // string name              # name of the group if commanding a joint group or joint if commanding a single joint
    // string reg               # register name (like Profile_Velocity, Profile_Acceleration, etc...)
    // int32 value              # desired register value (only set if 'setting' a register)
    // ---
    // int32[] values           # current register values (only filled if 'getting' a register)
    // clang-format on

        auto reg_req =
        std::make_shared<interbotix_xs_msgs::srv::RegisterValues_Request>();
        reg_req->cmd_type = "group";
        reg_req->name = "arm";
        reg_req->reg = reg_name;
        
        RCLCPP_INFO_STREAM(logger, "Getting reg value for " << reg_name);

        auto result = get_motor_reg_cli->async_send_request(reg_req);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_ptr, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            // RCLCPP_INFO_STREAM(logger, "" << result.get()->values);
            std::stringstream ss;
            for (int value : result.get()->values) {
                ss << value << " "; // Add each value to the stringstream, separated by spaces
            }
            RCLCPP_INFO_STREAM(logger, ss.str()); // Use ss.str() to get the string representation
        } else {
        RCLCPP_ERROR(logger, "Service call failed");
        }
    }

    void SetDynamixelReg(std::string joint_name, std::string reg_name, int32_t value) {

    // clang-format off
    // string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
    // string name              # name of the group if commanding a joint group or joint if commanding a single joint
    // string reg               # register name (like Profile_Velocity, Profile_Acceleration, etc...)
    // int32 value              # desired register value (only set if 'setting' a register)
    // ---
    // int32[] values           # current register values (only filled if 'getting' a register)
    // clang-format on

        auto reg_req =
            std::make_shared<interbotix_xs_msgs::srv::RegisterValues_Request>();
        reg_req->cmd_type = "single";
        reg_req->name = joint_name;
        reg_req->reg = reg_name;
        reg_req->value = value;

        RCLCPP_INFO_STREAM(logger, "Setting reg value for " << reg_name << " With "
                                                            << value);

        auto result = set_motor_reg_cli->async_send_request(reg_req);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_ptr, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {

        // RCLCPP_INFO_STREAM(logger, ""<< result.get()->values);
        RCLCPP_INFO_STREAM(logger, "Setting done");
        } else {
        RCLCPP_ERROR(logger, "Service call failed");
        }
    }

  void SetMotorPID(const std::string &joint_name, int32_t p, int32_t i, int32_t d) {

    // clang-format off
    // string cmd_type          # set to 'group' if commanding a joint group or 'single' if commanding a single joint
    // string name              # name of the group if commanding a joint group or joint if commanding a single joint
    // int32 kp_pos             # acts as a pass-through to the Position_P_Gain register
    // int32 ki_pos             # acts as a pass-through to the Position_I_Gain register
    // int32 kd_pos             # acts as a pass-through to the Position_D_Gain register
    // int32 k1                 # acts as a pass-through to the Feedforward_1st_Gain register
    // int32 k2                 # acts as a pass-through to the Feedforward_2nd_Gain register
    // int32 kp_vel             # acts as a pass-through to the Velocity_P_Gain register
    // int32 ki_vel             # acts as a pass-through to the Velocity_I_Gain register
    // ---

    // clang-format on

    auto pid_req =
        std::make_shared<interbotix_xs_msgs::srv::MotorGains_Request>();

    pid_req->cmd_type = "single";
    pid_req->name = joint_name;
    pid_req->kp_pos = p;
    pid_req->ki_pos = i;
    pid_req->kd_pos = d;
    pid_req->k1 = 0;
    pid_req->k2 = 0;
    pid_req->kp_vel = 100;
    pid_req->ki_vel = 1920;

    RCLCPP_INFO_STREAM(logger, "Setting PID to " << joint_name);

    auto result = set_motor_pid_cli->async_send_request(pid_req);
    // Wait for the result.

    if (rclcpp::spin_until_future_complete(node_ptr, result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(logger, "Setting successful");
    } else {
      RCLCPP_ERROR(logger, "Service call failed");
    }
  }

    rclcpp::Node::SharedPtr node_ptr;
    rclcpp::TimerBase::SharedPtr timer_;
    double rate_hz;
    rclcpp::Logger logger;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lip_pose_subscriber;
    rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr get_motor_reg_cli;
    rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr set_motor_reg_cli;
    rclcpp::Client<interbotix_xs_msgs::srv::MotorGains>::SharedPtr set_motor_pid_cli;
    
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
