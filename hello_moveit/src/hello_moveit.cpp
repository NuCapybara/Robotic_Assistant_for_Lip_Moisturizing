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
#include <visualization_msgs/msg/marker.hpp>
#include <interbotix_xs_msgs/srv/register_values.hpp>
#include <interbotix_xs_msgs/srv/detail/register_values__struct.hpp>
#include <interbotix_xs_msgs/srv/robot_info.hpp>
#include <sstream>


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
        node_ptr->declare_parameter<double>("rate", 0.1);
        rate_hz = node_ptr->get_parameter("rate").as_double();
        std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
        lip_pose_subscriber = node_ptr->create_subscription<geometry_msgs::msg::PoseArray>(
            "lip_pose", 1,
            std::bind(&CareMoveIt::store_plan_position, this,
            std::placeholders::_1));

        timer_ = node_ptr->create_wall_timer(
            rate, std::bind(&CareMoveIt::timer_callback, this));
        rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
        target_marker_pub = node_ptr->create_publisher<visualization_msgs::msg::Marker>("~/target", qos_policy);

        

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
        RCLCPP_INFO_STREAM(logger, "AAAAA");
        GetDynamixelReg("Position_I_Gain");
        RCLCPP_INFO_STREAM(logger, "BBBBB");
        GetDynamixelReg("Position_P_Gain");
        RCLCPP_INFO_STREAM(logger, "CCCCC");


        // Now this is useable, but we must set all gains together, which I don'
        // want to. SetMotorPID("waist", 1920, 100, 200); SetMotorPID("shoulder",
        // 1920, 100, 200); SetMotorPID("elbow", 1920, 100, 200);
        // SetMotorPID("wrist_angle", 1920, 100, 200);

        // Clean these up later.

        SetDynamixelReg("waist", "Position_P_Gain", 1900);
        SetDynamixelReg("shoulder", "Position_P_Gain", 2200);
        SetDynamixelReg("elbow", "Position_P_Gain", 1900);
        SetDynamixelReg("wrist_angle", "Position_P_Gain", 1900);

        SetDynamixelReg("waist", "Position_I_Gain", 400);
        // SetDynamixelReg("shoulder", "Position_I_Gain", 20);
        SetDynamixelReg("elbow", "Position_I_Gain", 400);
        SetDynamixelReg("wrist_angle", "Position_I_Gain", 400);

        SetDynamixelReg("waist", "Position_D_Gain", 300);
        SetDynamixelReg("shoulder", "Position_D_Gain", 400);
        SetDynamixelReg("elbow", "Position_D_Gain", 300);
        SetDynamixelReg("wrist_angle", "Position_D_Gain", 300);

        GetDynamixelReg("Position_D_Gain");
        GetDynamixelReg("Position_I_Gain");
        GetDynamixelReg("Position_P_Gain");
    }

private:


    void store_plan_position(const geometry_msgs::msg::PoseArray &point_msg) {
        maybe_curr_lip_pose = point_msg;
        // RCLCPP_INFO_STREAM(logger, "Im in store_plan_position");
        //  << curr_lip_pose.poses[0].position.x << " " << curr_lip_pose.poses[0].position.y << " " << curr_lip_pose.poses[0].position.z);
    }

    void timer_callback(){
        ///the target pose initialization
        if (! maybe_curr_lip_pose.has_value()){
            return;
        }


        auto curr_lip_pose  = maybe_curr_lip_pose.value();
        maybe_curr_lip_pose.reset();
        rclcpp::Time lip_time = curr_lip_pose.header.stamp ;
        rclcpp::Duration time_diff =  lip_time - node_ptr->get_clock()->now();
        RCLCPP_ERROR_STREAM(logger, "lip stamp "<< lip_time.nanoseconds() << " time_diff " <<
        time_diff.seconds() );

        RCLCPP_INFO_STREAM(logger, "START PLAN AND EXECUTE");
        RCLCPP_INFO_STREAM(logger, "curr_lip_pose.poses.size() " << curr_lip_pose.poses.size());
        for(size_t i = 0; i < 7; i++){
            geometry_msgs::msg::Pose eachPose;
            eachPose.position.x = curr_lip_pose.poses[i].position.x;
            eachPose.position.y = curr_lip_pose.poses[i].position.y;
            eachPose.position.z = curr_lip_pose.poses[i].position.z;

            ///a pose to help avoid collision on the face
            geometry_msgs::msg::Pose buffer_pose;
            geometry_msgs::msg::Pose target_pose;

            if(eachPose.position.z > 0.4){
                target_pose.position.x = 0.35;
                buffer_pose.position.x = 0.28;
            }
            else{
                /// before reaching target position, move to a buffer position first to realize dabbing
                buffer_pose.position.x = eachPose.position.z/1000 - 0.25 + 0.015;
                ///0.12065 for bar
                target_pose.position.x = eachPose.position.z/1000 - 0.12 + 0.015;

            }
            buffer_pose.position.y = -eachPose.position.x/1000 - 0.165;
            buffer_pose.position.z = -eachPose.position.y/1000 + 0.175;
            target_pose.position.y = -eachPose.position.x/1000 - 0.165;
            target_pose.position.z = -eachPose.position.y/1000 + 0.175;
            // RCLCPP_INFO_STREAM(logger, "robot command 111111" << " x "<< target_pose.position.x << " y " << target_pose.position.y << " z " << target_pose.position.z);
            ///Start buffer pose
            double z_angle_buffer = std::atan2(buffer_pose.position.y, buffer_pose.position.x);
            tf2::Quaternion target_q_buffer;
            target_q_buffer.setRPY(0.0, 0.0, z_angle_buffer); // Set Euler angles

            buffer_pose.orientation.x = target_q_buffer.x();
            buffer_pose.orientation.y = target_q_buffer.y();
            buffer_pose.orientation.z = target_q_buffer.z();
            buffer_pose.orientation.w = target_q_buffer.w();
            // RCLCPP_INFO_STREAM(logger, "robot command" << " x "<< buffer_pose.position.x << " y " << buffer_pose.position.y << " z " << buffer_pose.position.z << " w " << buffer_pose.orientation.w << " x " << buffer_pose.orientation.x << " y " << buffer_pose.orientation.y << " z " << buffer_pose.orientation.z);
            move_group_interface.setPoseTarget(buffer_pose);

            // Planning and execution
            moveit::planning_interface::MoveGroupInterface::Plan msg_bf;
            auto const success_bf = static_cast<bool>(move_group_interface.plan(msg_bf));
            auto const plan_bf = msg_bf;

            // Execute the plan
            if(success_bf) {
                move_group_interface.execute(plan_bf);
                RCLCPP_INFO_STREAM(logger, "SUCCESS Buffer Pose" << "index" << i);
            } else {
                RCLCPP_ERROR(logger, "Planning failed! Buffer Pose");
                RCLCPP_INFO_STREAM(logger, "FAIL Buffer Pose" << "index" << i);
            }
            rclcpp::sleep_for(std::chrono::seconds(1));

            ///start dabbing
            double z_angle = std::atan2(target_pose.position.y, target_pose.position.x);
            tf2::Quaternion target_q;
            target_q.setRPY(0.0, 0.0, z_angle); // Set Euler angles

            target_pose.orientation.x = target_q.x();
            target_pose.orientation.y = target_q.y();
            target_pose.orientation.z = target_q.z();
            target_pose.orientation.w = target_q.w();


            ///publishing visualization marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "wx200/base_link";
            marker.header.stamp = node_ptr->now();
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = target_pose.position.x;
            marker.pose.position.y = target_pose.position.y;
            marker.pose.position.z = target_pose.position.z;
            marker.pose.orientation.x = target_pose.orientation.x;
            marker.pose.orientation.y = target_pose.orientation.y;
            marker.pose.orientation.z =  target_pose.orientation.z;
            marker.pose.orientation.w = target_pose.orientation.w;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            target_marker_pub->publish(marker);

            RCLCPP_INFO_STREAM(logger, "robot command" << " x "<< target_pose.position.x << " y " << target_pose.position.y << " z " << target_pose.position.z << " w " << target_pose.orientation.w << " x " << target_pose.orientation.x << " y " << target_pose.orientation.y << " z " << target_pose.orientation.z);
            move_group_interface.setPoseTarget(target_pose);

            // Planning and execution
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const success = static_cast<bool>(move_group_interface.plan(msg));
            auto const plan = msg;

            // Execute the plan
            if(success) {
                move_group_interface.execute(plan);
                RCLCPP_INFO_STREAM(logger, "SUCCESS Target Pose index" << i);
            } else {
                RCLCPP_ERROR(logger, "Planning failed! index");
                RCLCPP_INFO_STREAM(logger, "FAIL Target Pose index" << i);
            }
            rclcpp::sleep_for(std::chrono::seconds(1));

            ///going back to reset_pose everytime after finishing a dabbing
            geometry_msgs::msg::Pose reset_pose;
            reset_pose.position.x = 0.25;
            reset_pose.position.y = -0.00600772;
            reset_pose.position.z = 0.109082;
            // reset_pose.position.z = 0.45;
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
            // RCLCPP_INFO_STREAM(logger, "RECEIVE PLAN2");
            // Execute the plan
            if(success2) {
                move_group_interface.execute(plan2);
                RCLCPP_INFO_STREAM(logger, "SUCCESS RESETTTTT in msg index RESETTTT" << i);
            } else {
                RCLCPP_ERROR(logger, "Planning failed!");
            }
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        maybe_curr_lip_pose.reset();
        // exit(1);
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
        RCLCPP_INFO_STREAM(logger, "RESULT RESLUT RESULT");
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_ptr, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {

            RCLCPP_INFO_STREAM(logger, "Successfully get reg value for " << reg_name);
        // RCLCPP_INFO_STREAM(logger, ""<< interbotix_xs_msgs::srv::to_yaml(result.get()));
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
    std::optional<geometry_msgs::msg::PoseArray> maybe_curr_lip_pose;
    rclcpp::Logger logger;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lip_pose_subscriber;
    rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr get_motor_reg_cli;
    rclcpp::Client<interbotix_xs_msgs::srv::RegisterValues>::SharedPtr set_motor_reg_cli;
    rclcpp::Client<interbotix_xs_msgs::srv::MotorGains>::SharedPtr set_motor_pid_cli;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub;
    
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
