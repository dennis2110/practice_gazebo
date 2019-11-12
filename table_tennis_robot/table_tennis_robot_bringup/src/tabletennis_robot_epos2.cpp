#include "tabletennis_robot_epos2.h"

namespace TabletennisRobot
{
    EPOS2::EPOS2(){

    }

    EPOS2::~EPOS2(){

    }

    bool EPOS2::init(ros::NodeHandle &node){
        slide_rail_joint_command_sub_ = node.subscribe(
            "/linear_slide_rail/slide_rail_joint_position_controller/command",
             5, &EPOS2::slide_rail_Callback, this);

        arm_1_joint_command_sub_ = node.subscribe(
            "/linear_slide_rail/arm_joint1_position_controller/command",
             5, &EPOS2::arm_1_Callback, this);

        arm_2_joint_command_sub_ = node.subscribe(
            "/linear_slide_rail/arm_joint2_position_controller/command",
             5, &EPOS2::arm_2_Callback, this);
            
        arm_3_joint_command_sub_ = node.subscribe(
            "/linear_slide_rail/arm_joint3_position_controller/command",
             5, &EPOS2::arm_3_Callback, this);

        arm_4_joint_command_sub_ = node.subscribe(
            "/linear_slide_rail/arm_joint4_position_controller/command",
             5, &EPOS2::arm_4_Callback, this);
    
        return true;
    }

    void EPOS2::slide_rail_Callback(const std_msgs::Float64ConstPtr& command_msg){
        motor_cmd[0] = command_msg->data;
    }

    void EPOS2::arm_1_Callback(const std_msgs::Float64ConstPtr& command_msg){
        motor_cmd[1] = command_msg->data;
    }

    void EPOS2::arm_2_Callback(const std_msgs::Float64ConstPtr& command_msg){
        motor_cmd[2] = command_msg->data;
    }

    void EPOS2::arm_3_Callback(const std_msgs::Float64ConstPtr& command_msg){
        motor_cmd[3] = command_msg->data;
    }

    void EPOS2::arm_4_Callback(const std_msgs::Float64ConstPtr& command_msg){
        motor_cmd[4] = command_msg->data;
    }

}