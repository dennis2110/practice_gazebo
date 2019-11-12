#include "tabletennis_robot_epos2.h"

namespace TabletennisRobot
{
    EPOS2::EPOS2(){

    }

    EPOS2::~EPOS2(){

    }

    bool EPOS2::init(ros::NodeHandle &node){
        ///////////////////////
        ///// ROS setting /////
        ///////////////////////
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
    
        statusCheck_service_ = node.advertiseService("/status_check",&EPOS2::statuscheck_Callback,this);

        ////////////////////////
        ///// EPOS setting /////
        ////////////////////////
        if(epos_device_.initialization() == MMC_SUCCESS)
            ROS_INFO("init success");
        else
            ROS_ERROR("init fail");
        


        if(epos_device_.deviceOpenedCheck())
        {
            return true;
        }
        else
        {
            return false;
        }
        
    }

    void EPOS2::close_device(){
        if((epos_device_.closeDevice()) == MMC_FAILED)
            ROS_ERROR("Device closing failed");
        else
            ROS_INFO("Device closing successed");
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

    bool EPOS2::statuscheck_Callback(table_tennis_robot_msgsrv::EPOSstatus::Request& request, table_tennis_robot_msgsrv::EPOSstatus::Response& response){
        response.DeviceStatus = epos_device_.deviceOpenedCheck();
        response.HomingStatus = epos_device_.homingCheck();
        return true;
    }

}