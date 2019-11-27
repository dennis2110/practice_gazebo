#include "tabletennis_robot_epos2.h"

namespace TabletennisRobot
{
    EPOS2::EPOS2(){
        for(int i=0;i<5;i++){
            motor_cmd[i] = 0.0;
        }
        motor_vel = 0.0;
        motor_pos = 0.0;
            
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
        openDevice_service_ = node.advertiseService("/open_device",&EPOS2::opendevice_Callback, this);
        closeDevice_service_ = node.advertiseService("/close_device", &EPOS2::closedevice_Callback, this);
        setPPM_service_ = node.advertiseService("/set_PPM", &EPOS2::setPPM_Callback, this);
        stop_service_ = node.advertiseService("/stop_motor",&EPOS2::stopMotor_Callback, this);
        homing_service_ = node.advertiseService("/homing", &EPOS2::homing_Callback, this);

        motor_status_pub_ = node.advertise<table_tennis_robot_msgsrv::motor_status>("/motor_status",10);

        ////////////////////////
        ///// EPOS setting /////
        ////////////////////////
        if(epos_device_.initialization() == MMC_SUCCESS)
        {
            ROS_INFO("init success");
            return true;
        }
        else
        {
            ROS_ERROR("init fail");
            return false;
        }
    }

    void EPOS2::update(){
        if(epos_device_.deviceOpenedCheck()){
            if(epos_device_.PPMCheck()){
                if(!stop_motor){
                    std::cout << motor_cmd[0] << " " << motor_cmd[1] <<std::endl;
            
                    epos_device_.setPosition(epos_device_.g_pKeyHandle, epos_device_.g_usNodeId1, motor_cmd[0]);




                    if(epos_device_.getPosition(epos_device_.g_usNodeId1,&motor_status_msg.position)==MMC_FAILED){
                        ROS_ERROR("123");
                    }
                    std::cout <<"######################" << motor_status_msg.position << std::endl;
                    if(epos_device_.getVelocity(epos_device_.g_usNodeId1,&motor_status_msg.velocity)==MMC_FAILED){
                        ROS_ERROR("456");
                    }
                    motor_status_pub_.publish(motor_status_msg);
                }
            }
        }
    }

    void EPOS2::close_device(){
        if(epos_device_.motorEnableCheck()){
            if(epos_device_.disableMotor() == MMC_FAILED){
                ROS_ERROR("motor disable failed");
            }
            else{
                ROS_INFO("motor disable successed");
            }
        }

        if(epos_device_.deviceOpenedCheck()){
            if(epos_device_.closeDevice() == MMC_FAILED)
                ROS_ERROR("Device closing failed");
            else
            {
                ROS_INFO("Device closing successed");
            }
        }
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
        stop_motor = false;
        response.DeviceStatus = epos_device_.deviceOpenedCheck();
        response.MotorStatus = epos_device_.motorEnableCheck();
        response.HomingStatus = epos_device_.homingCheck();
        epos_device_.getVelocityUnit();
        return true;
    }

    bool EPOS2::opendevice_Callback(table_tennis_robot_msgsrv::OpenDevice::Request& request, table_tennis_robot_msgsrv::OpenDevice::Response& response){
        if(!epos_device_.deviceOpenedCheck()){
            if(epos_device_.openDevice(request.deviceName, request.protocolStackName, request.interfaceName, request.portName, request.baudrate) == MMC_FAILED)
            {
                ROS_ERROR("open device fail");
            }
        }

        if(!epos_device_.motorEnableCheck()){
            if(epos_device_.enableMotor() == MMC_FAILED){
                ROS_ERROR("enable motor fail");
            }
        }

        return true;
    }

    bool EPOS2::closedevice_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
        close_device();
        return true;    
    }

    bool EPOS2::setPPM_Callback(table_tennis_robot_msgsrv::setPPMparam::Request& request, table_tennis_robot_msgsrv::setPPMparam::Response& response){
        if((int)request.usNodeId == 1){
            if(epos_device_.startProfilePositionMode(epos_device_.g_pKeyHandle, request.usNodeId, request.profile_velocity, request.profile_acceleration, request.profile_deceleration) == MMC_FAILED){
                ROS_ERROR("set PPM fail");
            }
        }else{
            if(epos_device_.startProfilePositionMode(epos_device_.subKeyHandle, request.usNodeId, request.profile_velocity, request.profile_acceleration, request.profile_deceleration) == MMC_FAILED){
                ROS_ERROR("set PPM fail");
            }
        }
        return true;
    }

    bool EPOS2::stopMotor_Callback(table_tennis_robot_msgsrv::stopMotor::Request& request, table_tennis_robot_msgsrv::stopMotor::Response& response){
        stop_motor = true;
        if((int)request.usNodeID == 1){
            if(epos_device_.stopPosition(epos_device_.g_pKeyHandle,request.usNodeID) == MMC_FAILED){
                ROS_ERROR("stop PPM fail");
            }
        }else{
            if(epos_device_.stopPosition(epos_device_.subKeyHandle,request.usNodeID) == MMC_FAILED){
                ROS_ERROR("stop PPM fail");
            }
        }
        return true;
    }

    bool EPOS2::homing_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
        if(epos_device_.autoHoming()==MMC_FAILED){
            ROS_ERROR("auto home fail");
        }
        return true;
    }

}