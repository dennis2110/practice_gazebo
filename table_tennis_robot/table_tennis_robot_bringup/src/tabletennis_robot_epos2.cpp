#include "tabletennis_robot_epos2.h"

namespace TabletennisRobot
{
    EPOS2::EPOS2(){
        for(int i=0;i<5;i++){
            motor_cmd[i] = 0.0;
            last_motor_cmd[i] = 0.0;
            motor_status_msg.position.push_back(0);
            motor_status_msg.velocity.push_back(0);
        }
        motor_vel = 0.0;
        motor_pos = 0.0;
        loop_count_ = 0;
        stop_motor = true;
        motor_status_msg.name.push_back("real_joint_0_rpm");
        motor_status_msg.name.push_back("real_joint_1_rpm");
        motor_status_msg.name.push_back("real_joint_2_rpm");
        motor_status_msg.name.push_back("real_joint_3_rpm");
        motor_status_msg.name.push_back("real_joint_4_rpm");
        
    }

    EPOS2::~EPOS2(){

    }

    bool EPOS2::init(ros::NodeHandle &node){
        ///////////////////////
        ///// ROS setting /////
        ///////////////////////
        slide_rail_joint_command_sub_ = node.subscribe(
            "/TTbot/slide_rail_joint_position_controller/command",
             5, &EPOS2::slide_rail_Callback, this);

        arm_1_joint_command_sub_ = node.subscribe(
            "/TTbot/arm_joint1_position_controller/command",
             5, &EPOS2::arm_1_Callback, this);

        arm_2_joint_command_sub_ = node.subscribe(
            "/TTbot/arm_joint2_position_controller/command",
             5, &EPOS2::arm_2_Callback, this);
            
        arm_3_joint_command_sub_ = node.subscribe(
            "/TTbot/arm_joint3_position_controller/command",
             5, &EPOS2::arm_3_Callback, this);

        arm_4_joint_command_sub_ = node.subscribe(
            "/TTbot/arm_joint4_position_controller/command",
             5, &EPOS2::arm_4_Callback, this);
    
        statusCheck_service_ = node.advertiseService("/status_check",&EPOS2::statuscheck_Callback,this);
        openDevice_service_ = node.advertiseService("/open_device",&EPOS2::opendevice_Callback, this);
        closeDevice_service_ = node.advertiseService("/close_device", &EPOS2::closedevice_Callback, this);
        setPPM_service_ = node.advertiseService("/set_PPM", &EPOS2::setPPM_Callback, this);
        stop_service_ = node.advertiseService("/stop_motor",&EPOS2::stopMotor_Callback, this);
        homing_service_ = node.advertiseService("/homing", &EPOS2::homing_Callback, this);

        motor_status_pub_ = node.advertise<table_tennis_robot_msgsrv::motor_status>("/motor_status_rpm",10);
        // motor_status_publisher_.reset(
        //     new realtime_tools::RealtimePublisher<table_tennis_robot_msgsrv::motor_status>(node, "/motor_status_rpm", 1));
        
        ////////////////////////
        ///// EPOS setting /////
        ////////////////////////
        if(epos_device_.initialization() == MMC_SUCCESS)
        {
            ROS_INFO("init success");
            // if(motor_status_publisher_ && motor_status_publisher_->trylock())
            // {
            //     for(int i=0;i<5;i++){
            //         motor_status_publisher_->msg_.position.push_back(0);
            //         motor_status_publisher_->msg_.velocity.push_back(0);
            //     }
            //     motor_status_publisher_->unlockAndPublish();
            // }
            return true;
        }
        else
        {
            ROS_ERROR("init fail");
            return false;
        }
    }

    void EPOS2::update(){
        ros::Time start_update_time = ros::Time::now();
        if(epos_device_.motorEnableCheck()){
        //    if(epos_device_.PPMCheck()){
                if(!stop_motor){
                    loop_count_++;
                    //std::cout << motor_cmd[0] << " " << motor_cmd[1] <<std::endl;
                    //ROS_INFO("in loop : %d", loop_count_);
                    if(last_motor_cmd[0] != motor_cmd[0])
                        epos_device_.setPosition(epos_device_.g_pKeyHandle, epos_device_.g_usNodeId1, motor_cmd[0]);
                    if(last_motor_cmd[1] != motor_cmd[1])
                        epos_device_.setPosition(epos_device_.subKeyHandle, epos_device_.g_usNodeId2, motor_cmd[1]);
                    if(last_motor_cmd[2] != motor_cmd[2])
                        epos_device_.setPosition(epos_device_.subKeyHandle, epos_device_.g_usNodeId3, motor_cmd[2]);
                    if(last_motor_cmd[3] != motor_cmd[3])
                        epos_device_.setPosition(epos_device_.subKeyHandle, epos_device_.g_usNodeId4, motor_cmd[3]);
                    if(last_motor_cmd[4] != motor_cmd[4])
                        epos_device_.setPosition(epos_device_.subKeyHandle, epos_device_.g_usNodeId5, motor_cmd[4]);

                    last_motor_cmd[0] = motor_cmd[0];
                    last_motor_cmd[1] = motor_cmd[1];
                    last_motor_cmd[2] = motor_cmd[2];
                    last_motor_cmd[3] = motor_cmd[3];
                    last_motor_cmd[4] = motor_cmd[4];

                    




                    // if(motor_status_publisher_ && motor_status_publisher_->trylock())
                    // {
                    // epos_device_.getPosition(epos_device_.g_usNodeId1,&motor_status_publisher_->msg_.position.at(0));
                    // epos_device_.getPosition(epos_device_.g_usNodeId2,&motor_status_publisher_->msg_.position.at(1));
                    // epos_device_.getPosition(epos_device_.g_usNodeId3,&motor_status_publisher_->msg_.position.at(2));
                    // epos_device_.getPosition(epos_device_.g_usNodeId4,&motor_status_publisher_->msg_.position.at(3));
                    // epos_device_.getPosition(epos_device_.g_usNodeId5,&motor_status_publisher_->msg_.position.at(4));
                    // epos_device_.getVelocity(epos_device_.g_usNodeId1,&motor_status_publisher_->msg_.velocity.at(0));
                    // epos_device_.getVelocity(epos_device_.g_usNodeId2,&motor_status_publisher_->msg_.velocity.at(1));
                    // epos_device_.getVelocity(epos_device_.g_usNodeId3,&motor_status_publisher_->msg_.velocity.at(2));
                    // epos_device_.getVelocity(epos_device_.g_usNodeId4,&motor_status_publisher_->msg_.velocity.at(3));
                    // epos_device_.getVelocity(epos_device_.g_usNodeId5,&motor_status_publisher_->msg_.velocity.at(4));
                    // motor_status_publisher_->unlockAndPublish();
                    // }


                     epos_device_.getPosition(epos_device_.g_usNodeId1,&motor_status_msg.position.at(0));
                     epos_device_.getPosition(epos_device_.g_usNodeId2,&motor_status_msg.position.at(1));
                     epos_device_.getPosition(epos_device_.g_usNodeId3,&motor_status_msg.position.at(2));
                     epos_device_.getPosition(epos_device_.g_usNodeId4,&motor_status_msg.position.at(3));
                     epos_device_.getPosition(epos_device_.g_usNodeId5,&motor_status_msg.position.at(4));
                    
                     epos_device_.getVelocity(epos_device_.g_usNodeId1,&motor_status_msg.velocity.at(0));
                     epos_device_.getVelocity(epos_device_.g_usNodeId2,&motor_status_msg.velocity.at(1));
                     epos_device_.getVelocity(epos_device_.g_usNodeId3,&motor_status_msg.velocity.at(2));
                     epos_device_.getVelocity(epos_device_.g_usNodeId4,&motor_status_msg.velocity.at(3));
                     epos_device_.getVelocity(epos_device_.g_usNodeId5,&motor_status_msg.velocity.at(4));

                    motor_status_pub_.publish(motor_status_msg);
                }
        //    }
        }
        ros::Duration time_dura = ros::Time::now() - start_update_time;
        ROS_INFO("update duration: %4.3f s", time_dura.toSec());
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
        //add offset here
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
        response.MotorStatus = epos_device_.motorEnableCheck();
        response.HomingStatus = epos_device_.homingCheck();
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
        }else{
            stop_motor = false;
        }
        return true;
    }

}