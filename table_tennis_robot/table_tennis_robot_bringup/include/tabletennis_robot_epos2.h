#ifndef TABLETENNIS_ROBOT_EPOS2_H
#define TABLETENNIS_ROBOT_EPOS2_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <table_tennis_robot_msgsrv/EPOSstatus.h>
#include <table_tennis_robot_msgsrv/OpenDevice.h>
#include <table_tennis_robot_msgsrv/setPPMparam.h>
#include <table_tennis_robot_msgsrv/motor_status.h>
#include <table_tennis_robot_msgsrv/stopMotor.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/JointState.h>

#include "epos_communication.h"

#include <math.h>

namespace TabletennisRobot
{
class EPOS2
{
    public:
      EPOS2();
      ~EPOS2();
      bool init(ros::NodeHandle &node);
      void update();
      void close_device();



    private:
      void slide_rail_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_1_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_2_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_3_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_4_Callback(const std_msgs::Float64ConstPtr& command_msg);
      bool statuscheck_Callback(table_tennis_robot_msgsrv::EPOSstatus::Request& request, table_tennis_robot_msgsrv::EPOSstatus::Response& response);
      bool opendevice_Callback(table_tennis_robot_msgsrv::OpenDevice::Request& request, table_tennis_robot_msgsrv::OpenDevice::Response& response);
      bool closedevice_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
      bool setPPM_Callback(table_tennis_robot_msgsrv::setPPMparam::Request& request, table_tennis_robot_msgsrv::setPPMparam::Response& response);
      bool stopMotor_Callback(table_tennis_robot_msgsrv::stopMotor::Request& request, table_tennis_robot_msgsrv::stopMotor::Response& response);
      bool homing_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    public:

    private:
      int result_fun;
      unsigned int loop_count_;
      double motor_cmd[5];
      double last_motor_cmd[5];
      float motor_vel;
      float motor_pos;
      bool stop_motor;
      

      table_tennis_robot_msgsrv::motor_status motor_status_msg;
      //sensor_msgs::JointState motor_status_msg;

//////////////////////////////////////////////////////////////////
      ros::Subscriber slide_rail_joint_command_sub_;
      ros::Subscriber arm_1_joint_command_sub_;
      ros::Subscriber arm_2_joint_command_sub_;
      ros::Subscriber arm_3_joint_command_sub_;
      ros::Subscriber arm_4_joint_command_sub_;

      ros::ServiceServer statusCheck_service_;
      ros::ServiceServer openDevice_service_;
      ros::ServiceServer closeDevice_service_;
      ros::ServiceServer setPPM_service_;
      ros::ServiceServer stop_service_;
      ros::ServiceServer homing_service_;

      ros::Publisher motor_status_pub_;

      EposCommunication epos_device_;
};

}




#endif // TABLETENNIS_ROBOT_EPOS2_H
