#ifndef TABLETENNIS_ROBOT_EPOS2_H
#define TABLETENNIS_ROBOT_EPOS2_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tabletennisrobot_msgsrv/EPOSstatus.h>
#include <tabletennisrobot_msgsrv/OpenDevice.h>
#include <tabletennisrobot_msgsrv/setPPMparam.h>
//#include <table_tennis_robot_msgsrv/motor_status.h>
#include <tabletennisrobot_msgsrv/stopMotor.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/JointState.h>

#include "epos_communication.h"

#include <math.h>

#include <memory>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#define JOINT_NUM 6

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
      void real_joint_Callback(const std_msgs::Float64MultiArrayConstPtr& command_msg);
      bool statuscheck_Callback(tabletennisrobot_msgsrv::EPOSstatus::Request& request, tabletennisrobot_msgsrv::EPOSstatus::Response& response);
      bool opendevice_Callback(tabletennisrobot_msgsrv::OpenDevice::Request& request, tabletennisrobot_msgsrv::OpenDevice::Response& response);
      bool closedevice_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
      bool setPPM_Callback(tabletennisrobot_msgsrv::setPPMparam::Request& request, tabletennisrobot_msgsrv::setPPMparam::Response& response);
      bool stopMotor_Callback(tabletennisrobot_msgsrv::stopMotor::Request& request, tabletennisrobot_msgsrv::stopMotor::Response& response);
      bool homing_Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    public:

    private:
      int result_fun;
      unsigned int loop_count_;
      double motor_cmd[JOINT_NUM];
      double last_motor_cmd[JOINT_NUM];
      float motor_vel;
      float motor_pos;
      bool stop_motor;
      int joint4_count;
      int joint5_count;
      

      //table_tennis_robot_msgsrv::motor_status motor_status_msg;
      sensor_msgs::JointState motor_status_msg;

//////////////////////////////////////////////////////////////////
      ros::Subscriber real_joint_command_sub_;

      ros::ServiceServer statusCheck_service_;
      ros::ServiceServer openDevice_service_;
      ros::ServiceServer closeDevice_service_;
      ros::ServiceServer setPPM_service_;
      ros::ServiceServer stop_service_;
      ros::ServiceServer homing_service_;

      ros::Publisher motor_status_pub_;
      //std::unique_ptr<
      // realtime_tools::RealtimePublisher<
      //  table_tennis_robot_msgsrv::motor_status> > motor_status_publisher_ ;

      EposCommunication epos_device_;
};

}




#endif // TABLETENNIS_ROBOT_EPOS2_H
