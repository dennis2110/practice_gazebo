#ifndef TABLETENNIS_ROBOT_EPOS2_H
#define TABLETENNIS_ROBOT_EPOS2_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "Definitions.h"

#include <math.h>

namespace TabletennisRobot
{
class EPOS2
{
    public:
      EPOS2();
      ~EPOS2();
      bool init(ros::NodeHandle &node);

    private:
      void slide_rail_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_1_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_2_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_3_Callback(const std_msgs::Float64ConstPtr& command_msg);
      void arm_4_Callback(const std_msgs::Float64ConstPtr& command_msg);

    public:

    private:
      double motor_cmd[5];
      

      ros::Subscriber slide_rail_joint_command_sub_;
      ros::Subscriber arm_1_joint_command_sub_;
      ros::Subscriber arm_2_joint_command_sub_;
      ros::Subscriber arm_3_joint_command_sub_;
      ros::Subscriber arm_4_joint_command_sub_;
};

}




#endif // TABLETENNIS_ROBOT_EPOS2_H
