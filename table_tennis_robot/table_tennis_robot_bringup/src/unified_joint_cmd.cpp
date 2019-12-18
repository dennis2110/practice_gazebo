#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

std_msgs::Float64MultiArray unified_cmd;

void unified_cmd_Callback(const std_msgs::Float64MultiArrayConstPtr& msg){
    unified_cmd = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unified_joint_cmd");

  ros::NodeHandle n;

  ros::Subscriber unified_cmd_sub_ = n.subscribe("/unified_joint_cmd",10,unified_cmd_Callback);
  ros::Publisher rel_joint_pub = n.advertise<std_msgs::Float64MultiArray>("/realTTbot/command",10);
  ros::Publisher sim_j0_pub = n.advertise<std_msgs::Float64>("/TTbot/slide_rail_joint_position_controller/command",10);
  ros::Publisher sim_j1_pub = n.advertise<std_msgs::Float64>("/TTbot/arm_joint1_position_controller/command",10);
  ros::Publisher sim_j2_pub = n.advertise<std_msgs::Float64>("/TTbot/arm_joint2_position_controller/command",10);
  ros::Publisher sim_j3_pub = n.advertise<std_msgs::Float64>("/TTbot/arm_joint3_position_controller/command",10);
  ros::Publisher sim_j4_pub = n.advertise<std_msgs::Float64>("/TTbot/arm_joint4_position_controller/command",10);
  
  for(int i=0;i<5;i++){
      unified_cmd.data.push_back(0.0);
  }

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    std_msgs::Float64MultiArray rel_cmd;
    std_msgs::Float64 sim_cmd;

    rel_cmd = unified_cmd;
    rel_joint_pub.publish(rel_cmd);

    sim_cmd.data = unified_cmd.data.at(0);
    sim_j0_pub.publish(sim_cmd);

    sim_cmd.data = unified_cmd.data.at(1);
    sim_j1_pub.publish(sim_cmd);

    sim_cmd.data = unified_cmd.data.at(2)-2.2;
    sim_j2_pub.publish(sim_cmd);

    sim_cmd.data = unified_cmd.data.at(3)-2.9;
    sim_j3_pub.publish(sim_cmd);

    sim_cmd.data = unified_cmd.data.at(4)-1.9;
    sim_j4_pub.publish(sim_cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}