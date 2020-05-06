#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"


std_msgs::Float64MultiArray unified_cmd;

void j0_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(0) = msg->data;
}
void j1_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(1) = msg->data;
}
void j2_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(2) = msg->data;
}
void j3_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(3) = msg->data;
}
void j4_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(4) = msg->data;
}
void j5_cmd_Callback(const std_msgs::Float64ConstPtr& msg){
    unified_cmd.data.at(5) = msg->data;
}

std_msgs::Float64MultiArray sim_to_real_cmd(std_msgs::Float64MultiArray sim_cmd){
    std_msgs::Float64MultiArray real_cmd = sim_cmd;

    real_cmd.data.at(0) = 0.5+sim_cmd.data.at(0);
    real_cmd.data.at(1) = sim_cmd.data.at(1)-3.14;
    real_cmd.data.at(2) = 2.2 - sim_cmd.data.at(2);
    real_cmd.data.at(3) = 2.9 - sim_cmd.data.at(3);
    real_cmd.data.at(4) = 512 + (sim_cmd.data.at(4)*180/M_PI/0.322);
    real_cmd.data.at(5) = 512 + (sim_cmd.data.at(5)*180/M_PI/0.322);

    //sim_cmd.data = 1.2 - unified_cmd.data.at(0);
    //sim_cmd.data = unified_cmd.data.at(1);
    //sim_cmd.data = 2.2 - unified_cmd.data.at(2);
    //sim_cmd.data = 2.9 - unified_cmd.data.at(3);
    //sim_cmd.data = 2.2 - unified_cmd.data.at(4);

    return real_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "to_real_cmd");
  ros::NodeHandle n;
  
  ros::Publisher rel_joint_pub = n.advertise<std_msgs::Float64MultiArray>("/realTTbot/command",10);

  ros::Subscriber sim_j0_sub = n.subscribe("/TTbot/slide_rail_joint_position_controller/command",10,j0_cmd_Callback);
  ros::Subscriber sim_j1_sub = n.subscribe("/TTbot/arm_joint1_position_controller/command",10,j1_cmd_Callback);
  ros::Subscriber sim_j2_sub = n.subscribe("/TTbot/arm_joint2_position_controller/command",10,j2_cmd_Callback);
  ros::Subscriber sim_j3_sub = n.subscribe("/TTbot/arm_joint3_position_controller/command",10,j3_cmd_Callback);
  ros::Subscriber sim_j4_sub = n.subscribe("/TTbot/arm_joint4_position_controller/command",10,j4_cmd_Callback);
  ros::Subscriber sim_j5_sub = n.subscribe("/TTbot/arm_joint5_position_controller/command",10,j5_cmd_Callback);
  
  
  unified_cmd.data.push_back(0.0);
  unified_cmd.data.push_back(0.0);
  unified_cmd.data.push_back(0.0);
  unified_cmd.data.push_back(0.0);
  unified_cmd.data.push_back(0.0);
  unified_cmd.data.push_back(0.0);
  

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    std_msgs::Float64MultiArray rel_cmd;
    
    rel_cmd = sim_to_real_cmd(unified_cmd);
    rel_joint_pub.publish(rel_cmd);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}