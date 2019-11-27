#include "ros/ros.h"
#include "table_tennis_robot_msgsrv/motor_status.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

float rel_vel_joint0 = 0.0;
float rel_pos_joint0 = 0.0;

float sim_vel_joint0 = 0.0;
float sim_pos_joint0 = 0.0;

void motor_status_Callback(const table_tennis_robot_msgsrv::motor_statusConstPtr& motorstatus_msg){
    rel_vel_joint0 = motorstatus_msg->velocity;
    rel_pos_joint0 = motorstatus_msg->position;
}

void joint_status_Callback(const sensor_msgs::JointStateConstPtr& jointstate_msg){
    sim_vel_joint0 = jointstate_msg->velocity[4];
    sim_pos_joint0 = jointstate_msg->position[4];
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber motor_status_sub_ = n.subscribe("/motor_status",10,motor_status_Callback);
  ros::Subscriber joint_status_sub_ = n.subscribe("/linear_slide_rail/joint_states",10,joint_status_Callback);

  ros::Publisher sim_vel_pub = n.advertise<std_msgs::Float32>("/sim_vel",10);
  ros::Publisher sim_pos_pub = n.advertise<std_msgs::Float32>("/sim_pos",10);
  ros::Publisher rel_vel_pub = n.advertise<std_msgs::Float32>("/rel_vel",10);
  ros::Publisher rel_pos_pub = n.advertise<std_msgs::Float32>("/rel_pos",10);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
      std::cout << "rel_vel: " << rel_vel_joint0 << " rel_pos: " << rel_pos_joint0 << 
                  " sim_vel: " << sim_vel_joint0 << " sim_pos: " << sim_pos_joint0 << std::endl; 
      std_msgs::Float32 msg;
      msg.data = sim_vel_joint0;
      sim_vel_pub.publish(msg);
      msg.data = sim_pos_joint0;
      sim_pos_pub.publish(msg);

      msg.data = rel_vel_joint0;
      rel_vel_pub.publish(msg);
      msg.data = rel_pos_joint0;
      rel_pos_pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}