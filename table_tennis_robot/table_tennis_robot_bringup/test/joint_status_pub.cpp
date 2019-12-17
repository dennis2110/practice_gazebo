#include "ros/ros.h"
#include "table_tennis_robot_msgsrv/motor_status.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

table_tennis_robot_msgsrv::motor_status rel_joint_state;
sensor_msgs::JointState sim_joint_state;

/*float rel_vel_joint0 = 0.0;
float rel_pos_joint0 = 0.0;

float sim_vel_joint0 = 0.0;
float sim_pos_joint0 = 0.0;*/

void motor_status_Callback(const table_tennis_robot_msgsrv::motor_statusConstPtr& motorstatus_msg){
    rel_joint_state = *motorstatus_msg;
    //rel_vel_joint0 = motorstatus_msg->velocity[0];
    //rel_pos_joint0 = motorstatus_msg->position[0];
}

void joint_status_Callback(const sensor_msgs::JointStateConstPtr& jointstate_msg){
    sim_joint_state = *jointstate_msg;
    //sim_vel_joint0 = jointstate_msg->velocity[0];
    //sim_pos_joint0 = jointstate_msg->position[0];//-0.72;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  if (argc != 2)
  {
    ROS_INFO("usage: linstener joint_X");
    return 1;
  }
  for(int i=0;i<5;i++){
      rel_joint_state.position.push_back(0);
      rel_joint_state.velocity.push_back(0);
      sim_joint_state.position.push_back(0.0);
      sim_joint_state.velocity.push_back(0.0);
  }
  int joint_x = atoi(argv[1]);

  ros::NodeHandle n;

  ros::Subscriber motor_status_sub_ = n.subscribe("/motor_status",10,motor_status_Callback);
  ros::Subscriber joint_status_sub_ = n.subscribe("/TTbot/joint_states",10,joint_status_Callback);

  ros::Publisher sim_vel_pub = n.advertise<std_msgs::Float32>("/sim_vel",10);
  ros::Publisher sim_pos_pub = n.advertise<std_msgs::Float32>("/sim_pos",10);
  ros::Publisher rel_vel_pub = n.advertise<std_msgs::Float32>("/rel_vel",10);
  ros::Publisher rel_pos_pub = n.advertise<std_msgs::Float32>("/rel_pos",10);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
      std::cout << "rel_vel: " << rel_joint_state.velocity[joint_x] << " rel_pos: " << rel_joint_state.position[joint_x] << 
                  " sim_vel: " << sim_joint_state.velocity[joint_x] << " sim_pos: " << sim_joint_state.position[joint_x] << std::endl; 
      std_msgs::Float32 msg;
      msg.data = sim_joint_state.velocity[joint_x];
      sim_vel_pub.publish(msg);
      msg.data = sim_joint_state.position[joint_x];
      sim_pos_pub.publish(msg);

      /*msg.data = rel_vel_joint0;
      rel_vel_pub.publish(msg);
      msg.data = rel_pos_joint0;
      rel_pos_pub.publish(msg);*/

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}