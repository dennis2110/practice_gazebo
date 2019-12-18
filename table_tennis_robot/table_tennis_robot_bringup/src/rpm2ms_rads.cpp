#include "ros/ros.h"
#include "table_tennis_robot_msgsrv/motor_status.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"

table_tennis_robot_msgsrv::motor_status rel_joint_state_rpm;

void motor_status_Callback(const table_tennis_robot_msgsrv::motor_statusConstPtr& motorstatus_msg){
    rel_joint_state_rpm = *motorstatus_msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rpm2ms_rads");

  for(int i=0;i<5;i++){
      rel_joint_state_rpm.position.push_back(0);
      rel_joint_state_rpm.velocity.push_back(0);
  }

  ros::NodeHandle n;

  ros::Subscriber motor_status_sub_ = n.subscribe("/motor_status_rpm",10,motor_status_Callback);
  ros::Publisher rel_joint_state = n.advertise<sensor_msgs::JointState>("/motor_status",10);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
      sensor_msgs::JointState real_joint_msg;
      real_joint_msg.position.at(0) = (double)rel_joint_state_rpm.position.at(0)/90000.0;
      real_joint_msg.position.at(1) = (double)rel_joint_state_rpm.position.at(1)/17303.0;
      real_joint_msg.position.at(2) = (double)rel_joint_state_rpm.position.at(2)/17360.0;
      real_joint_msg.position.at(3) = (double)rel_joint_state_rpm.position.at(3)/17303.0;
      real_joint_msg.position.at(4) = (double)rel_joint_state_rpm.position.at(4)/45034.0;

      real_joint_msg.velocity.at(0) = (double)rel_joint_state_rpm.velocity.at(0)*2048.0/(90000.0*60.0);
      real_joint_msg.velocity.at(1) = (double)rel_joint_state_rpm.velocity.at(1)*2048.0/(17303.0*60.0);
      real_joint_msg.velocity.at(2) = (double)rel_joint_state_rpm.velocity.at(2)*2048.0/(17360.0*60.0);
      real_joint_msg.velocity.at(3) = (double)rel_joint_state_rpm.velocity.at(3)*2048.0/(17303.0*60.0);
      real_joint_msg.velocity.at(4) = (double)rel_joint_state_rpm.velocity.at(4)*2048.0/(45034.0*60.0);

      rel_joint_state.publish(real_joint_msg);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}