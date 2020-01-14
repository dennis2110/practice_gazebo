#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"

#include "std_msgs/Float64MultiArray.h"

double ball_x = 0.0, ball_y = 0.0, ball_z = 0.0;
double L1 = 0.26, L2 = 0.215, L3 = 0.26;
double inv_y0, inv_z0;
double inv_y1, inv_z1;
double s_square;
double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, psi = 0.0, beta = 0.0, phi = 0.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_ball_inverse");
  ros::NodeHandle n;
  
  ros::ServiceClient getModelState_client = n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  ros::Publisher unified_cmd_pub = n.advertise<std_msgs::Float64MultiArray>("/unified_joint_cmd",10);


  gazebo_msgs::GetModelState srv;
  srv.request.model_name = "ball";

  std_msgs::Float64MultiArray unified_cmd;
  // // init pose
  // unified_cmd.data.push_back(0.95); 
  // unified_cmd.data.push_back(1.3);
  // unified_cmd.data.push_back(1.1);
  // unified_cmd.data.push_back(4.2);
  // unified_cmd.data.push_back(3.6);
  // unified_cmd.data.push_back(0.26);
  // init pose (static)
  unified_cmd.data.push_back(0.5); 
  unified_cmd.data.push_back(1.57);
  unified_cmd.data.push_back(2.2); //2.2 //1.1
  unified_cmd.data.push_back(2.9); //2.9 //4.2
  unified_cmd.data.push_back(2.2); //2.2 //3.6
  unified_cmd.data.push_back(0.0);
  unified_cmd_pub.publish(unified_cmd);


  ros::Rate loop_rate(50);
  //ros::Time start_time = ros::Time::now();
  while (ros::ok())
  {
    // if((ros::Time::now()-start_time).toSec() > 2.0){
    //   break;
    // }
    if(getModelState_client.call(srv)){
        ball_x = srv.response.pose.position.x;
        ball_y = srv.response.pose.position.y;
        ball_z = srv.response.pose.position.z;
        //ROS_INFO("ball pos x %f",srv.response.pose.position.x);
    }
    inv_y1 = ball_y - L3;
    inv_z1 = ball_z;
    inv_y0 = 0.4735 - 0.5;
    inv_z0 = 0.937;
    s_square = pow(inv_y1-inv_y0, 2) + pow(inv_z1-inv_z0, 2);

    theta2 = acos((s_square - pow(L1,2) - pow(L2, 2)) / (2*L1*L2));
    beta = atan2(inv_z1-inv_z0, abs(inv_y1-inv_y0));
    psi = acos((s_square + pow(L1,2) - pow(L2,2)) / (2*sqrt(s_square)*L1));

    theta1 = beta + psi;
    theta3 = phi - theta1 - theta2;

    ROS_INFO("value: %4.3f %4.3f %4.3f %4.3f %4.3f", theta2*57.3, beta*57.3, psi*57.3, theta1*57.3, theta3*57.3);


    ROS_INFO("J2: %4.3f", 2.2-1.57-theta1);
    unified_cmd.data.at(2) = 2.2-1.57-theta1;
    //unified_cmd.data.at(3) = 2.9 + theta2;
    //unified_cmd.data.at(4) = 2.2 - theta3;
    unified_cmd_pub.publish(unified_cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}