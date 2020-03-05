#include "gazebo_msgs/GetModelState.h"
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <vector>

#include "std_msgs/Float64MultiArray.h"

bool is_ball_right = false, is_ball_left = false;
bool start = true;
bool changepos = false, changeside = false;
// bool is_side_change = false;

double slide_cmd = 0.0;
double J1_cmd = 0.0;
double ball_x = 0.0, ball_y = 0.0, ball_z = 0.0;
double L1 = 0.253, L2 = 0.216, L3 = 0.26;
double inv_x0, inv_z0;
double inv_x1, inv_z1;
double s_square, s;
double theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, psi = 0.0, beta = 0.0,
       phi = 0.0;

int main(int argc, char **argv) {
	ros::init(argc, argv, "track_ball_inverse_right");
  	ros::NodeHandle n;

  	ros::ServiceClient getModelState_client =
			n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  	ros::Publisher unified_cmd_pub =
			n.advertise<std_msgs::Float64MultiArray>("/unified_joint_cmd", 10);

  	gazebo_msgs::GetModelState srv;
  	srv.request.model_name = "ball";

  	std_msgs::Float64MultiArray unified_cmd;

  	unified_cmd.data.push_back(0.4735);
  	unified_cmd.data.push_back(1.57);//1.57);
  	unified_cmd.data.push_back(2.2); // 2.2 //1.1
  	unified_cmd.data.push_back(2.9); // 2.9 //4.2
  	unified_cmd.data.push_back(2.2); // 2.2 //3.6
  	unified_cmd.data.push_back(0.0);
  	unified_cmd_pub.publish(unified_cmd);

  	ros::Rate loop_rate(50);
  	// ros::Time start_time = ros::Time::now();
  	while (ros::ok()) {
    	// if((ros::Time::now()-start_time).toSec() > 2.0){
    	//   break;
    	// }
    	if (getModelState_client.call(srv)) {
      		ball_x = srv.response.pose.position.x;
      		ball_y = srv.response.pose.position.y;
      		ball_z = srv.response.pose.position.z;
    	}

  		inv_x1 = ball_x - L3;
  		inv_z1 = ball_z;
  		inv_x0 = 0.4735 - unified_cmd.data.at(0);
    	inv_z0 = 0.94; // 0.937
    	s_square = pow(inv_x1 - inv_x0, 2) + pow(inv_z1 - inv_z0, 2);
    	s = sqrt(s_square);

		if (s <= L1 + L2) {
      		//theta2 = M_PIf64 -
      		//         acos((s_square - pow(L1, 2) - pow(L2, 2)) / (-2 * L1 * L2));
      		theta2 = -M_PIf64 +
            		acos((s_square - pow(L1, 2) - pow(L2, 2)) / (-2 * L1 * L2));
      		// theta2 = acos((-s_square + pow(L1,2) + pow(L2, 2)) / (2*L1*L2));
      		beta = atan2(abs(inv_z1 - inv_z0), abs(inv_x1 - inv_x0));
      		psi = acos((s_square + pow(L1, 2) - pow(L2, 2)) / (2 * s * L1));

      		theta1 = beta + psi;
      		theta3 = phi - theta1 - theta2;
	  		ROS_INFO("ball x: ,%4.3f , %4.3f , %4.3f", ball_x,ball_y,ball_z);
      		ROS_INFO("s^2:, %4.3f", s_square);
      		//ROS_INFO("x0: %4.3f, z0: %4.3f, x1: %4.3f, z1: %4.3f", inv_x0, inv_z0,
        	//       inv_x1, inv_z1);
			ROS_INFO("x0: ,%4.3f, %4.3f, %4.3f, %4.3f", inv_x0, inv_z0, inv_x1, inv_z1);
      		ROS_INFO("value: ,%4.3f ,%4.3f ,%4.3f ,%4.3f ,%4.3f", theta2 * 57.3,
               							beta * 57.3, psi * 57.3, theta1 * 57.3, theta3 * 57.3);
      		// ROS_INFO("value: %4.3f %4.3f %4.3f %4.3f %4.3f", theta2, beta, psi,
      		// theta1, theta3);

      		// ROS_INFO("J3: %4.3f", 2.9 + theta2);

	  		ROS_INFO("\n\n");
      		unified_cmd.data.at(2) = 0.63 + theta1; // 2.2-1.57+theta1;
      		unified_cmd.data.at(3) = 2.9 - theta2;
      		unified_cmd.data.at(4) = 2.2 + theta3;
    	}

    	unified_cmd_pub.publish(unified_cmd);
    	start = false;
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
 	 return 0;
}