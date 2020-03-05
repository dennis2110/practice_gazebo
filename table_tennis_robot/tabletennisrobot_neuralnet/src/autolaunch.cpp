#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "std_srvs/SetBool.h"
#include <stdio.h>

bool gazebo_stste = false;
int system_return;

bool gazebo_switch_callback(std_srvs::SetBool::Request &req,
                            						std_srvs::SetBool::Response &res)
{
	
	if(req.data == true)
	{
		if(gazebo_stste == true){
			res.success = false;
			res.message = "service fail. gazebo already open";
			return false;
		}
		//open gazebo
		gazebo_stste = true;
		system_return = system("roslaunch tabletennisrobot_gazebo ttbot_5f.launch &");
		res.message = "service success. gazebo is opened";
	}
	else if(req.data == false)
	{
		if(gazebo_stste == false){
			res.success = false;
			res.message = "service fail. gazebo already close";
			return false;
		}
		//close gazebo
		gazebo_stste = false;
		system_return = system("kill %1");
		res.message = "service success. gazebo is closed";
	}
	res.success = true;
	return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "autolaunch");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("gazebo_switch", gazebo_switch_callback);
	gazebo_stste = true;
	//system_return = system("roslaunch tabletennisrobot_gazebo ttbot_5f.launch &");
	//ros::Duration(10).sleep();
	
	//ROS_INFO("aaa");
	/*system_return = system("jobs");

	ros::Duration(3).sleep();
	system_return = system("kill %1");*/
	
	// FILE * fp;
	// char buffer[200];
	// fp=popen("ps -fC roslaunch","r");
	// fgets(buffer, sizeof (buffer),fp);
	// printf("aaa%s",buffer);
	// pclose(fp);

	system_return = system("ps -fC roslaunch");
	
 


	//ros::spin();
  return 0;
}