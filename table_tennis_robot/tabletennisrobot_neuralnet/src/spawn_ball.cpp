#include <iostream>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <vector>
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "spawn_ball");
	ros::NodeHandle n;

	ros::ServiceClient spawnModel_client = n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	ros::ServiceClient deleteModel_client = n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

	//SpawnModel Client
	gazebo_msgs::SpawnModel spawnModel_srv;
	std::fstream ifs;
	ifs.open("/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/pingpong_description/models/pingpong.sdf", std::ios::in);
	spawnModel_srv.request.model_name = "ball2";
	//ifs >> spawnModel_srv.request.model_xml;
	spawnModel_srv.request.model_xml.assign((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
	// std::cout << "#################" << std::endl;
	// std::cout << spawnModel_srv.request.model_xml.c_str();
	// std::cout << "#################" << std::endl;
	spawnModel_srv.request.initial_pose.position.x = -1.0;
	spawnModel_srv.request.initial_pose.position.y = 0.0;
	spawnModel_srv.request.initial_pose.position.z = 2.0;
	spawnModel_srv.request.initial_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	spawnModel_srv.request.reference_frame = "";
	spawnModel_client.call(spawnModel_srv);
	// ros::Duration(5.0).sleep();

	// ros::Rate loop_rate(50);
	// ros::Time start_time = ros::Time::now();
	// while (ros::ok())
	// {
	//   ros::spinOnce();
	//   loop_rate.sleep();
	// }

	// //DeleteModel Client
	// gazebo_msgs::DeleteModel delete_srv;
	// delete_srv.request.model_name = "ball2";
	// deleteModel_client.call(delete_srv);

	return 0;
}