#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SpawnModel.h"
#include "ros/ros.h"
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <vector>

#include "std_msgs/Float64MultiArray.h"

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
std::vector<double> matrix;

void readTrajectoryFromData()
{
	std::ifstream fin;
	fin.open("/home/denn1slu/Dennis/pingpong_trajectory/30fps_test_gazebo.csv",
			 std::ios::in);
	// fin.open("/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/pingpong_description/models/pingpong.sdf",
	// std::ios::in); std::cout << "###############"<<std::endl; std::cout <<
	// fin.rdbuf(); std::cout <<
	// "###############"<<std::endl;
	std::string line;
	while (std::getline(fin, line))
	{
		std::istringstream templine(line); // string 轉換成 stream
		std::string data;
		while (std::getline(templine, data, ',')) //讀檔讀到逗號
		{
			matrix.push_back(atof(data.c_str())); // string 轉換成數字
		}
	}
	ROS_INFO("in  matrix: %d", matrix.size());
	fin.close();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_ball_move_arm");
	ros::NodeHandle n;

	ros::ServiceClient spawnModel_client =
		n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	ros::ServiceClient deleteModel_client =
		n.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
	ros::ServiceClient setModelState_client =
		n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	ros::ServiceClient getModelState_client =
		n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	ros::Publisher unified_cmd_pub =
		n.advertise<std_msgs::Float64MultiArray>("/unified_joint_cmd", 10);

	// //SpawnModel Client
	// gazebo_msgs::SpawnModel spawnModel_srv;
	// std::fstream ifs;
	// ifs.open("/home/denn1slu/ros_workspace/vscode_ws/src/table_tennis_robot/pingpong_description/models/pingpong.sdf",
	// std::ios::in); spawnModel_srv.request.model_name = "ball2"; ifs >>
	// spawnModel_srv.request.model_xml;
	// spawnModel_srv.request.initial_pose.position.x = -1.0;
	// spawnModel_srv.request.initial_pose.position.y = 0.0;
	// spawnModel_srv.request.initial_pose.position.z = 2.0;
	// spawnModel_srv.request.initial_pose.orientation =
	// tf::createQuaternionMsgFromYaw(0.0); spawnModel_srv.request.reference_frame
	// = ""; spawnModel_client.call(spawnModel_srv); ros::Duration(5.0).sleep();

	//////////////// test //////////////
	readTrajectoryFromData();
	ROS_INFO("matrix: %d", matrix.size());
	ROS_INFO("matrix[0]: %f   [2]: %f", matrix.at(0), matrix.at(2));
	///////////////////////////////////

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
	unified_cmd.data.push_back(0.95);
	unified_cmd.data.push_back(1.57);
	unified_cmd.data.push_back(1.1);
	unified_cmd.data.push_back(4.2);
	unified_cmd.data.push_back(3.6);
	unified_cmd.data.push_back(0.0);
	unified_cmd_pub.publish(unified_cmd);

	ros::Duration(1.0).sleep();

	gazebo_msgs::SetModelState setModetState_srv;
	setModetState_srv.request.model_state.model_name = "ball";
	setModetState_srv.request.model_state.pose.position.x = matrix.at(0);
	setModetState_srv.request.model_state.pose.position.y = matrix.at(1);
	setModetState_srv.request.model_state.pose.position.z = matrix.at(2);
	setModetState_srv.request.model_state.twist.linear.x =
		(matrix.at(3) - matrix.at(0)) * 30;
	setModetState_srv.request.model_state.twist.linear.y =
		(matrix.at(4) - matrix.at(1)) * 30;
	setModetState_srv.request.model_state.twist.linear.z =
		(matrix.at(5) - matrix.at(2)) * 30;
	setModelState_client.call(setModetState_srv);

	ros::Rate loop_rate(50);
	ros::Time start_time = ros::Time::now();
	while (ros::ok())
	{
		if ((ros::Time::now() - start_time).toSec() > 2.0)
		{
			break;
		}

		if (getModelState_client.call(srv))
		{
			ROS_INFO("ball pos x %f", srv.response.pose.position.x);
		}

		if (srv.response.pose.position.x > 0.8)
		{
			// unified_cmd.data.at(1) = 1.9;
		}
		else
		{
			// unified_cmd.data.at(1) = 2.0;
		}

		unified_cmd_pub.publish(unified_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}

	// //DeleteModel Client
	// gazebo_msgs::DeleteModel delete_srv;
	// delete_srv.request.model_name = "ball2";
	// deleteModel_client.call(delete_srv);

	return 0;
}