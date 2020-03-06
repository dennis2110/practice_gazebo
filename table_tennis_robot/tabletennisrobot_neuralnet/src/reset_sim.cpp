#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reset_sim");
	ros::NodeHandle n;

	ros::ServiceClient reset_sim_client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
	ros::Publisher unified_cmd_pub =
		n.advertise<std_msgs::Float64MultiArray>("/unified_joint_cmd", 10);

	std_srvs::Empty srv;
	std_msgs::Float64MultiArray unified_cmd;

	unified_cmd.data.push_back(0.4735);
	unified_cmd.data.push_back(-1.57); //1.57);
	unified_cmd.data.push_back(2.2);   // 2.2 //1.1
	unified_cmd.data.push_back(2.9);   // 2.9 //4.2
	unified_cmd.data.push_back(2.2);   // 2.2 //3.6
	unified_cmd.data.push_back(0.0);
	if (reset_sim_client.call(srv))
	{
		ros::Duration(0.5).sleep();
		unified_cmd_pub.publish(unified_cmd);
	}

	return 0;
}