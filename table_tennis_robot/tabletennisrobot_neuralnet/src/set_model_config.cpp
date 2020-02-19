#include "ros/ros.h"
#include <vector>
#include "gazebo_msgs/SetModelConfiguration.h"
#include "std_msgs/Float64MultiArray.h"


int main(int argc, char**argv)
{
    ros::init(argc, argv,"set_model_config");
    ros::NodeHandle n;

    ros::ServiceClient reset_sim_client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::Publisher unified_cmd_pub =
      n.advertise<std_msgs::Float64MultiArray>("/unified_joint_cmd", 10);
    
    gazebo_msgs::SetModelConfiguration config_srv;
    std_msgs::Float64MultiArray unified_cmd;
    unified_cmd.data.push_back(0.4735);
    unified_cmd.data.push_back(-1.57);//1.57);
    unified_cmd.data.push_back(2.2); // 2.2 //1.1
    unified_cmd.data.push_back(2.9); // 2.9 //4.2
    unified_cmd.data.push_back(2.2); // 2.2 //3.6
    unified_cmd.data.push_back(0.0);

    config_srv.request.model_name = "TTbot";
    config_srv.request.urdf_param_name = "robot_description";
    config_srv.request.joint_names.clear();
    config_srv.request.joint_names.push_back("joint_0");
    config_srv.request.joint_names.push_back("joint_1");
    config_srv.request.joint_names.push_back("joint_2");
    config_srv.request.joint_names.push_back("joint_3");
    config_srv.request.joint_names.push_back("joint_4");
    config_srv.request.joint_names.push_back("joint_5");
    config_srv.request.joint_positions.clear();
    config_srv.request.joint_positions.push_back(0.5);
    config_srv.request.joint_positions.push_back(0.0);
    config_srv.request.joint_positions.push_back(0.0);
    config_srv.request.joint_positions.push_back(0.0);
    config_srv.request.joint_positions.push_back(0.0);
    config_srv.request.joint_positions.push_back(0.0);

    if(reset_sim_client.call(config_srv))
    {
        ROS_INFO("reset success");
        ros::Duration(0.5).sleep();
        unified_cmd_pub.publish(unified_cmd);  
    }else
    {
        ROS_INFO("reset fail");
    }
    //reset_sim_client.call(config_srv);
    return 0;
}