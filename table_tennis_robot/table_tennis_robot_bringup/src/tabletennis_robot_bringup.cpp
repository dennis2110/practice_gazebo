#include <ros/ros.h>
#include "tabletennis_robot_epos2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tabletennis_robot_bringup");
    ros::NodeHandle nh;


    TabletennisRobot::EPOS2 robot;

    if(robot.init(nh))
        ROS_INFO("device open success");
    else
        ROS_ERROR("device open fail");


    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
    
        loop_rate.sleep();
    }
    
    robot.close_device();

    return 0;
} 
