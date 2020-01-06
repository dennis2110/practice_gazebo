#include <ros/ros.h>
#include "tabletennis_robot_epos2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tabletennis_robot_bringup");
    ros::NodeHandle nh;


    TabletennisRobot::EPOS2 robot;

    if(robot.init(nh))
        ROS_INFO("robot init success");
    else
        ROS_ERROR("robot init fail");

    //30~40Hz if pub 1 joint pos and vel
    //20Hz if pub 5 joint pos and vel
    ros::Rate loop_rate(150);
    while (ros::ok())
    {
        ros::Time start_update_time = ros::Time::now();

        ros::spinOnce();
        robot.update();
        loop_rate.sleep();

        ros::Duration time_dura = ros::Time::now() - start_update_time;
        ROS_INFO("                   main loop: %4.3f Hz", 1.0/time_dura.toSec());
    }
    
    
    robot.close_device();

    return 0;
} 
