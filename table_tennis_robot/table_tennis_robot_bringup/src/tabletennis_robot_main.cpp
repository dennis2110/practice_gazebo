#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include "tabletennis_robot_hw.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tabletennis_robot_main");
  ros::NodeHandle node;
  ros::NodeHandle node2;
  ros::CallbackQueue queue;
  node.setCallbackQueue(&queue);

  TabletennisRobot robot;
  robot.init(&node2);
  controller_manager::ControllerManager cm(&robot,node);


  ros::AsyncSpinner spinner(4,&queue);
  spinner.start();

  ros::Rate r(1.0 / robot.getPeriod().toSec());
  while(ros::ok())
  {
    ros::Time now = robot.getTime();
    ros::Duration dt = robot.getPeriod();

    robot.read(now, dt);
    cm.update(now, dt);
    robot.write(now, dt);
    ros::spinOnce();
    r.sleep();
  }
  spinner.stop();

  return 0;
}