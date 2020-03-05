#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  if (argc != 2)
  {
    ROS_INFO("usage: add_two_ints_client bool");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("bool_service");
  std_srvs::SetBool srv;
  std::string action(argv[1]);
  if(action == "true"){
      srv.request.data = true;
  }else
  {
      srv.request.data = false;
  }
  
  if (client.call(srv))
  {
    ROS_INFO("aaa: %s", srv.response.message.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}