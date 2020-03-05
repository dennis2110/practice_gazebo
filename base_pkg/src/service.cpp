#include "ros/ros.h"
#include "std_srvs/SetBool.h"
bool bool_service(std_srvs::SetBool::Request &req,
                            std_srvs::SetBool::Response &res)
{
    res.success = req.data;
    if(res.success){
        res.message = "result true";
    }else{
        res.message = "result false";
    }
    std::cout << res.message.c_str() << std::endl;
    return true;
}
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("bool_service", bool_service);
  ROS_INFO("Ready to service");
  ros::spin();
  return 0;
}