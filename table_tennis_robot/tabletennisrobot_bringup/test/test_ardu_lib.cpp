#include "ros/ros.h"
#include "arduserial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ardu_lib");
    ros::NodeHandle n;

    ArduSerial homeArdu("/dev/ttyACM0", 6);
    homeArdu.init();
    homeArdu.readdata[0] = 0;
    homeArdu.readdata[1] = 0;
    std::cout << "123" << std::endl;

    homeArdu.read_dick(&homeArdu.readdata[0], 3);
    std::cout << "data[0]: " << homeArdu.readdata[0] << std::endl;
    std::cout << "data[1]: " << homeArdu.readdata[1] << std::endl;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        homeArdu.read_dick(&homeArdu.readdata[0], 3);
        std::cout << "data[0]: " << homeArdu.readdata[0] << std::endl;
        std::cout << "data[1]: " << homeArdu.readdata[1] << std::endl;

        if(homeArdu.readdata[0] == 0x31){//"1"
            std::cout << "slide homing" << std::endl;
        }
        if(homeArdu.readdata[1] == 0x30){//"0"
            std::cout << "joint1 homing" << std::endl;
        }
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
