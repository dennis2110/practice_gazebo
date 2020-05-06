#include "ros/ros.h"
#include "arduserial.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ardu_lib");
    ros::NodeHandle n;

    ArduSerial homeArdu("/dev/ttyACM0", 6);
    homeArdu.init();
    homeArdu.readdata[0] = 0x7E;
    homeArdu.readdata[1] = 0x7E;
    std::cout << "123" << std::endl;

    ros::Duration(2.0).sleep();
    
    uint8_t code[5];
    code[0] = 0x33;
    code[1] = 0x20;
    code[2] = 0x30;
    code[3] = 0x20;
    code[4] = 0x30;
    //homeArdu.write(code,5);
    
    
    
    //homeArdu.read_dick(&homeArdu.readdata[0], 3);
    //std::cout << "data[0]: " << homeArdu.readdata[0] << std::endl;
    //std::cout << "data[1]: " << homeArdu.readdata[1] << std::endl;
    std::cout << "in loop" << std::endl;
    ros::Rate loop_rate(10);
    std::string ardu_code("1 16 512");
    std::string ardu_code_2("1 16 450");
    std::string ardu_code_3("2 0 0");
    //homeArdu.write_dick(ardu_code);
    ros::Duration(0.1).sleep();
    int count = 0;
    while (ros::ok())
    {
        //write cmd to arduino
        //string "mode motor_id cmd"
        //
        count ++;
        std::cout << "loop " << count << std::endl;
        //homeArdu.write_dick(ardu_code);
        //std::cout << "aaa"<< std::endl;

        if(count > 1 && count < 10){
            homeArdu.write_dick(ardu_code);
        }else if(count > 30 && count < 40){
            homeArdu.write_dick(ardu_code_2);
        }else if(count > 50 && count < 60){
            homeArdu.write_dick(ardu_code_3);
        }
        //homeArdu.read_dick(&homeArdu.readdata[0], 3);
        //std::cout << "data[0]: " << homeArdu.readdata[0] << std::endl;
        //std::cout << "data[1]: " << homeArdu.readdata[1] << std::endl;
/*
        if(homeArdu.readdata[0] == 0x31){//"1"
            std::cout << "slide homing" << std::endl;
        }
        if(homeArdu.readdata[1] == 0x30){//"0"
            std::cout << "joint1 homing" << std::endl;
        }*/
    



        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
