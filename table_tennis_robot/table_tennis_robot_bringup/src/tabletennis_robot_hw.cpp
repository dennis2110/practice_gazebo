#include "tabletennis_robot_hw.h"

TabletennisRobot::TabletennisRobot(){
    slide_rail_cmd = 0;
    slide_rail_pos = 0;
    slide_rail_vel = 0;
    slide_rail_eff = 0;

    for (int i = 0; i < 4; i++)
    {
        arm_cmd[i]=0;
        arm_pos[i]=0;
        arm_vel[i]=0;
        arm_eff[i]=0;
    }
    // connect and register the joint state interface
    hardware_interface::JointStateHandle slide_rail_state_handle(
        "slide_rail_joint", &slide_rail_pos, &slide_rail_vel, &slide_rail_eff);
    jnt_state_interface.registerHandle(slide_rail_state_handle);

    hardware_interface::JointStateHandle base_arm_handle(
        "base_arm_joint", &arm_pos[0], &arm_vel[0], &arm_eff[0]);
    jnt_state_interface.registerHandle(base_arm_handle);

    hardware_interface::JointStateHandle arm_2_handle(
        "arm_2_joint", &arm_pos[1], &arm_vel[1], &arm_eff[1]);
    jnt_state_interface.registerHandle(arm_2_handle);

    hardware_interface::JointStateHandle arm_3_handle(
        "arm_3_joint", &arm_pos[2], &arm_vel[2], &arm_eff[2]);
    jnt_state_interface.registerHandle(arm_3_handle);

    hardware_interface::JointStateHandle arm_4_handle(
        "arm_4_joint", &arm_pos[3], &arm_vel[3], &arm_eff[3]);
    jnt_state_interface.registerHandle(arm_4_handle);

    registerInterface(&jnt_state_interface);

    //arm -> connect and register the joint position interface
    hardware_interface::JointHandle slide_rail_pos_cmd_handle(
        jnt_state_interface.getHandle("slide_rail_joint"), &slide_rail_cmd);
    jnt_eff_interface.registerHandle(slide_rail_pos_cmd_handle);

    hardware_interface::JointHandle base_arm_pos_cmd_handle(
        jnt_state_interface.getHandle("base_arm_joint"), &arm_cmd[0]);
    jnt_eff_interface.registerHandle(base_arm_pos_cmd_handle);

    hardware_interface::JointHandle arm_2_pos_cmd_handle(
        jnt_state_interface.getHandle("arm_2_joint"), &arm_cmd[1]);
    jnt_eff_interface.registerHandle(arm_2_pos_cmd_handle);

    hardware_interface::JointHandle arm_3_pos_cmd_handle(
        jnt_state_interface.getHandle("arm_3_joint"), &arm_cmd[2]);
    jnt_eff_interface.registerHandle(arm_3_pos_cmd_handle);

    hardware_interface::JointHandle arm_4_pos_cmd_handle(
        jnt_state_interface.getHandle("arm_4_joint"), &arm_cmd[3]);
    jnt_eff_interface.registerHandle(arm_4_pos_cmd_handle);

    registerInterface(&jnt_eff_interface);
    ROS_INFO("Tabletennis Robot HW creat");

}

TabletennisRobot::~TabletennisRobot(){

}

void TabletennisRobot::init(ros::NodeHandle* node){
    ROS_INFO("Tabletennis Robot init");
}

void TabletennisRobot::read(ros::Time time, ros::Duration period){

}

void TabletennisRobot::write(ros::Time time, ros::Duration period){
    ROS_INFO("joint 0~4 : %4.3f %4.3f %4.3f %4.3f %4.3f",
        slide_rail_cmd,arm_cmd[0],arm_cmd[1],arm_cmd[2],arm_cmd[3]);
}
