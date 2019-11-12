#ifndef TABLETENNIS_ROBOT_HW_H
#define TABLETENNIS_ROBOT_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class TabletennisRobot : public hardware_interface::RobotHW
{
public:
    TabletennisRobot();
    ~TabletennisRobot();
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.001);}

    void init(ros::NodeHandle* node);
    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);
private:

public:

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    //hardware_interface::VelocityJointInterface jnt_vel_interface;
    //hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    double slide_rail_cmd;
    double slide_rail_pos;
    double slide_rail_vel;
    double slide_rail_eff;

    double arm_cmd[4];
    double arm_pos[4];
    double arm_vel[4];
    double arm_eff[4];
};

#endif //TABLETENNIS_ROBOT_HW_H
