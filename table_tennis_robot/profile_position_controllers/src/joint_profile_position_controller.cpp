/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Dave Coleman
 Contributors: Jonathan Bohren, Wim Meeussen, Vijay Pradeep
 Desc: Velocity-based position controller using basic PID loop
*/

#include <ttbot_velocity_controllers/joint_profile_position_controller.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace ttbot_velocity_controllers {

JointProfilePositionController::JointProfilePositionController()
  : loop_count_(0) , last_velocity(0.0)
{}

JointProfilePositionController::~JointProfilePositionController()
{
  sub_command_.shutdown();
}

bool JointProfilePositionController::init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n)
{
  // Get joint name from parameter server
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }

  // Get ppm_param name from parameter server
  ros::NodeHandle nh(ros::NodeHandle(n,"ppm_param"));
  if (!nh.getParam("vel", ppm_params_struct_.velocity_))
  {
    ROS_ERROR("No vel param.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }else{
    std::cout << joint_name << " -> ppm_vel: " << ppm_params_struct_.velocity_ << std::endl;
  }
  if (!nh.getParam("acc", ppm_params_struct_.acceleration_))
  {
    ROS_ERROR("No acc param.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }else{
    std::cout << joint_name << " -> ppm_acc: " << ppm_params_struct_.acceleration_ << std::endl;
  }
  if (!nh.getParam("dec", ppm_params_struct_.deceleration_))
  {
    ROS_ERROR("No dec param.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }else{
    std::cout << joint_name << " -> ppm_dec: " << ppm_params_struct_.deceleration_ << std::endl;
  }



  // Load PID Controller using gains set on parameter server
  //if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
  //  return false;

  // Start realtime state publisher
  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

  // Start command subscriber
  sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &JointProfilePositionController::setCommandCB, this);

  // Get joint handle from hardware interface
  joint_ = robot->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", n))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

/*void JointProfilePositionController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
{
  pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
}

void JointProfilePositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

void JointProfilePositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void JointProfilePositionController::printDebug()
{
  pid_controller_.printValues();
}*/

std::string JointProfilePositionController::getJointName()
{
  return joint_.getName();
}

double JointProfilePositionController::getPosition()
{
  return joint_.getPosition();
}

// Set the joint position command
void JointProfilePositionController::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false; // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

// Set the joint position command with a velocity command as well
void JointProfilePositionController::setCommand(double pos_command, double vel_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.velocity_ = vel_command;
  command_struct_.has_velocity_ = true;

  command_.writeFromNonRT(command_struct_);
}

void JointProfilePositionController::starting(const ros::Time& time)
{
  double pos_command = joint_.getPosition();

  // Make sure joint is within limits if applicable
  enforceJointLimits(pos_command);

  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ = false;

  command_.initRT(command_struct_);

  //pid_controller_.reset();
}

void JointProfilePositionController::update(const ros::Time& time, const ros::Duration& period)
{
  command_struct_ = *(command_.readFromRT());
  double command_position = command_struct_.position_;
  //double command_velocity = command_struct_.velocity_;
  //bool has_velocity_ =  command_struct_.has_velocity_;

  double error;
  double commanded_velocity;
  double need_distance;

  double current_position = joint_.getPosition();
  double current_velocity = joint_.getVelocity();
  

  // Make sure joint is within limits if applicable
  enforceJointLimits(command_position);
  //std::string joint_zero = "joint_0";
  //if(joint_.getName() == joint_zero){
    
  error = command_position - current_position;
  need_distance = last_velocity*last_velocity/(2*ppm_params_struct_.acceleration_);

  if(abs(error) > 0.0001){
    if(abs(error) < need_distance){
      motion_state = uniformDec;
    }else if(last_velocity < ppm_params_struct_.velocity_){
      motion_state = uniformAcc;
    }else{
      motion_state = constantVel;
    }
  }else{
    motion_state = stop;
  }
    

    
    



    
  //std::cout << "aaa vel command is :" << commanded_velocity  << " last_velocity:" <<last_velocity << std::endl;
    
  switch (motion_state)
  {
  case stop:
    /* code */
    commanded_velocity = 0.0;
    
    break;
  
  case constantVel:
    /* code */
    commanded_velocity = ppm_params_struct_.velocity_;
    
    break;

  case uniformAcc:
    /* code */
    commanded_velocity = last_velocity + ( ppm_params_struct_.acceleration_ * period.toSec());
    
    break;

  case uniformDec:
    /* code */
    commanded_velocity = last_velocity - ( ppm_params_struct_.acceleration_ * period.toSec());
    
    break;
  
  default:
    commanded_velocity = 0.0;
    break;
  }
  
  commandVelocityLimits(commanded_velocity);
  last_velocity = commanded_velocity;
  if(error < 0.0){
    commanded_velocity = -commanded_velocity;
  }
  joint_.setCommand(commanded_velocity);

  // publish state
  /*if (loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_position;
      controller_state_publisher_->msg_.process_value = current_position;
      controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = commanded_velocity;

      double dummy;
      bool antiwindup;
      /*getGains(controller_state_publisher_->msg_.p,
        controller_state_publisher_->msg_.i,
        controller_state_publisher_->msg_.d,
        controller_state_publisher_->msg_.i_clamp,
        dummy,
        antiwindup);
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;*/
}

void JointProfilePositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void JointProfilePositionController::enforceJointLimits(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->upper ) // above upper limnit
    {
      command = joint_urdf_->limits->upper;
    }
    else if( command < joint_urdf_->limits->lower ) // below lower limit
    {
      command = joint_urdf_->limits->lower;
    }
  }
}

void JointProfilePositionController::commandVelocityLimits(double &command)
{
  if(command > joint_urdf_->limits->velocity)
  {
    command = joint_urdf_->limits->velocity;
  }
  else if(command < -joint_urdf_->limits->velocity)
  {
    command = -joint_urdf_->limits->velocity;
  }
}

} // namespace

PLUGINLIB_EXPORT_CLASS( ttbot_velocity_controllers::JointProfilePositionController, controller_interface::ControllerBase)
