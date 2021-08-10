#include "sixi_teleop_joy/sixi_teleop_joy.h"

// DEFINITION OF CONSTANTS (adjust if needed)
const std::vector<int> TeleopSixi::control_axes_  = {0, 1, 4, -1, 7, 6};
const int TeleopSixi::publish_rate_hz_            = 10;
const int TeleopSixi::n_joints_                   = 6;


TeleopSixi::TeleopSixi(bool verbose): verbose_(verbose)
{
  control_pub_ = nh_.advertise<sixi_robot::CmdControl>("sixi_robot/cmd_control", 1);
  joy_sub_ = nh_.subscribe("joy", 10, &TeleopSixi::joyCallback, this);
  current_joy_pos_.resize(n_joints_);
}


bool TeleopSixi::isJoyAtRest()
{
  return joy_at_rest_;
}


int TeleopSixi::getPublishRate() const
{
  return publish_rate_hz_;
}


void TeleopSixi::joyCallback(sensor_msgs::Joy::ConstPtr joy)
{
  std::vector<float> null_vec_axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<int> null_vec_buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (joy->axes == null_vec_axes & joy->buttons == null_vec_buttons){
    joy_at_rest_ = true;
    ROS_ERROR("joy at rest!!!!");
    current_joy_pos_ = {};
  }
  else{
    joy_at_rest_ = false;
    current_joy_pos_[0] = joy->axes[control_axes_[0]];
    current_joy_pos_[1] = joy->axes[control_axes_[1]];
    current_joy_pos_[2] = joy->axes[control_axes_[2]];
    current_joy_pos_[3] = joy->buttons[5] - joy->buttons[4];
    current_joy_pos_[4] = joy->axes[control_axes_[4]];
    current_joy_pos_[5] = joy->axes[control_axes_[5]];
    // ROS_ERROR("(%f, %f, %f)",current_joy_pos_[0] ,current_joy_pos_[1] ,current_joy_pos_[2] );
    ROS_WARN("Not working...");
  }
}


void TeleopSixi::publishCmd()
{
  sixi_robot::CmdControl cmd_control;
  cmd_control.pos.resize(n_joints_);

  for (int i=0; i < n_joints_; i++)
    cmd_control.pos[i] = current_joy_pos_[i];
  if (verbose_){
    for (int i=0; i < n_joints_; i++)
      ROS_WARN("%f", cmd_control.pos[i]);
    ROS_WARN("---------------------------");
  }
  cmd_control.mode = MODE_JOINTS;
  cmd_control.header.stamp = ros::Time::now();
  control_pub_.publish(cmd_control);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_sixi");
  TeleopSixi teleop_sixi(true);

  ros::Rate rate(teleop_sixi.getPublishRate());

  while(ros::ok()){
    ros::spinOnce();
    if (!teleop_sixi.isJoyAtRest())
      teleop_sixi.publishCmd();
    rate.sleep();
  }
}
