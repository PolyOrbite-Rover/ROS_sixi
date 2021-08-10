#include "sixi_teleop_joy/sixi_teleop_competition.h"

// DEFINITION OF CONSTANTS (adjust if needed)
const int TeleopSixi::publish_rate_hz_            = 10;
const int TeleopSixi::n_joints_                   = 6;


TeleopSixi::TeleopSixi(bool verbose): verbose_(verbose)
{
  control_pub_ = nh_.advertise<sixi_robot::CmdControl>("sixi_robot/cmd_control", 1);
  joy_sub_ = nh_.subscribe<sixi_robot::CmdWeb>("teleop_sixi", 10, &TeleopSixi::callback);
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


void TeleopSixi::callback(const sixi_robot::CmdWeb& joy)
{
  std::vector<float> null_vec = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  if (joy.zero == 0.0 && joy.one == 0.0 && joy.two == 0.0 && joy.three == 0.0 && joy.four == 0.0 && joy.five == 0.0){
    joy_at_rest_ = true;
    ROS_ERROR("joy at rest!!!!");
    current_joy_pos_ = {};
  }
  else{
    joy_at_rest_ = false;
    current_joy_pos_[0] = joy.zero;
    current_joy_pos_[1] = joy.one;
    current_joy_pos_[2] = joy.two;
    current_joy_pos_[3] = joy.three;
    current_joy_pos_[4] = joy.four;
    current_joy_pos_[5] = joy.five;
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
