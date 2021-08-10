#ifndef SIXI_TELEOP_COMPETITION_H
#define SIXI_TELEOP_COMPETITION_H

#define MODE_JOINTS 0

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "sixi_robot/CmdControl.h"
#include "joint_states_listener/ReturnJointPos.h"


class TeleopSixi
{
public:
  TeleopSixi(bool verbose=false);
  bool isJoyAtRest();
  void publishCmd();
  int getPublishRate() const;

private:
  // Private methods
  void callback(const sensor_msgs::Joy::ConstPtr& joy);
  // Private attributes
  static const std::vector<int> control_axes_;
  // static const std::vector<int> control_buttons_;
  static const int publish_rate_hz_, n_joints_;
  std::vector<float> current_joy_pos_;
  bool joy_at_rest_ = true;
  bool verbose_;
  int control_mode_ = MODE_JOINTS;
  ros::NodeHandle nh_;
  ros::Publisher control_pub_;
  ros::Subscriber joy_sub_;
};

#endif
