#include <exception>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sixi_robot/CmdPosJoints.h"

#define N_JOINTS 6

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > ArmControllerClient;

class RobotArm
{
public:
  // CTOR & DTOR
  RobotArm();
  ~RobotArm();
  // Methods
  void cmdCallback(sixi_robot::CmdPosJoints cmd_joint_pos);
private:
  ros::NodeHandle nh_;
  ros::Subscriber joints_sub_;
};

RobotArm::RobotArm()
{
  joints_sub_ = nh_.subscribe<sixi_robot::CmdPosJoints>("sixi_robot/cmd_pos_joints", 10, &RobotArm::cmdCallback, this);
  if (joints_sub_)
    ROS_WARN("subscrisber_test: cmd_pos_joints subscriber created!");
  else
    ROS_ERROR("subscrisber_test: cmd_pos_joints subscriber NOT valid!");
  // }
}

RobotArm::~RobotArm()
{
}

void RobotArm::cmdCallback(sixi_robot::CmdPosJoints cmd_joint_pos)
{
  ROS_WARN("Yepp!");
  //our goal variable
  ROS_WARN("%f", cmd_joint_pos.joint_pos[0]);
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  RobotArm arm;
  // Start the trajectory
  // arm.goTo(arm.testTrajectory());
  // Wait for trajectory completion
  ros::spin();
  // while(ros::ok()){}
  return 0;
}
