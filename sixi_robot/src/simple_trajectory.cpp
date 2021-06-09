#include <exception>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sixi_robot/CmdPosJoints.h"

#define NODE_NAME "------ trajectory_controller.cpp ------"
#define MAX_SERVER_ITERATIONS 3
#define N_JOINTS 6
#define TIME_TO_REACH_SEC 0.2


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > ArmControllerClient;

class RobotArm
{
public:
  // CTOR & DTOR
  RobotArm();
  ~RobotArm();
  // Methods
  void goTo(control_msgs::FollowJointTrajectoryGoal goal);
  void initGoal(control_msgs::FollowJointTrajectoryGoal& goal);
  void cmdCallback(sixi_robot::CmdPosJoints cmd_joint_pos);
  bool initializationIsGood();
  control_msgs::FollowJointTrajectoryGoal testTrajectory();
  actionlib::SimpleClientGoalState getState();
private:
  bool initialization_successfull_ = false;
  ros::NodeHandle nh_;
  ros::Subscriber joints_sub_;
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  ArmControllerClient* arm_controller_client_;
};

RobotArm::RobotArm()
{
  // Initialize the action client and wait for action server to come up
  arm_controller_client_ = new ArmControllerClient("arm_controller/follow_joint_trajectory", true);
  int iteration = 0;
  while(!arm_controller_client_->waitForServer(ros::Duration(3.0)) && iteration < MAX_SERVER_ITERATIONS){
    ROS_WARN("%s\nWaiting for the joint_trajectory action server to start...", NODE_NAME);
    iteration++;
  }
  if (iteration == MAX_SERVER_ITERATIONS){
    ROS_ERROR("%s\njoint_trajectory action server not avaible!", NODE_NAME);
  }
  else{
    ROS_WARN("%s\nAction server started successfully!", NODE_NAME);
    joints_sub_ = nh_.subscribe<sixi_robot::CmdPosJoints>("sixi_robot/cmd_pos_joints", 10, &RobotArm::cmdCallback, this);
    if (joints_sub_){
      ROS_WARN("%s\ncmd_pos_joints subscriber created successfully!", NODE_NAME);
      initialization_successfull_ = true;
    }
    else
      ROS_ERROR("%s\ncmd_pos_joints subscriber NOT valid!", NODE_NAME);
  }
}

RobotArm::~RobotArm()
{
  delete arm_controller_client_; // Clean up the action client
}

bool RobotArm::initializationIsGood()
{
  return initialization_successfull_;
}

void RobotArm::cmdCallback(sixi_robot::CmdPosJoints cmd_joint_pos)
{
  ROS_WARN("%s\nCommand received: %f %f", NODE_NAME, cmd_joint_pos.joint_pos[0], cmd_joint_pos.joint_pos[1]);
  //our goal variable
  control_msgs::FollowJointTrajectoryGoal goal;
  initGoal(goal);
  // Positions
  goal.trajectory.points[0].positions.resize(N_JOINTS);
  goal.trajectory.points[0].positions = cmd_joint_pos.joint_pos;
  // To be reached 0.5 second after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration(TIME_TO_REACH_SEC);

  // Sends the command to start a given trajectory as fast as possible (now)
  goal.trajectory.header.stamp = ros::Time::now();
  arm_controller_client_->sendGoal(goal);
}

void RobotArm::goTo(control_msgs::FollowJointTrajectoryGoal goal)
{
  // Sends the command to start a given trajectory as fast as possible (now)
  goal.trajectory.header.stamp = ros::Time::now();
  arm_controller_client_->sendGoal(goal);
}

void RobotArm::initGoal(control_msgs::FollowJointTrajectoryGoal& goal)
{
  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("j0_shoulder");
  goal.trajectory.joint_names.push_back("j1_bicep");
  goal.trajectory.joint_names.push_back("j2_forearm");
  goal.trajectory.joint_names.push_back("j3_tuningfork");
  goal.trajectory.joint_names.push_back("j4_picassobox");
  goal.trajectory.joint_names.push_back("j5_hand");
  // We will have two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].velocities.resize(N_JOINTS);
  goal.trajectory.points[0].velocities = {}; // Initialize with all zeros
}
  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
control_msgs::FollowJointTrajectoryGoal RobotArm::testTrajectory()
{
  //our goal variable
  control_msgs::FollowJointTrajectoryGoal goal;
  initGoal(goal);
  // First trajectory point
  // Positions
  int ind = 0;
  goal.trajectory.points[ind].positions.resize(N_JOINTS);
  goal.trajectory.points[ind].positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // To be reached 0.5 second after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration(0.5);

  // Second trajectory point
  // Positions
  ind++;
  goal.trajectory.points[ind].positions.resize(N_JOINTS);
  goal.trajectory.points[ind].positions = {1.2, 0.5, 0.0, 0.0, 0.0, 0.0};
  // To be reached 1 second after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);
  //we are done; return the goal
  return goal;
}

actionlib::SimpleClientGoalState RobotArm::getState()
{
  // Returns the current state of the action
  return arm_controller_client_->getState();
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  RobotArm arm;
  if (arm.initializationIsGood())
    ros::spin();
  else
    ros::shutdown();
  return 0;
}
