#include "ros/ros.h"
#include "joint_states_listener/ReturnJointPos.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_client");
  if (argc == 0)
  {
    ROS_ERROR("usage: listener_client name1 ... nameN");
    return 1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<joint_states_listener::ReturnJointPos>("return_joint_pos");
  joint_states_listener::ReturnJointPos srv;
  if (client.call(srv))
  {
    for (auto i = srv.response.position.begin(); i != srv.response.position.end(); i++)
    ROS_INFO("%f", *i);
  }
  else
  {
    ROS_ERROR("Failed to call service joint_states_listener");
    return 1;
  }
  return 0;
}
