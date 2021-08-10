#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from sixi_robot.msg import CmdPosJoints, CmdControl
from joint_states_listener.srv import ReturnJointPos
import numpy as np


class HardwareInterface:
  def __init__(self):
    self.sub_IK = rospy.Subscriber("sixi_robot/cmd_pos_joints", CmdPosJoints, self.IK_callback)
    self.pub_to_arduino = rospy.Publisher("arduino_motors", Float32MultiArray, queue_size=2)
    self.sub_to_arduino = rospy.Subscriber("arduino_sensors", Float32MultiArray, self.arduino_callback)
    self.pub_to_joint_state = rospy.Publisher("joint_states", JointState, queue_size=2)
   
  def IK_callback(self, cmd_msg):
    print("msg received!")
    arduino_cmd = Float32MultiArray()
    arduino_cmd.data = np.rad2deg(cmd_msg.joint_pos)
    self.pub_to_arduino.publish(arduino_cmd)

  def arduino_callback(self, sensors_msg):
    print("msg received!")
    joint_state_msg = JointState()
    joint_state_msg.name = ["j0_shoulder", "j1_bicep", "j2_forearm", "j3_tuningfork", "j4_picassobox", "j5_hand"]
    joint_state_msg.position = np.deg2rad(sensors_msg.data)
    self.pub_to_joint_state.publish(joint_state_msg)


if __name__ == '__main__':
  rospy.init_node('Hardware_Interface')
  HardwareInterface()
  rospy.spin()
