#!/usr/bin/env python

import rospy
from sixi_robot.msg import CmdPosJoints, CmdControl
from joint_states_listener.srv import ReturnJointPos
import numpy as np



class ControllerAnalysis:

  def __init__(self):
    self.sub = rospy.Subscriber("sixi_robot/cmd_pos_joints", CmdPosJoints, self.cmd_callback)
    # self.pub = rospy.Publisher("sixi_robot/cmd_pos_joints", CmdPosJoints, queue_size=2)
    self.pos_client = rospy.ServiceProxy("return_joint_pos", ReturnJointPos)
    # Kinematic parameters
    self.d1 = 194.45
    self.a2 = 357.96
    self.a3 = 64.25
    self.d4 = 387.05
    self.d6 = 0
    self.t_a3_a4 = np.arctan2(self.a3, self.d4)
    self.D_H_table = np.array([ [0      , np.pi/2 , self.d1 , np.pi/2 ],
                                [self.a2, 0       , 0       , np.pi/2 ],
                                [self.a3, np.pi/2 , 0       , 0       ],
                                [0      , -np.pi/2, self.d4 , 0       ],
                                [0      , np.pi/2 , 0       , 0       ],
                                [0      , 0       , self.d6 , 0       ]])
    self.ZERO_MARGIN = 1e-8

  def cmd_callback(self, control_msg):
    try:
      srv = self.pos_client()
    except rospy.ServiceException as e:
      rospy.logerr(f"Joint position service call failed with error: {e}")
    else:
      current_Theta = np.array(srv.position)
      cmd_X = self.DK(control_msg.joint_pos)
      current_X = self.DK(current_Theta)
      
      with np.printoptions(precision=2, suppress=True):
        print(f"cmd_X = {cmd_X}")
        print(f"current_X = {current_X}")
      


  def DK(self, Theta, verbose=False):
    """ Compute the Direct Kinematic (DK) for Sixi 2 robot

    Args:
        Theta (numpy.array): The angles of the joints given as [t1, t2, t3, t4, t5, t6]
        verbose (bool, optional): Verbosity. Defaults to False.

    Returns:
        numpy.array: Cartesian coordinates given as [x, y, z, alpha, beta, gamma]
    """
    # Compute the homogeneous transformation matrix T
    A = []
    for i, t in enumerate(Theta):
      t += self.D_H_table[i, 3]
      A.append(np.array([[np.cos(t), -np.sin(t)*np.cos(self.D_H_table[i,1]), np.sin(t)*np.sin(self.D_H_table[i,1]), self.D_H_table[i,0]*np.cos(t)],
                         [np.sin(t), np.cos(t)*np.cos(self.D_H_table[i,1]), -np.cos(t)*np.sin(self.D_H_table[i,1]), self.D_H_table[i,0]*np.sin(t)],
                         [0        , np.sin(self.D_H_table[i,1])          , np.cos(self.D_H_table[i,1])           , self.D_H_table[i,2]],
                         [0        , 0                               , 0                                , 1]]))

    T = A[0] @ A[1] @ A[2] @ A[3] @ A[4] @ A[5]

    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    # Compute Euler angles for the mobile ZYZ convention
    beta = np.arccos(T[2, 2])
    if np.abs(np.abs(T[2, 2]) - 1) < self.ZERO_MARGIN:
      alpha = 0
      gamma = np.arctan2(T[1, 0], T[1, 1])
    else:
      alpha = np.arctan2(T[1, 2], T[0, 2])
      gamma = np.arctan2(T[2, 1], -T[2, 0])

    X = np.array([x, y, z, alpha, beta, gamma])
    if verbose:
      with np.printoptions(precision=2, suppress=True):
        print(f"[x, y, z, alpha, beta, gamma] = \n{X}")
    return X


if __name__ == '__main__':
  rospy.init_node('Controller_Analysis')
  ControllerAnalysis()
  rospy.spin()
