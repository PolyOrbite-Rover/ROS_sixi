#!/usr/bin/env python

import rospy
from sixi_robot.msg import CmdPosJoints, CmdControl
from joint_states_listener.srv import ReturnJointPos
import numpy as np



class InverseKinematic:

  def __init__(self):
    self.sub = rospy.Subscriber("sixi_robot/cmd_control", CmdControl, self.cmd_callback)
    self.pub = rospy.Publisher("sixi_robot/cmd_pos_joints", CmdPosJoints, queue_size=2)
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
    self.mvt_scale_and_direction_XYZ = np.array([-20.0, 20.0, 20.0, 0.1, 0.1, 0.1])
    self.mvt_scale_and_direction_JOINTS = 0.2* np.array([1, -1, -1, 1, 1, 1])


  def cmd_callback(self, control_msg):
    try:
      srv = self.pos_client()
    except rospy.ServiceException as e:
      rospy.logerr(f"Joint position service call failed with error: {e}")
    else:
      current_Theta = np.array(srv.position)

      if (control_msg.mode == 1): # XYZ mode
        movement = self.mvt_scale_and_direction_XYZ * np.array(control_msg.pos)
        current_X = self.DK(current_Theta)
        next_X = current_X + movement
        next_Theta = self.IK(next_X)
        with np.printoptions(precision=2, suppress=True):
          print(f"current_Theta = {current_Theta}")
          print(f"{current_X} + {movement} = {next_X}")
          print(f"next_Theta = {next_Theta}")
      else: # Joints mode
        movement = self.mvt_scale_and_direction_JOINTS * np.array(control_msg.pos)
        next_Theta = current_Theta + movement
        with np.printoptions(precision=2, suppress=True):
          print(f"{current_Theta} + {movement} = {next_Theta}")

      joints_msg = CmdPosJoints()
      joints_msg.joint_pos = next_Theta
      self.pub.publish(joints_msg)


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

  def IK(self, X, verbose=False):
    """ Compute the Inverse Kinematic (IK) for Sixi 2 robot

    Args:
        X (numpy.array)         : Desired cartesian coordinates given as [x, y, z, alpha, beta, gamma]
        verbose (bool, optional): Verbosity. Defaults to False.

    Returns:
        numpy.array: The angles of the joints given as [t1, t2, t3, t4, t5, t6]
    """
    # Position (t1, t2, t3)
    x, y, z = X[:3]
    t1 = - np.arctan2(x, y)

    l1 = self.a2
    l2 = (self.a3**2 + self.d4**2)**0.5
    r = (x**2 + y**2)**0.5
    z  = z - self.d1
    k1 = (-l2**2 + l1**2 + r**2 + z**2)/(2*r)
    k2 = z/r
    p_z = (k1*k2 + ((k1*k2)**2 - (1 + k2**2)*(k1**2 - l1**2))**0.5)/(1+k2**2)
    p_r = k1 - k2*p_z

    try:
      t2 = - np.arctan2(p_r, p_z)
      t3 = -t2 + np.arctan2(z - p_z, r - p_r) - self.t_a3_a4
    except TypeError:
      print("Impossible to reach this position...")

    # Orientation (t4, t5, t6)
    a, b, g = X[3:]

    R_0_6 = [[ np.cos(a)*np.cos(b)*np.cos(g) - np.sin(a)*np.sin(g) , -np.cos(a)*np.cos(b)*np.sin(g) - np.sin(a)*np.cos(g), np.cos(a)*np.sin(b)],
            [  np.sin(a)*np.cos(b)*np.cos(g) + np.cos(a)*np.sin(g) , -np.sin(a)*np.cos(b)*np.sin(g) + np.cos(a)*np.cos(g), np.sin(a)*np.sin(b)],
            [ -np.sin(b)*np.cos(g)                                 , np.sin(b)*np.sin(g)                                 ,  np.cos(b)         ]]
    R_0_6 = np.array(R_0_6)

    R = []
    for i, t in enumerate([t1, t2, t3]):
      t += self.D_H_table[i, 3]
      R.append(np.array([[np.cos(t), -np.sin(t)*np.cos(self.D_H_table[i,1]), np.sin(t)*np.sin(self.D_H_table[i,1])],
                         [np.sin(t), np.cos(t)*np.cos(self.D_H_table[i,1]), -np.cos(t)*np.sin(self.D_H_table[i,1])],
                         [0        , np.sin(self.D_H_table[i,1])          , np.cos(self.D_H_table[i,1])           ]]))

    R_0_3 = R[0] @ R[1] @ R[2]
    R_0_3_inv = np.linalg.inv(R_0_3)
    R_3_6 = R_0_3_inv @ R_0_6

    t5 = np.arccos(R_3_6[2, 2])
    if np.abs(np.abs(R_3_6[2, 2]) - 1) < self.ZERO_MARGIN:
      t4 = 0
      t6 = np.arctan2(R_3_6[1, 0], R_3_6[1, 1])
    else:
      t4 = np.arctan2(R_3_6[1, 2], R_3_6[0, 2])
      if (abs(t4) <= np.pi/2 + self.ZERO_MARGIN): # Good configuration
        t6 = np.arctan2(R_3_6[2, 1], -R_3_6[2, 0])
      else: # Wrong configuration (take the other one)
        t5 = -t5
        t4 = np.arctan2(-R_3_6[1, 2], -R_3_6[0, 2])
        t6 = np.arctan2(-R_3_6[2, 1], R_3_6[2, 0])

    Theta = np.array([t1, t2, t3, t4, t5, t6])

    if verbose:
      with np.printoptions(precision=2, suppress=True):
        print(f"Theta = \n{Theta}")

    return Theta


if __name__ == '__main__':
  rospy.init_node('Inverse_Kinematic')
  InverseKinematic()
  rospy.spin()
