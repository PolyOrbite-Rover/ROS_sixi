#!/usr/bin/env python
# spins off a thread to listen for joint_states messages
# and provides the same information (or subsets of) as a service

import rospy
import threading
from sensor_msgs.msg import JointState
from joint_states_listener.srv import ReturnJointPos, ReturnJointPosResponse


#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()
        self.position = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('return_joint_pos', ReturnJointPos, self.return_joint_pos)

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()

    # Callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.position = msg.position
        self.lock.release()

    # Server callback: returns array of position
    def return_joint_pos(self, req):
        self.lock.acquire()
        if not self.position:
            rospy.logerr("Unable to get the 'joint_states': MAKE SURE this topic is active!")
        position = self.position
        self.lock.release()
        return ReturnJointPosResponse(position)

#run the server
if __name__ == "__main__":
    latestjointstates = LatestJointStates()
    print ("joints_states_listener server started, waiting for queries")
    rospy.spin()
