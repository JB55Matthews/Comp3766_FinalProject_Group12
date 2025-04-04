#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

def ik():
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.init_node("ik_solver_node", anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joint_positions = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
        joint_msg.position = joint_positions
        joint_pub.publish(joint_msg)

if __name__ == "__main__":
    try:
        ik()
    except rospy.ROSInterruptException:
        pass