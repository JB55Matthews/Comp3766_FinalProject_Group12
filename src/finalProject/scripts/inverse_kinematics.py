#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
# import modern_robotics as mr

L1 = 550
L2 = 300
L3 = 60

ew = 0.001
ev = 0.0001

position = np.array([0.3157, 0.0, 0.6571])
orientation = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])

printed_count = 0

def inverse_kinematics(position, orientation):
    theta = np.array([0, np.pi/6, np.pi/6, np.pi/6, 0, np.pi/6, 0])
    M = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0,0,0, L1 + L2 + L3],
                 [0, 0, 0, 1]])

    B1 = np.array([0, 0, 1, 0, 0, 0])
    B2 = np.array([0, 1, 0, L1 + L2 + L3, 0, 0])
    B3 = np.array([0, 0, 1, 0, 0, 0])
    B4 = np.array([0, 1, 0, L2 + L3, 0, 0])
    B5 = np.array([0, 0, 1, 0, 0, 0])
    B6 = np.array([0, 1, 0, L3, 0, 0])
    B7 = np.array([0, 0, 1, 0, 0, 0])
    BList = np.array([B1, B2, B3, B4, B5, B6, B7]).T 
   
    T = np.array([[orientation[0, 0], orientation[0, 1], orientation[0, 2], position[0]],
                  [orientation[1, 0], orientation[1, 1], orientation[1, 2], position[1]],
                  [orientation[2, 0], orientation[2, 1], orientation[2, 2], position[2]],
                  [0, 0, 0, 1]])
    joint_angles = mr.IKinBody(BList, M, T, theta, ew, ev)

    print("Joint Angles: ", joint_angles) 
    return joint_angles[0]

if __name__ == "__main__":
    rospy.init_node("ik_solver_node", anonymous=False)

    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    joint_positions = inverse_kinematics(position, orientation)

    joint_msg = JointState()
    joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]

    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.position = joint_positions
    joint_pub.publish(joint_msg)
    print("Publishing joint states...", joint_msg)

    rospy.spin()