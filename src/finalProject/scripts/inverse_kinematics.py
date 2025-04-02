
# import rospy
import numpy as np
# from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState
import modern_robotics as mr

L1 = 550
L2 = 300
L3 = 60

ew = 0.001
ev = 0.0001

position = np.array([0, 20, 50])
orientation = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])

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

inverse_kinematics(position, orientation)          

                 

