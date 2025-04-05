#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from topic_tools.srv import MuxSelect, MuxSelectRequest
import modern_robotics as mr
def rotationFromQuaternion(x, y, z, w):
    return np.array([[1-2*y**2-2*z**2, 2*x*y-2*w*z, 2*x*z+2*w*y],
     [2*y*x+2*z*w, 1-2*x**2-2*z**2, 2*z*y-2*x*w],
     [2*x*z-2*w*y, 2*z*y+2*x*w, 1-2*x**2-2*y**2]])
#---------------------------------------------------------
# Note: RViz link 8 orientation and position will be 
#       correct, but expand window to see e-0x as will not show
#       exact positions if 0, but along lines of "1.7e-09"
#
# User input position
position = np.array([0.3157, 0.0, 0.6571])

# User input rotation matrix
orientation = np.array([[0, 0, -1],
                        [0, 1, 0],
                        [1, 0, 0]])

# or give orientation as Quaternion. If so, comment out above matrix
# orientation = rotationFromQuaternion(0, -0.707, 0, 0.707)

# make sure to "source devel/setup.bash"
# then simply "rosrun finalProject inverse_kinematcs.py"
#------------------------------------------------------

L1 = 0.55
L2 = 0.3
L3 = 0.06

ew = 0.001
ev = 0.0001

def inverse_kinematics(position, orientation):
    theta = np.array([0, np.pi/6, np.pi/6, np.pi/6, 0, np.pi/6, np.pi/6])
    M = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0,0,1, L1 + L2 + L3],
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

def control_arm():
    rospy.init_node('script_joint_publisher')
    pub = rospy.Publisher('/mux/script', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Switch mux to script input
    rospy.wait_for_service('/mux/select')
    mux_select = rospy.ServiceProxy('/mux/select', MuxSelect, persistent=True)
    mux_select('/mux/script')  # Take control

    joint_positions = inverse_kinematics(position, orientation)

    try:
        while not rospy.is_shutdown():
            # Create and publish your JointState message
            # joint_positions = inverse_kinematics(position, orientation)
            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
            joint_msg.position = joint_positions  # Your target positions
            pub.publish(joint_msg)
            rate.sleep()
    finally:
        # Ensure mux switches back to GUI input even on exception
        mux_select_request = MuxSelectRequest()
        mux_select_request.topic = "/mux/gui"
        mux_select = rospy.ServiceProxy('/mux/select', MuxSelect, persistent=True)
        mux_select(mux_select_request)  # Release control

if __name__ == "__main__":
    try:
        control_arm()
    except rospy.ROSInterruptException:
        pass