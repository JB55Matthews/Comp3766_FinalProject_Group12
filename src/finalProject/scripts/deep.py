#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from topic_tools.srv import MuxSelect, MuxSelectRequest
import numpy as np

def control_robot_arm():
    rospy.init_node('script_joint_publisher')
    pub = rospy.Publisher('/mux/script', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Switch mux to script input
    print("here")
    rospy.wait_for_service('mux/select')
    print("here 2")
    mux_select = rospy.ServiceProxy('mux/select', MuxSelect)
    mux_select('/mux/script')  # Take control

    try:
        while not rospy.is_shutdown():
            # Create and publish your JointState message
            joint_positions = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
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
        #rospy.wait_for_service('mux/select', timeout =2)
        mux_select = rospy.ServiceProxy('mux/select', MuxSelect)
        mux_select(mux_select_request)  # Release control

if __name__ == '__main__':
    try:
        control_robot_arm()
    except rospy.ROSInterruptException:
        pass