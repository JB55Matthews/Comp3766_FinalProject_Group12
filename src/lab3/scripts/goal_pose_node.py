#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix

# Global constants for the goal pose (students should modify these values)
#GOAL_POSITION = np.array([-0.15, 0.429+0.429, 0.0]) # Modify these values 
#GOAL_POSITION = np.array([0.4, 0.4, 0.0]) # Modify these values 
#GOAL_POSITION = np.array([-0.15, 0.863, 0.0])
GOAL_POSITION = np.array([0.6, 0.3, 0.0])
# GOAL_ROTATION_MATRIX = np.a5rray([
#     [1.0, 0.0, 0.0],  # Modify these values
#     [0.0, 1.0, 0.0],
#     [0.0, 0.0, 1.0]
# ])
# GOAL_ROTATION_MATRIX = np.array([
#     [0.0, 0.0, -1.0],  # Modify these values
#     [-1.0, 0.0, 0.0],
#     [0.0, 1.0, 0.0]
# ])
GOAL_ROTATION_MATRIX = np.array([
    [-1.0, 0.0, 0.0],  # Modify these values
    [0.0, 0.0, -1.0],
    [0.0, 1.0, 0.0]
])

def publish_goal_pose():
    rospy.init_node("goal_pose_publisher", anonymous=True)
    pose_pub = rospy.Publisher("/goal_pose", Pose, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Convert rotation matrix to quaternion
    rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = GOAL_ROTATION_MATRIX
    quaternion = quaternion_from_matrix(rotation_matrix_4x4)

    # Define the goal pose
    goal_pose = Pose()
    goal_pose.position.x = GOAL_POSITION[0]
    goal_pose.position.y = GOAL_POSITION[1]
    goal_pose.position.z = GOAL_POSITION[2]

    goal_pose.orientation.x = quaternion[0]
    goal_pose.orientation.y = quaternion[1]
    goal_pose.orientation.z = quaternion[2]
    goal_pose.orientation.w = quaternion[3]

    rospy.loginfo("Publishing goal pose...")
    rospy.loginfo(f"Goal Position: {GOAL_POSITION}")
    rospy.loginfo(f"Goal Rotation Matrix:\n{GOAL_ROTATION_MATRIX}")
    rospy.loginfo(f"Converted Quaternion: {quaternion}")

    while not rospy.is_shutdown():
        pose_pub.publish(goal_pose)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_goal_pose()
    except rospy.ROSInterruptException:
        pass
