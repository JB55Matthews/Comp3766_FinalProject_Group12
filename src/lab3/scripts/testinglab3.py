#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def publish_joint_states():
    rospy.init_node('joint_state_publisher_node') # Initialize the node

    # Create a publisher for the /joint_states topic
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # Create a JointState message
    joint_state = JointState()

    # Set the joint names
    joint_state.name = ['joint1', 'joint2', 'joint3']

    # Set the joint positions
    joint_state.position = [1.57, 1.57, -1.57]

    # # Set the joint velocities (optional)
    # joint_state.velocity = [0.0, 0.0, 0.0]

    # # Set the joint efforts (optional)
    # joint_state.effort = [0.0, 0.0, 0.0]

    # Publish the message
    rospy.loginfo("Publishing joint states")
    pub.publish(joint_state)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    publish_joint_states()

# import rospy
# import numpy as np
# from geometry_msgs.msg import Pose
# from sensor_msgs.msg import JointState
# from tf.transformations import quaternion_from_matrix

# def compute_ik():
#     """
#     Function that calulates the PUMA analytical IK function.
#     Should take a 3x1 position vector and a 3x3 rotation matrix,
#     and return a list of joint positions.
#     """
#     GOAL_POSITION = np.array([0.6, 0.3, 0.0])
#     GOAL_ROTATION_MATRIX = np.array([
#     [-1.0, 0.0, 0.0],  # Modify these values
#     [0.0, 0.0, -1.0],
#     [0.0, 1.0, 0.0]])

#     # Convert rotation matrix to quaternion
#     rotation_matrix_4x4 = np.eye(4)
#     rotation_matrix_4x4[:3, :3] = GOAL_ROTATION_MATRIX
#     quaternion = quaternion_from_matrix(rotation_matrix_4x4)

#     goal_pose = Pose()
#     goal_pose.position.x = GOAL_POSITION[0]
#     goal_pose.position.y = GOAL_POSITION[1]
#     goal_pose.position.z = GOAL_POSITION[2]

#     goal_pose.orientation.x = quaternion[0]
#     goal_pose.orientation.y = quaternion[1]
#     goal_pose.orientation.z = quaternion[2]
#     goal_pose.orientation.w = quaternion[3]

#     position = np.array([goal_pose.position.x, goal_pose.position.y, goal_pose.position.z])
#     q = goal_pose.orientation
#     orientation = np.array([
#         [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
#         [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
#         [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
#     ])
#     # print("\nReceived Position:")
#     # print(position)

#     # print("\nReceived Orientation (3x3 Rotation Matrix):")
#     # print(orientation)

#     # PUMA Robot Parameters (meters)
#     d1, a2, a3 = 0.150, 0.432, 0.432  # Given parameters
    
#     # Replace with the actual analytical IK computation
#     # Insert you code here
#     # Your code here:
#     # Follow Section 6.1.1 6R PUMA-Type Arm of the textbook to implement the inverse kinematics.
#     px, py, pz = position[0], position[1], position[2]

#     theta1 = np.arctan2(py, px) - np.arctan2(d1, np.sqrt(px**2 + py**2 - d1**2))
#     D = (px**2 + py**2 + pz**2 - d1**2 - a2**2 - a3**2)/(2*a2*a3)
#     theta3 = np.arctan2(np.sqrt(1-(D**2)), D)
#     theta2 = np.arctan2(pz, np.sqrt(px**2+py**2-d1**2)) - np.arctan2(a3*np.sin(theta3), a2+a3*np.cos(theta3))
    
#     # Only need rotation components of expenential matrices to compute R for ZYX Euler angles
#     # w1: (0,0,1), w2: (0, -1, 0), w3: (0, -1, 0)
#     I = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])
#     skew1 = np.array([[0, -1, 0],[1, 0, 0],[0, 0, 0]])
#     skew23 = np.array([[0, 0, -1],[0, 0, 0],[1, 0, 0]])

#     e_skew1 = I + np.sin(theta1)*skew1 + (1-np.cos(theta1))*np.matmul(skew1, skew1)
#     e_skew2 = I + np.sin(theta2)*skew23 + (1-np.cos(theta2))*np.matmul(skew23, skew23)
#     e_skew3 = I + np.sin(theta3)*skew23 + (1-np.cos(theta3))*np.matmul(skew23, skew23)

#     inverse_e_skew1 = np.transpose(e_skew1)
#     inverse_e_skew2 = np.transpose(e_skew2)
#     inverse_e_skew3 = np.transpose(e_skew3)

#     # For PUMA arm in 0 position, end_effector frame is co-incedent with space frame, so M_rotate = I3, can leave out

#     R = np.matmul(np.matmul(inverse_e_skew3, inverse_e_skew2) ,np.matmul(inverse_e_skew1, orientation))

#     theta4 = np.arctan2(R[1][0], R[0][0])
#     theta5 = np.arctan2(-R[2][0], np.sqrt(R[0][0]**2 + R[1][0]**2))
#     theta6 = np.arctan2(R[2][1], R[2][2])


#     joint_angles = np.zeros(6)  # Placeholder for the actual joint angles, comment this line
#     joint_angles[0], joint_angles[1], joint_angles[2] = theta1, theta2, theta3
#     joint_angles[3], joint_angles[4], joint_angles[5] = theta4, theta5, theta6
    
#     # Comment this line when you have your solution
#     # theta1, theta2, theta3, theta4, theta5, theta6 = [0.0] * 6

#     # joint_positions = np.array([theta1, theta2, theta3, theta4, theta5, theta6])
#     print(joint_angles)
#     return joint_angles
# def pose_callback():
#     """
#     Callback function to handle incoming end-effector pose messages.
#     You probably do not have to change this
#     """
#     rospy.init_node("ik_solver_node", anonymous=True)

#     # Publisher: sends joint positions

#     joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
#     # Extract position (3x1)
#     # position = np.array([msg.position.x, msg.position.y, msg.position.z])

#     # Extract orientation (3x3 rotation matrix from quaternion)
#     # q = msg.orientation
#     # orientation = np.array([
#     #     [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
#     #     [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
#     #     [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
#     # ])

#     # Compute inverse kinematics
#     joint_positions = compute_ik()

#     # Publish joint states
#     joint_msg = JointState()
#     joint_msg.header.stamp = rospy.Time.now()
#     joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
#     joint_msg.position = joint_positions
#     joint_pub.publish(joint_msg)

#     rospy.spin()
# if __name__ == "__main__":
#     # rospy.init_node("ik_solver_node", anonymous=True)

#     # # Publisher: sends joint positions

#     # joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
#     # rospy.spin()
#     pose_callback()

# def publish_goal_pose():
#     #rospy.init_node("goal_pose_publisher", anonymous=True)
#     rospy.init_node("ik_solver_node", anonymous=True)
#     # pose_pub = rospy.Publisher("/goal_pose", Pose, queue_size=10)
#     joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
#     rate = rospy.Rate(10)  # 10 Hz

#     joint_positions = compute_ik()

#     # Publish joint states
#     joint_msg = JointState()
#     joint_msg.header.stamp = rospy.Time.now()
#     joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
#     joint_msg.position = joint_positions

#     while not rospy.is_shutdown():
#         joint_pub.publish(joint_msg)
#         rate.sleep()

# if __name__ == "__main__":
#     try:
#         publish_goal_pose()
#     except rospy.ROSInterruptException:
#         pass
