#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler

from base_controller_interface.msg import BaseMoveAction, BaseMoveGoal, BaseMoveResult, BaseMoveFeedback

def base_controller_client(goal_pose, is_relative_pose):
    client = actionlib.SimpleActionClient('base_move_action', BaseMoveAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server found!")

    goal = BaseMoveGoal()

    # Set desired pose [x, y, z, roll, pitch, yaw]
    x, y, z = goal_pose[0], goal_pose[1], goal_pose[2]
    roll, pitch, yaw = goal_pose[3], goal_pose[4], goal_pose[5]
    quat = quaternion_from_euler(roll, pitch, yaw)

    goal.pose = Pose()
    goal.pose.position = Point(x, y, z)
    goal.pose.orientation = Quaternion(*quat)

    goal.is_relative_pose = is_relative_pose

    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Action completed! Success: {result.success}")

if __name__ == '__main__':
    rospy.init_node('test_base_controller_client')

    goal_pose = [0,0,0.57,0,0,0]
    base_controller_client(goal_pose, is_relative_pose=False)
