#!/usr/bin/env python3
'''
This is an interface to position controller from navigation_drs
'''
import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
import position_controller.msg
import time

class BaseController():

    def __init__(self):
        self.actionClient = actionlib.SimpleActionClient('position_controller', position_controller.msg.PositionControllerAction)
        self.actionClient.wait_for_server() # Waits until the action server has started up and started

    def executeTasks(self, pose, timeout=100):
        def convert_pose(pos):
            pos_ros = Point(pos[0], pos[1], pos[2])
            quat = quaternion_from_euler (pos[3],pos[4],pos[5])
            quat_ros = Quaternion(quat[0],quat[1],quat[2],quat[3])
            goal_pose = Pose(pos_ros, quat_ros)
            # send goal to position controller: for trot and static_walk
            pose = PoseStamped()
            pose.pose = goal_pose
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            return pose
        
        pose = convert_pose(pose)
        goal = position_controller.msg.PositionControllerGoal(goal=pose)
        self.actionClient.send_goal(goal)
        rospy.loginfo('Base goal sent. Moving to goal..')
        
        while (self.actionClient.wait_for_result() is False) and timeout>0:
            time.sleep(1)
            timeout-=1

        return self.actionClient.wait_for_result()

if __name__ == "__main__":
    rospy.init_node('base_controller')
    contrl = BaseController()
    re = contrl.executeTasks([0,0,0.57, 0,0,0]) # [x,y,z,r,p,y]
    print('Success: {}'.format(re))