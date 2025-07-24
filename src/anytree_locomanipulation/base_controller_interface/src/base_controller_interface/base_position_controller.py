#!/usr/bin/env python3
'''
This is an interface to position controller from navigation_drs
'''
import rospy
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped,Point, Quaternion
import position_controller.msg
import time
import numpy as np
import tf.transformations as tft
import math

class BaseController:
    ERROR_TOLERANCE = 0.25
    def __init__(self):
        self.actionClient = actionlib.SimpleActionClient('position_controller', position_controller.msg.PositionControllerAction)
        self.actionClient.wait_for_server() # Waits until the action server has started up and started

        self.base_pose_sub = rospy.Subscriber("/state_estimator/pose_in_odom", PoseWithCovarianceStamped, self.base_pose_cb, queue_size=10)
        self.base_pose = [0]*6

        self.cached_target_pose = None # to track stability of the base
        time.sleep(0.1) # such that it updates the init base pose

        self.timer = rospy.Timer(rospy.Duration(1),self.rebase_callback)
        self.moving = False

    def base_pose_cb(self, data):        
        pose = data.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        q = pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tft.euler_from_quaternion(quaternion)
        self.base_pose = [x, y, z, roll, pitch, yaw]

    def send_command(self, msg, timeout=60):
        '''
        args:
            - msg: PositionControllerGoal type
        '''
        self.moving = True
        self.actionClient.send_goal(msg)
        print('waiting for result')
        while(self.actionClient.wait_for_result(rospy.Duration.from_sec(1.0)) is False):
            pass
        self.moving = False
        # checking success manually because navigation_drs/position_controller returns None for self.actionClient.get_result())
        if self.calc_distance(self.cached_target_pose, self.base_pose) > self.ERROR_TOLERANCE:
            return False
        return True

    def go_abs(self, abs_pose):
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
        
        self.cached_target_pose = abs_pose
        pose = convert_pose(abs_pose)
        
        goal = position_controller.msg.PositionControllerGoal(goal=pose)
        rospy.loginfo('Moving to base goal: {}'.format(abs_pose))
        return self.send_command(goal)

    def go_rel(self, rel_pose):
        abs_pose = np.asarray(self.base_pose)+np.asarray(rel_pose)
        return self.go_abs(abs_pose)
    
    def rebase_callback(self, event):
        if not self.moving:
            if self.cached_target_pose is not None:
                if self.calc_distance(self.cached_target_pose, self.base_pose) > self.ERROR_TOLERANCE:
                    rospy.logwarn("Distance to cached goal increased to {}. Rebasing..!".format(self.calc_distance(self.cached_target_pose, self.base_pose)))
                    self.go_abs(self.cached_target_pose)

    def calc_distance(self, p1, p2):
        def normalize_angle(a):
            return (a + math.pi) % (2 * math.pi) - math.pi
        # Normalize angular parts: roll, pitch, yaw
        a1 = [normalize_angle(a) for a in p1[3:]]
        a2 = [normalize_angle(a) for a in p2[3:]]
        # Combine translation and rotation
        d_linear = math.dist(p1[:3], p2[:3])
        d_angular = math.dist(a1, a2)
        return math.hypot(d_linear, d_angular)
    
if __name__ == "__main__":
    rospy.init_node('base_controller')
    contrl = BaseController()
    re = contrl.go_rel([0,0, 0, 0,0,-1.57]) # [x,y,z,r,p,y]
    rospy.loginfo('Success: {}'.format(re))
