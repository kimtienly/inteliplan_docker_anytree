#!/usr/bin/env python3

import rospy
import actionlib
from base_controller_interface.base_position_controller import BaseController
from base_controller_interface.msg import BaseMoveAction, BaseMoveResult, BaseMoveFeedback
import numpy as np
from tf.transformations import euler_from_quaternion

class BaseControllerActionServer:
    def __init__(self):
        self.controller = BaseController()
        self.server = actionlib.SimpleActionServer(
            'base_move_action',
            BaseMoveAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("BaseControllerActionServer started.")

    def execute_cb(self, goal):
        rospy.loginfo("Received goal: %s", goal.pose)

        result = BaseMoveResult()

        received_pose = [
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                *self._quat_to_euler(goal.pose.orientation)
            ]

        if goal.is_relative_pose:
            target_pose = self.get_abs_pose(received_pose)
        else: 
            target_pose = received_pose

        try:
            success = self.controller.go_abs(target_pose)
            result.success = success
            self.server.set_succeeded(result)
        except Exception as e:
            rospy.logerr("Base move failed: %s", str(e))
            result.success = False
            self.server.set_aborted(result)

    def get_abs_pose(self, rel_pose):
        abs_pose = np.asarray(self.controller.base_pose)+np.asarray(rel_pose)
        return abs_pose

    def _quat_to_euler(self, quat):
        return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


if __name__ == '__main__':
    rospy.init_node("base_controller_action_server")
    server = BaseControllerActionServer()
    rospy.spin()
