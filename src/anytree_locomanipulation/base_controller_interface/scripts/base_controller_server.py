#!/usr/bin/env python3

import rospy
import actionlib
from base_controller_interface.base_position_controller import BaseController
from base_controller_interface.msg import BaseMoveAction, BaseMoveResult, BaseMoveFeedback

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

        feedback = BaseMoveFeedback()
        feedback.progress = 0.0
        self.server.publish_feedback(feedback)

        result = BaseMoveResult()

        try:
            success = self.controller.go_abs([
                goal.pose.position.x,
                goal.pose.position.y,
                goal.pose.position.z,
                *self._quat_to_euler(goal.pose.orientation)
            ])
            feedback.progress = 1.0
            self.server.publish_feedback(feedback)

            result.success = True
            self.server.set_succeeded(result)
        except Exception as e:
            rospy.logerr("Base move failed: %s", str(e))
            result.success = False
            self.server.set_aborted(result)

    def _quat_to_euler(self, quat):
        from tf.transformations import euler_from_quaternion
        return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])


if __name__ == '__main__':
    rospy.init_node("base_controller_action_server")
    server = BaseControllerActionServer()
    rospy.spin()
