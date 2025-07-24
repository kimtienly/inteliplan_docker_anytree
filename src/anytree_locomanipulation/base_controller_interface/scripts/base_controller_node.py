#!/usr/bin/env python3
"""
This node wraps the BaseController class and exposes it as a ROS service node.
Other nodes can call /base_controller/go_abs or /go_rel to command base movement.
"""

import rospy
from base_controller_interface.srv import GoPose, GoPoseResponse  # custom service
from base_controller_interface.base_position_controller import BaseController

def go_abs_service(req):
    """
    ROS service handler for absolute pose commands
    """
    success = controller.go_abs(list(req.pose.data))
    return GoPoseResponse(success=True)


def go_rel_service(req):
    """
    ROS service handler for relative pose commands
    """
    success = controller.go_rel(list(req.pose.data))
    return GoPoseResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("base_controller")

    controller = BaseController()

    rospy.Service("/base_controller/go_abs", GoPose, go_abs_service)
    rospy.Service("/base_controller/go_rel", GoPose, go_rel_service)

    rospy.loginfo("base_controller service started!")
    rospy.spin()
