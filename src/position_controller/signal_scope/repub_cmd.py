#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from anymal_msgs.msg import *

import tf

import math
import numpy

global qsMsg
qsMsg = None

def cmdCb(msg):
    print("got it")
    ts = TwistStamped()
    ts.twist = msg
    ts.header = qsMsg.header

    pub.publish(ts)


def qsCb(msg):
    global qsMsg
    qsMsg = msg

rospy.init_node('repub_cmd', anonymous=True)
pub = rospy.Publisher("/position_controller/position_controller_cmd_tmp", TwistStamped, queue_size=10)
rospy.Subscriber("/position_controller/position_controller_cmd", Twist, cmdCb)
rospy.Subscriber("/state_estimator/anymal_state", QuadrupedState, qsCb)
rospy.spin()
