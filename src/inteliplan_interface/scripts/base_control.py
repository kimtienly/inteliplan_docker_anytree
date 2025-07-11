#!/usr/bin/env python
'''
This script directly command twist command to the robot
'''
import rospy
from geometry_msgs.msg import TwistStamped

def main():
    rospy.init_node('twist_mux_publisher')
    # pub = rospy.Publisher('/motion_reference/command_twist', TwistStamped, queue_size=10)
    pub = rospy.Publisher('/twist_mux/twist', TwistStamped, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    msg = TwistStamped()
    msg.header.frame_id = "base"
    msg.twist.linear.x = -0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
