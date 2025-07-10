import numpy
import colorsys

HSV_tuples =      [(0., 1.0, 1.0),(0.15, 1.0, 1.0), (0.3, 1.0, 1.0), (0.45, 1.0, 1.0), (0.6, 1.0, 1.0), (0.75, 1.0, 1.0), (0.9, 1.0, 1.0)]
RGB_tuples = [colorsys.hsv_to_rgb(*x) for x in HSV_tuples]

HSV_tuples_mid = [(0., 1.0, 0.75),(0.15, 1.0, 0.75), (0.3, 1.0, 0.75), (0.45, 1.0, 0.75), (0.6, 1.0, 0.75), (0.75, 1.0, 0.75), (0.9, 1.0, 0.75)]
RGB_tuples_mid = [colorsys.hsv_to_rgb(*x) for x in HSV_tuples_mid]

HSV_tuples_dark = [(0., 1.0, 0.5),(0.15, 1.0, 0.5), (0.3, 1.0, 0.5), (0.45, 1.0, 0.5), (0.6, 1.0, 0.5), (0.75, 1.0, 0.5), (0.9, 1.0, 0.5)]
RGB_tuples_dark = [colorsys.hsv_to_rgb(*x) for x in HSV_tuples_dark]


#addSignal('/PoseStamped', msg.header.stamp, msg.pose.position.x)
#addSignal('/state_estimator/twist', msg.header.stamp, msg.twist.twist.linear.x, color=RGB_tuples[0])
addSignal('/position_controller/position_controller_cmd', msg.header.stamp, msg.twist.linear.x, color=RGB_tuples[1])
addSignal('/state_estimator/anymal_state', msg.header.stamp, msg.twist.twist.linear.x, color=RGB_tuples[2])
#addSignal('/position_controller/position_controller_cmd_tmp', msg.header.stamp, msg.twist.linear.x, color=RGB_tuples[1])

p = addPlot()
#addSignal('/state_estimator/twist', msg.header.stamp, msg.twist.twist.linear.y, color=RGB_tuples[0])
addSignal('/position_controller/position_controller_cmd', msg.header.stamp, msg.twist.linear.y, color=RGB_tuples[1])
addSignal('/state_estimator/anymal_state', msg.header.stamp, msg.twist.twist.linear.y, color=RGB_tuples[2])
#addSignal('/position_controller/position_controller_cmd_tmp', msg.header.stamp, msg.twist.linear.y, color=RGB_tuples[1])

p = addPlot()
#addSignal('/state_estimator/twist', msg.header.stamp, msg.twist.twist.linear.z, color=RGB_tuples[0])
addSignal('/position_controller/position_controller_cmd', msg.header.stamp, msg.twist.linear.z, color=RGB_tuples[1])
addSignal('/state_estimator/anymal_state', msg.header.stamp, msg.twist.twist.linear.z, color=RGB_tuples[2])
#addSignal('/position_controller/position_controller_cmd_tmp', msg.header.stamp, msg.twist.linear.z, color=RGB_tuples[1])




p = addPlot()
#addSignal('/state_estimator/twist', msg.header.stamp, msg.twist.twist.linear.z, color=RGB_tuples[0])
addSignal('/position_controller/position_controller_cmd', msg.header.stamp, msg.twist.angular.z, color=RGB_tuples[1])
addSignal('/state_estimator/anymal_state', msg.header.stamp, msg.twist.twist.angular.z, color=RGB_tuples[2])
#addSignal('/position_controller/position_controller_cmd_tmp', msg.header.stamp, msg.twist.angular.z, color=RGB_tuples[1])


