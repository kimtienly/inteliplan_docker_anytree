import rospy
from std_msgs.msg import Float64MultiArray
from base_controller_interface.srv import GoPose

rospy.init_node("test_caller")

rospy.wait_for_service("/base_controller/go_abs")
go_abs = rospy.ServiceProxy("/base_controller/go_abs", GoPose)

pose = Float64MultiArray(data=[0, 0, 0.57, 0, 0, 0])  # x,y,z,r,p,y
resp = go_abs(pose)
print("Sent goal, success:", resp.success)
