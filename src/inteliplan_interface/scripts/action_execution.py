'''
This is the interface to control the robot, which includes:
- communication with anytree_manipulation for 'pick' and 'place' actions.
- directly control simple actions: 'turn', 'search', 'go'.
'''

import actionlib
import anytree_manipulation.msg as msg
import math
import rospy
from geometry_msgs.msg import *
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import position_controller.msg
import time
from base_position_controller import BaseController
from base_controller_interface.msg import BaseMoveAction, BaseMoveGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class AnytreeInterface:
    """
    This class is an interface to HSR, including communicating with Orion manipulation stack and internal command
    """

    def __init__(self):
        # TF defaults to buffering 10 seconds of transforms. Choose 60 second buffer.
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration.from_sec(60.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # rospy.wait_for_service("/base_controller/go_abs")
        self.base_client = actionlib.SimpleActionClient('base_move_action', BaseMoveAction)
        rospy.loginfo("Waiting for action server...")
        self.base_client.wait_for_server()
        rospy.loginfo("Base position controller server found!")

        self.pickup_client = actionlib.SimpleActionClient("pick_up_object", msg.PickUpObjectAction)

        print("Waiting for pick_up_object server")
        self.pickup_client.wait_for_server()
        print("Finished waiting for pick_up_object server")
        
        
        self.place_client = actionlib.SimpleActionClient("put_object_on_surface", msg.PutObjectOnSurfaceAction)

        print("Waiting for put_object_on_surface server")
        self.place_client.wait_for_server()
        print("Finished waiting for put_object_on_surface server")

        self.motion_plan_publisher = rospy.Publisher("/motion_plan",
                                                     JointTrajectory, 
                                                     queue_size=10,
                                                     tcp_nodelay=True
                                                     )
        self.joint_states_sub = rospy.Subscriber(
            "/z1_gazebo/joint_states_filtered", JointState, self.joint_states_cb, queue_size=10)
        self.joint_states_position = None
        while self.joint_states_position is None:
            rospy.sleep(1)
        rospy.loginfo("Anytree interface initialized!")



    def send_base_goal(self, goal_pose, is_relative_pose):
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
        self.base_client.send_goal(goal)

        self.base_client.wait_for_result()
        result = self.base_client.get_result()
        rospy.loginfo(f"Base action completed! Success: {result.success}")
        return result.success

    def pick_up_object_client(self, object_pose, approach_axis=None, extend_distance=0, is_bin_bag=False, goal_tf = "pick_up_goal"):
        
        self.publish_goal_pose_tf(object_pose, goal_pose_name=goal_tf)
        # goal_tf = object_pose


        # Creates a goal to send to the action server.
        goal_msg = msg.PickUpObjectGoal(goal_tf=goal_tf, approach_axis=approach_axis, extend_distance=extend_distance, is_bin_bag=is_bin_bag)

        # Sends the goal to the action server.
        self.pickup_client.send_goal(goal_msg)

        # Waits for the server to finish performing the action.
        self.pickup_client.wait_for_result()

        rospy.sleep(1)
        # Return the result of executing the action
        return self.pickup_client.get_result()

    def put_object_on_surface_client(self, surface_pose, shelf_tf_ref_="", goal_tf = "place_goal",override_ori=True):

        if override_ori:
            trans_stamped = self._tf_buffer.lookup_transform("map", "link00", rospy.Time(), timeout=rospy.Duration(0))
            rot = trans_stamped.transform.rotation

        self.publish_goal_pose_tf(surface_pose, goal_pose_name=goal_tf, ori=rot)



        # Creates a goal to send to the action server.
        # See PutObjectOnSurface.action to see description of these fields
        goal_msg = msg.PutObjectOnSurfaceGoal(goal_tf=goal_tf,
                                            abandon_action_if_no_plane_found=False,
                                            drop_object_by_metres=0.0,
                                            check_weight_grams=-1,
                                            shelf_tf_ref=shelf_tf_ref_)

        # Sends the goal to the action server.
        self.place_client.send_goal(goal_msg)

        # Waits for the server to finish performing the action.
        self.place_client.wait_for_result()

        # Return the result of executing the action
        return self.place_client.get_result()
    

    def turn(self, direction):
        if direction == 'left':
            angle = math.pi/2
        elif direction == 'right':
            angle = -math.pi/2
        elif direction == 'around':
            angle = -math.pi
        try:
            rel_pose = [0, 0, 0, 0, 0, angle]
            return self.send_base_goal(rel_pose, is_relative_pose=True)
        except:
            return False

    def search(self, obj, vision_func):
        delta_joint0 = math.pi/3
        scan_range = np.linspace(-delta_joint0, delta_joint0, 5).tolist()
        curr_joint = self.joint_states_position
        for rot in scan_range:
            print('step:',rot)
            desire_joint = np.asarray(curr_joint)+np.asarray([rot,0,0,0,0,0])
            self.send_joint_command(desire_joint)
            if vision_func(obj):
                print('<vision>: found ')
                import time
                time.sleep(2)
                # rospy.sleep(2)
                print('Checking feasibility..')
                print('<feasibility>: 1')
                return True
                    
        return False

    def move(self, loc, absolute = True):
        '''
        args: loc: [x, y, z, r, p, y] of robot base
        a representation of API, but works the same way as self.send_base_goal
        '''
        return self.send_base_goal(loc, is_relative_pose=not (absolute))
        
    def drawer(self, pull_dis, handle_sub = "yolo_pointcloud/detection_poses"):#
        import tf
        import numpy as np
        from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, TransformStamped
        from std_msgs.msg import Header
        from manipulation.collision_mapping import CollisionMapper
        from manipulation.msg import BoundingBox
        from pyquaternion import Quaternion
        import hsrb_interface.geometry as geometry
        

        LOOK_DISTANCE = 0.1
        SAFE_DISTANCE = 0.15
        FINGER_LENGTH = 0.088
        GRASP_DISTANCE = SAFE_DISTANCE-FINGER_LENGTH

        def pose_cb(msg):
            # print('callback')
            global live_poses,live_trans,live_rot,live_time

            live_time = msg.header.stamp.to_sec()

            (trans,rot) = listener.lookupTransform('map', 'head_rgbd_sensor_rgb_frame', msg.header.stamp)
            live_trans = np.array(trans)
            live_rot = Quaternion(rot[3],rot[0],rot[1],rot[2])

            live_poses = [(
                live_trans+live_rot.rotate(np.array([pose.position.x,pose.position.y,pose.position.z])),
                live_rot*Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z),
                np.array([pose.position.x,pose.position.y,pose.position.z]),
                Quaternion(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z)
                ) for pose in msg.poses]
            

        def get_closest_pose(target):
            global live_poses, live_time
            MAX_TRIES = 20

            time = rospy.Time.now().to_sec()
            i = 0
            while live_poses is None and i < MAX_TRIES:
                rospy.sleep(0.1)
                i += 1
            while live_time<time:
                rospy.sleep(0.1)
            

            ## find closest handle to target
            dist = 100000
            pose = None
            while pose == None and i < MAX_TRIES:
                for p in live_poses:
                    d = np.linalg.norm(p[0]-target)
                    if d < dist:
                        dist = d
                        pose = p
                if pose == None:
                    rospy.sleep(0.1)
                elif pose[1].rotate(np.array([0.0,0.0,1.0])) @ np.array([0.0,0.0,1.0]) >= 0.2:
                    pose == None
                    rospy.sleep(0.1)
                i += 1
            # print(pose)
            if pose == None:
                return (None,None,None,None)
            return pose
            

        def move_base(move):
            if move:
                self.whole_body.linear_weight = 30.0
                self.whole_body.angular_weight = 50.0
            else:
                self.whole_body.linear_weight = 100.0
                self.whole_body.angular_weight = 100.0

        def look_at_hand(look):
            self.whole_body.looking_hand_constraint = look


        def open(target,pull_distance):
            global live_trans, live_rot

            
            ## move to initial pose
            move_base(True)
            look_at_hand(True)
            pos, ori, _, _ = get_closest_pose(target)

            collision_world = get_goal_cropped_collision_map(Point(pos[0],pos[1],pos[2]))
            print('Collision world has been cropped')
            with collision_world:
                pos += ori.rotate(np.array([-LOOK_DISTANCE,0.0,-SAFE_DISTANCE]))
                print("publish initial pose")
                pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                #rospy.sleep(1)
                # self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(pos[0],pos[1],pos[2]),geometry.Quaternion(ori.x,ori.y,ori.z,ori.w))], ref_frame_id='map')
                #rospy.sleep(2)

                ## move to secondary pose
                move_base(False)
                abs_pos, abs_ori, rel_pos, rel_ori = get_closest_pose(target)
                print("publish second pose")
                if rel_pos is not None:
                    pos = abs_pos
                    ori = abs_ori
                    pos += ori.rotate(np.array([0.0,0.0,-SAFE_DISTANCE]))
                    print("Moving rel")
                    rel_pos += rel_ori.rotate(np.array([0.0,0.0,-SAFE_DISTANCE]))
                    pub.publish(Header(frame_id='head_rgbd_sensor_rgb_frame'),Pose(Point(rel_pos[0],rel_pos[1],rel_pos[2]),rel_ori))
                    self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(rel_pos[0],rel_pos[1],rel_pos[2]),geometry.Quaternion(rel_ori.x,rel_ori.y,rel_ori.z,rel_ori.w))], ref_frame_id='head_rgbd_sensor_rgb_frame')
                else:
                    print("Moving abs")
                    pos += ori.rotate(np.array([LOOK_DISTANCE,0.0,0.0]))
                    pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                    self.whole_body.move_end_effector_pose([geometry.Pose(geometry.Vector3(pos[0],pos[1],pos[2]),geometry.Quaternion(ori.x,ori.y,ori.z,ori.w))], ref_frame_id='map')
                    

                ## show grasp pose relitive to map
                pos += ori.rotate(np.array([0.0,0.0,GRASP_DISTANCE]))
                print("publish predicted grasp pose")
                pub.publish(Header(frame_id='map'),Pose(Point(pos[0],pos[1],pos[2]),ori))
                #rospy.sleep(1)

                ## show grasp pose relitive to hand
                (trans,rot) = listener.lookupTransform('hand_palm_link', 'map',rospy.Time())
                move = (Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,GRASP_DISTANCE]))
                print("publish target grasp pose")
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(move[0],move[1],move[2]),Quaternion(1,0,0,0)))
                #rospy.sleep(1)

                ## move hand to handle and grab
                self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,1])), GRASP_DISTANCE, ref_frame_id='hand_palm_link')  # 0.058
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(0,0,FINGER_LENGTH),Quaternion(1,0,0,0)))
                # gripper.command(-1)
                self.gripper.apply_force(1)
                # gripper.set_distance(0.0)
                # gripper.apply_force(0.8)

                ## pull handle
                move_base(True)
                (trans,rot) = listener.lookupTransform('hand_palm_link', 'map',rospy.Time())
                pub.publish(Header(frame_id='hand_palm_link'),Pose(Point(0,0,0),Quaternion(rot[3],rot[0],rot[1],rot[2])*ori))
                self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,-1])), pull_distance, ref_frame_id='hand_palm_link')

                ## let go and return to safe pose
                self.gripper.command(1.2)
                # self.whole_body.move_end_effector_by_line((Quaternion(rot[3],rot[0],rot[1],rot[2])*ori).rotate(np.array([0.0,0.0,-1])), SAFE_DISTANCE, ref_frame_id='hand_palm_link')        

        def get_goal_cropped_collision_map(goal, crop_dist_3d=0.1):
            """
            Get a 2m*2m*2m collision map centred on goal_tf, with the area around goal_tf
            cropped out to a distance of crop_dist_3d.
            """

            goal_x = goal.x
            goal_y = goal.y
            goal_z = goal.z

            external_bounding_box = BoundingBox(
                min=Point(goal_x - 1.0, goal_y - 1.0, goal_z - 1.0),
                max=Point(goal_x + 1.0, goal_y + 1.0, goal_z + 1.0),
            )

            crop_bounding_boxes = []
            # NOTE this is a hard-coded axis-aligned goal bounding box
            if crop_dist_3d > 0:
                bound_x = bound_y = bound_z = crop_dist_3d
                object_bounding_box = BoundingBox(
                    min=Point(goal_x - bound_x, goal_y - bound_y, goal_z - bound_z),
                    max=Point(goal_x + bound_x, goal_y + bound_y, goal_z + bound_z),
                )
                crop_bounding_boxes.append(object_bounding_box)

            (trans,rot) = listener.lookupTransform('map','head_rgbd_sensor_rgb_frame' ,rospy.Time())
            bound_x = bound_y = bound_z = 0.01
            head_bounding_box = BoundingBox(
                min=Point(trans[0] - bound_x, trans[1] - bound_y, trans[2] - bound_z),
                max=Point(trans[0] + bound_x, trans[1] + bound_y, trans[2] + bound_z),
            )
            crop_bounding_boxes.append(head_bounding_box)
            bound_x = bound_y = bound_z = 0.01
            floor_bounding_box = BoundingBox(
                min=Point(goal_x - 1, goal_y - 1, -bound_z),
                max=Point(goal_x + 1, goal_y + 1, bound_z),
            )
            crop_bounding_boxes.append(floor_bounding_box)

            collision_world = collision_mapper.build_collision_world(
                external_bounding_box, crop_bounding_boxes=crop_bounding_boxes
            )

            return collision_world
        global live_poses, live_time
        live_poses = None
        live_time = 0
        i=1
        listener = tf.TransformListener()
        transformer = tf.Transformer()
        pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

        pose_sub = rospy.Subscriber("yolo_pointcloud/detection_poses", PoseArray, pose_cb, queue_size=1)
        targets=[np.array([1.05,0,0.25]),np.array([1.05,0,0.5]),np.array([1.05,0,0.5]),np.array([1.05,0,0.25]),np.array([1.05,0,0.7]),np.array([1.05,0,0.7])]
        collision_mapper = CollisionMapper(self.robot)
        self.whole_body.move_to_neutral()
        move_base(True)
        look_at_hand(False)
        self.gripper.command(1.2)
        self.base_controller.go_abs(0, 0, 0, timeout=40.0)
        self.whole_body.gaze_point(targets[i], ref_frame_id='map')
        print('planning action..')
        open(targets[i],pull_dis)
        self.whole_body.move_to_neutral()

    def publish_goal_pose_tf(
        self, p, goal_pose_name, ori=None
    ):
        t = Transform()
        t.translation.x = p[0]
        t.translation.y = p[1]
        t.translation.z = p[2]
        if ori is None:
            t.rotation.x = 0
            t.rotation.y = 0
            t.rotation.z = 0
            t.rotation.w = 1
        else:
            t.rotation=ori
        # self.publish_tf(t, 'z1/wrist_camera_link', goal_pose_name)
        self.publish_tf(t, 'map', goal_pose_name)

    def publish_tf(self, transform, source_frame_id, child_frame_id):
        """
        Convenience function to publish a transform.
        Args:
            transform: geometry_msgs Transform type, from source_frame to child_frame
            source_frame_id: name of source frame
            child_frame_id: name of child frame
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source_frame_id
        t.child_frame_id = child_frame_id
        t.transform = transform
        self.tf_broadcaster.sendTransform(t)

        # Wait until transform is available
        return self._tf_buffer.can_transform(
            source_frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(1.0)
        )
    
    def joint_states_cb(self,data):
        self.joint_states_position = data.position[:6]

    def send_joint_command(self,joints_position):
        rospy.loginfo('Sending joint commands: ')
        print(joints_position)

        dt = 0.02 #Iteration time-step 0.02 corresponds to 50Hz control rate
        
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]

        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = joints_position.copy()
        trajectory_point.time_from_start = rospy.Duration.from_sec(dt)

        trajectory_msg.points.append(trajectory_point)
        self.motion_plan_publisher.publish(trajectory_msg)


if __name__=='__main__':
    rospy.init_node('anytree_interface_node')
    robot = AnytreeInterface()
    # robot.turn('right')
    # robot.pick_up_object_client('apple_0')
    # robot.pick_up_object_client('cup_0')
    # robot.put_object_on_surface_client([-1, 1.2, 0.6])
    # robot.send_base_goal([0,0,0.57,0,0,1.57],is_relative_pose=False)
    # robot.put_object_on_surface_client([0.6, -0.1, 0.72])
    # robot.open_drawer()
    robot.search('none','none')