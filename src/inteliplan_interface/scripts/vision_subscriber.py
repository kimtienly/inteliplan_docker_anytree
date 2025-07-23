'''
This is the vision module to verify the presence of the target object.
The module communicates with orion_recognition to get the list of detected objects with its corresponding postition.
'''
from orion_actions.msg import Detection, DetectionArray
import rospy
import tf2_ros
import tf.transformations as transformations
import numpy as np
import geometry_msgs.msg


class DetectionSubscriber:
    def __init__(self):
        self.objects_in_scene = {}
        self.detected_pose = {}
        self.tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1)
        self.__detection_sub = rospy.Subscriber('/vision/bbox_detections', DetectionArray, self.detection_cb, queue_size=1)
        # self.camera_frame = "z1/wrist_camera_link"
        self.camera_frame = "z1/wrist_camera_depth_optical_frame" # this is the header.frame_id of "/z1/wrist_camera/aligned_depth_to_color/image_raw", read in a similar way to from detection_tf_publisher
        self.global_frame = "map"
        self.cache_query_object =""
        rospy.loginfo("DetectionSubscriber -- initialised")

    def detection_cb(self,_detections):
        for detection in _detections.detections:
            if detection.label.confidence < 0.5:
                continue

            if detection.label.name not in self.objects_in_scene:
                try:
                    # Get camera to global (e.g., map) transform
                    camera_to_global = self.tfBuffer.lookup_transform(
                        self.global_frame,
                        self.camera_frame,
                        rospy.Time(0),
                        rospy.Duration(1))

                    # Convert camera-to-global transform to matrix
                    cam_trans = [
                        camera_to_global.transform.translation.x,
                        camera_to_global.transform.translation.y,
                        camera_to_global.transform.translation.z
                    ]
                    cam_rot = [
                        camera_to_global.transform.rotation.x,
                        camera_to_global.transform.rotation.y,
                        camera_to_global.transform.rotation.z,
                        camera_to_global.transform.rotation.w
                    ]
                    cam_tf = transformations.quaternion_matrix(cam_rot)
                    cam_tf[0:3, 3] = cam_trans

                    # Pose of the object in camera frame from t (translation + quaternion)
                    obj_trans = [
                        detection.translation_x,
                        detection.translation_y,
                        detection.translation_z
                    ]
                    obj_rot = [0.5, -0.5, 0.5, 0.5]  # As given

                    obj_tf = transformations.quaternion_matrix(obj_rot)
                    obj_tf[0:3, 3] = obj_trans

                    # Compose: global_pose = cam_to_global * obj_in_cam
                    global_tf = np.dot(cam_tf, obj_tf)

                    # Store the global position (translation component only)
                    global_position = global_tf[0:3, 3]
                    self.objects_in_scene[detection.label.name] = global_position

                except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn(f"TF lookup failed: {e}")
               
    def detection_cb_old(self, _detections):
        for detection in _detections.detections:
            if detection.label.confidence < 0.5:
                continue

            if not (detection.label.name in self.objects_in_scene):                 
                # Getting the transform from :
                camera_to_global = self.tfBuffer.lookup_transform(
                    self.global_frame,
                    self.camera_frame, 
                    rospy.Time(0),
                    rospy.Duration(1))
                
                # Create a transform matrixswwep
                translation = [camera_to_global.transform.translation.x,
                               camera_to_global.transform.translation.y,
                               camera_to_global.transform.translation.z]

                rotation = [camera_to_global.transform.rotation.x,
                            camera_to_global.transform.rotation.y,
                            camera_to_global.transform.rotation.z,
                            camera_to_global.transform.rotation.w]
                
                matrix = transformations.quaternion_matrix(rotation)
                matrix[0:3, 3] = translation

                pose_array = [detection.translation_x-0.05, 
                              detection.translation_y, 
                              detection.translation_z,
                              1]
                
                transformed_pose_array = np.dot(matrix, pose_array)
                self.objects_in_scene[detection.label.name] = transformed_pose_array

    def is_seen(self, input_string):
        self.cache_query_object=''
        x = input_string.split()
        for t in x:
            if t in self.objects_in_scene:
                self.detected_pose[t] = self.objects_in_scene[t]
                self.cache_query_object =t
                return True
        return False

    def get_pose(self, target=None): 
        if target is None:
            target = self.cache_query_object
            if target == '':
                print('Vision cannot find object pose due to no object specified or cached.')
                return None
        if target in self.objects_in_scene:
            print('Detected pose of {} is {}'.format(target,self.objects_in_scene[target][:3]))

        return self.detected_pose[target]

if __name__ == '__main__':
    rospy.init_node('detection_subscriber')
    detection_sub = DetectionSubscriber()
    rospy.spin()