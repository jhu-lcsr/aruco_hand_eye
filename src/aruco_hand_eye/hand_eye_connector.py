
import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray 
from visp_hand2eye_calibration.srv import compute_effector_camera_quick 

class HandEyeConnector(object):
    def __init__(self):

        self.camera_marker_frame = rospy.get_param('~camera_marker_frame')
        self.hand_marker_frame = rospy.get_param('~hand_marker_frame')
        self.publish_tf = rospy.get_param('~publish_tf')
        self.tf_suffix = rospy.get_param('~tf_suffix')
        self.xyz_optical_base = rospy.get_param('~xyz_optical_base', [0,0,0])
        self.rpy_optical_base = rospy.get_param('~rpy_optical_base', [0,0,0])
        self.F_optical_base = PyKDL.Frame(
                PyKDL.Rotation.RPY(*self.rpy_optical_base),
                PyKDL.Vector(*self.xyz_optical_base))
        self.F_base_optical = self.F_optical_base.Inverse()

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.aruco_subscriber = rospy.Subscriber(
                'aruco_tracker/transform', 
                TransformStamped, 
                self.aruco_cb, 
                queue_size=1)

        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
                'compute_effector_camera_quick', 
                compute_effector_camera_quick)

        self.rate = rospy.Rate(2)
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()


    def aruco_cb(self, msg):

        rospy.loginfo("Received marker sample.")

        parent_frame_id = msg.header.frame_id

        try:
            self.listener.waitForTransform(self.hand_marker_frame, '/world', msg.header.stamp, rospy.Duration(0.1))
            (trans,rot) = self.listener.lookupTransform(self.hand_marker_frame, '/world', msg.header.stamp)
        except tf.Exception as ex:
            rospy.logwarn(str(ex))
            return
        
        self.hand_world_samples.header.frame_id = parent_frame_id
        self.hand_world_samples.transforms.append(Transform(Vector3(*trans), Quaternion(*rot)))

        self.camera_marker_samples.header.frame_id = parent_frame_id
        self.camera_marker_samples.transforms.append(msg.transform)

        if len(self.hand_world_samples.transforms) != len(self.camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples.")
            return

        n_min = 2
        if len(self.hand_world_samples.transforms) < n_min:
            rospy.logerr("%d more samples needed..." % (n_min-len(self.hand_world_samples.transforms)))
            return

        rospy.loginfo("Computing from %g poses..." % len(self.hand_world_samples.transforms) )
        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: "+str(ex))
            return

        if self.publish_tf:
            self.broadcaster.sendTransform(
                    (result.effector_camera.translation.x, result.effector_camera.translation.y, result.effector_camera.translation.z),
                    (result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w),
                    rospy.Time.now(),
                    parent_frame_id + self.tf_suffix,
                    'world')

        rospy.loginfo("Result:\n"+str(result))

        ec = result.effector_camera
        xyz = (ec.translation.x, ec.translation.y, ec.translation.z)
        xyzw = (ec.rotation.x, ec.rotation.y, ec.rotation.z, ec.rotation.w)
        rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        F_optical_world = PyKDL.Frame(PyKDL.Rotation.Quaternion(*xyzw), PyKDL.Vector(*xyz))
        F_base_world = F_optical_world * self.F_base_optical

        bw = tfconv.toMsg(F_base_world)
        xyz = (bw.position.x, bw.position.y, bw.position.z)
        xyzw = (bw.orientation.x, bw.orientation.y, bw.orientation.z, bw.orientation.w)
        rpy = tuple(PyKDL.Rotation.Quaternion(*xyzw).GetRPY())

        rospy.loginfo("Base xyz: ( %f %f %f ) rpy: ( %f %f %f ) xyzw: ( %f %f %f %f )" % (xyz+rpy+xyzw))

        self.rate.sleep()
    

