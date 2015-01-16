
import rospy
import tf
import tf_conversions.posemath as tfconv
import PyKDL

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import compute_effector_camera_quick

class HandEyeConnector(object):
    def __init__(self):

        # Frame which is rigidly attached to the camera
        # The transform from this frame to the camera optical frame is what
        # we're trying to compute.
        # For the eye-in-hand case, this is the end-effector frame.
        # For the eye-on-base case, this is the world or base frame.
        self.camera_parent_frame_id = rospy.get_param('~camera_parent_frame')

        # Frame which is rigidly attached to the marker
        # The transform from the camera parent frame to the marker parent frame
        # is given by forward kinematics.
        # For the eye-in-hand case, this is the world or base frame.
        # For the eye-on-base case, this is the end-effector frame.
        self.marker_parent_frame_id = rospy.get_param('~marker_parent_frame')

        self.publish_tf = rospy.get_param('~publish_tf')
        self.tf_suffix = rospy.get_param('~tf_suffix')
        self.sample_rate = rospy.get_param('~sample_rate')
        self.interactive = rospy.get_param('~interactive')

        # Compute the camera base to optical transform
        self.xyz_optical_base = rospy.get_param('~xyz_optical_base', [0,0,0])
        self.rpy_optical_base = rospy.get_param('~rpy_optical_base', [0,0,0])
        self.F_optical_base = PyKDL.Frame(
                PyKDL.Rotation.RPY(*self.rpy_optical_base),
                PyKDL.Vector(*self.xyz_optical_base))
        self.F_base_optical = self.F_optical_base.Inverse()

        # tf structures
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # rate limiter
        self.rate = rospy.Rate(self.sample_rate)

        # input data
        self.hand_world_samples = TransformArray()
        self.camera_marker_samples = TransformArray()

        # marker subscriber
        self.aruco_subscriber = rospy.Subscriber(
                'aruco_tracker/transform',
                TransformStamped,
                self.aruco_cb,
                queue_size=1)

        # calibration service
        rospy.wait_for_service('compute_effector_camera_quick')
        self.calibrate = rospy.ServiceProxy(
                'compute_effector_camera_quick',
                compute_effector_camera_quick)

    def compute_calibration(self, msg):
        rospy.loginfo("Computing from %g poses..." % len(self.hand_world_samples.transforms) )
        result = None

        # Get the camera optical frame for convenience
        optical_frame_id = msg.header.frame_id

        try:
            result = self.calibrate(self.camera_marker_samples, self.hand_world_samples)
        except rospy.ServiceException as ex:
            rospy.logerr("Calibration failed: "+str(ex))
            return None

        if self.publish_tf:
            self.broadcaster.sendTransform(
                    (result.effector_camera.translation.x, result.effector_camera.translation.y, result.effector_camera.translation.z),
                    (result.effector_camera.rotation.x, result.effector_camera.rotation.y, result.effector_camera.rotation.z, result.effector_camera.rotation.w),
                    rospy.Time.now(),
                    optical_frame_id + self.tf_suffix,
                    self.camera_parent_frame_id)

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

        return result

    def aruco_cb(self, msg):

        rospy.loginfo("Received marker sample.")

        # Get the camera optical frame for convenience
        optical_frame_id = msg.header.frame_id

        try:
            # Get the transform between the marker and camera frames (from FK)
            self.listener.waitForTransform(
                self.marker_parent_frame_id, self.camera_parent_frame_id,
                msg.header.stamp, rospy.Duration(0.1))

            (trans,rot) = self.listener.lookupTransform(
                self.marker_parent_frame_id, self.camera_parent_frame_id,
                msg.header.stamp)
        except tf.Exception as ex:
            rospy.logwarn(str(ex))
            return

        # Update data
        self.hand_world_samples.header.frame_id = optical_frame_id
        self.hand_world_samples.transforms.append(Transform(Vector3(*trans), Quaternion(*rot)))

        self.camera_marker_samples.header.frame_id = optical_frame_id
        self.camera_marker_samples.transforms.append(msg.transform)

        if len(self.hand_world_samples.transforms) != len(self.camera_marker_samples.transforms):
            rospy.logerr("Different numbers of hand-world and camera-marker samples.")
            return

        n_min = 2
        if len(self.hand_world_samples.transforms) < n_min:
            rospy.logwarn("%d more samples needed..." % (n_min-len(self.hand_world_samples.transforms)))
        else:
            self.compute_calibration(msg)

        # interactive
        if self.interactive:
            i = raw_input('Hit [enter] to accept this latest sample, or `d` to discard: ')
            if i == 'd':
                del self.hand_world_samples.transforms[-1]
                del self.camera_marker_samples.transforms[-1]
                self.compute_calibration(msg)
            raw_input('Hit [enter] to capture the next sample...')
        else:
            self.rate.sleep()
