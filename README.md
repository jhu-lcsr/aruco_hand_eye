
ARUCO / VISP Hand-Eye Calibration
=================================

![rviz wam screenshot](doc/aruco_hand_eye_wam.png)

## Use Cases

This package uses the ARUCO planar target tracker from `aruco_ros` and the VISP
hand-eye calibration from `visp_hand2eye_calibration` to provide a simple
camera pose estimation package.

If you're unfamiliar with Tsai's hand-eye calibration [1], it can be used in two ways:

- **eye-in-hand** -- To compute the static transform from a robot's
  end-effector to the optical frame of a camera. In this case, the camera is
  mounted on the end-effector, and you place the visual target so that it is
  fixed relative to the base of the rboot.
- **eye-on-base** -- To compute the static transform from a robot's base to the
  optical frame of a camera. In this case, the camera is mounted to the base of
  the robot (or kinematic chain), and you place the visual target so that it is
  fixed relative to the end-effector of the robot.

## Usage

For both use cases, you can either launch the `aruco_hand_eye.launch`
launchfile, or you can include it in another launchfile as shown below. Either
way, the launchfile will bring up the `aruco_ros` tracker and the
`visp_hand2eye_calibration` solver, along with an integration script. By
default, the integration script will interactively ask you to accept or discard
each sample.

### eye-in-hand

```xml
<launch>
  <include file="$(find aruco_hand_eye)/launch/aruco_hand_eye.launch">
    <arg name="markerid"   value="582"/>
    <arg name="markersize" value="0.141"/>
    <arg name="publish_tf" value="true"/>

    <arg name="marker_parent_frame" value="/base_link"/>
    <arg name="camera_parent_frame" value="/ee_link"/>

    <arg name="camera" value="/camera/rgb"/>
    <arg name="camera_frame" value="/camera_rgb_optical_frame"/>
  </include>
</launch>
```

### eye-on-base

```xml
<launch>
  <include file="$(find aruco_hand_eye)/launch/aruco_hand_eye.launch">
    <arg name="markerid"   value="582"/>
    <arg name="markersize" value="0.141"/>
    <arg name="publish_tf" value="true"/>

    <arg name="marker_parent_frame" value="/ee_link"/>
    <arg name="camera_parent_frame" value="/base_link"/>

    <arg name="camera" value="/camera/rgb"/>
    <arg name="camera_frame" value="/camera_rgb_optical_frame"/>
  </include>
</launch>
```

## Examples

For calibrating a kinect mounted to the base of a manipulator that can grasp the target:

```
roslaunch aruco_hand_eye kinect.launch ee_frame:=/my/robot/ee_link
```

## References

[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*
