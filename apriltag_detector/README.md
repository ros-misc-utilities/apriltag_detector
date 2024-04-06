# ROS Apriltag detector package

![banner image](images/apriltag_detections.png)

This package has the base class header files for apriltag detector plugins
for the [UMich](../apriltag_detector_umich/README.md) and the
[MIT](../apriltag_detector_mit/README.md) detector.

It also has launch files for both of these detectors, so by installing this package
you can use both of these detectors.

## Supported platforms

Should work on Ubuntu under all ROS2 distros starting with Humble.

## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-apriltag-detector
```

### From source

Set the following shell variables:
```bash
repo=apriltag_detector
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## How to run

```
ros2 launch apriltag_detector detector.launch.py type:=umich camera:=my_camera_name
ros2 run rqt_image_view rqt_image_view
```
Then publish image under topic ``/my_camera_name/image_raw``.

NOTE: you must subscribe to the tags topic or else the detector will not subscribe to images!

## Topics

- ``~/image``: (subscribe) image topic.
- ``~/tags``: (publish) tag detections.

## Launch file arguments

- ``type``: Detector type to launch ``mit`` or ``umich``. Default: ``umich``.
- ``image_qos_profile``: QoS profile for image subscription. Allowed values: ``default``, `` sensor_data``.
    Use this parameter to achieve QoS compatibility when subscribing to image data. Defaults to ``default``. 
- ``image_transport``: The type of image transport to use, e.g. ``compressed``. Default: ``raw``.
- ``tag_family``: Apriltag family. Allowed values: ``tf16h5``, ``tf25h9``, ``tf36h11``. Default: ``tf36h11``.
    Note: The UMich detector supports additional families.

### additional arguments when launching the MIT detector:

- ``blur``: Sigma (in pixels) for gaussian blur. Default: 0 (no blur).

### additional arguments when launching the UMich detector:

- ``blur``: Sigma (in pixels) for gaussian blur. Default: 0 (no blur).
- ``decimate_factor``: Decimate image by this amount. Factor 2 will decimate by 2 in both directions.
    Default: 1 (no decimation).
- ``max_allowed_hamming_distance``: discard tags with hamming error larger than this.
    Default: 0 (no error allowed).
- ``num_threads``: number of threads to use for processing. Default: 1.
- ``tag_family``: Additional families allowed:
  ``tfCircle21h7``, ``tfCircle49h12``, ``tfStandard41h12``, ``tfStandard52h13``, ``tfCustom48h12``.

## License

This software is issued under the Apache License Version 2.0.
```

### configure and build on ROS2:

```bash
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## How to use

Examine the launch file and adjust the topic remapping, tag family
etc. Then start as follows (assuming your camera publishes under /my_camera_name/image_raw):

ROS1:
```
roslaunch apriltag_detector node.launch camera:=my_camera_name
rqt_image_view
```
The detector publishes a debug image and a tag topic.


ROS2:
```
ros2 launch apriltag_detector node.launch.py camera:=my_camera_name
ros2 run rqt_image_view rqt_image_view
```

Parameters:

- ``tag_family``. Apriltag family, something like "tf36h11".
- ``max_hamming_distance``. Maximum allowable hamming distance
  (defaults to 0).
- ``decimate_factor``. By how much to decimate the image to speed up
  detection. Defaults to 1 (no decimation). To half the resolution,
  use ``decimate_factor=2``.
- ``blur``. Gaussian blur to apply. Defaults to 0 (no blur). Can be
  negative to sharpen.
- ``image_qos_profile``. QoS profile of image messages that are subscribed to. Defaults to ``default``, but can also be set to ``sensor_data``. Use this parameter to achieve QoS compatibility when subscribing to image data.
- ``num_threads``. Number of threads on which the Apriltag library
  will operate. Defaults to 1.


## License

This software is issued under the Apache License Version 2.0.
