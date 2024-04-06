# ROS MIT Apriltag detector package

This repository has ROS2 components for detecting Apriltags using the
[MIT apriltag library implementation](https://people.csail.mit.edu/kaess/apriltags).

For documentation also see [the base repository](https://github.com/ros-misc-utilities/apriltag_detector).

## Supported platforms

Should work on Ubuntu under all ROS2 distros starting with Humble.

## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-apriltag-detector-mit
```

### From source

Set the following shell variables:
```bash
repo=apriltag_detector_mit
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## How to run

```
ros2 launch apriltag_detector_mit detector.launch.py camera:=my_camera_name
ros2 run rqt_image_view rqt_image_view
```

## Topics

- ``~/image``: (subscribe) image topic.
- ``~/tags``: (publish) tag detections.

## Parameters

- ``black_border_width``. Integer specifying the thickness of the black border (in bits). Default: 1.
- ``image_qos_profile``. QoS profile for image subscription. Allowed values: ``default``, `` sensor_data``.
    Use this parameter to achieve QoS compatibility when subscribing to image data. Defaults to ``default``. 
- ``image_transport``. The type of image transport to use, e.g. ``compressed``. Default: ``raw``.
- ``tag_family``. Apriltag family. Allowed values: ``tf16h5``, ``tf25h9``, ``tf36h11``. Default: ``tf36h11``.

## License

This software is issued under the Apache License Version 2.0.
