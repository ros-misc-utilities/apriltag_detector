# ROS Apriltag detector package

![banner image](images/apriltag_detections.png)

This repository has ROS/ROS2 nodes and nodelets/components for
detecting Apriltags using the [Apriltag 3](https://github.com/AprilRobotics/apriltag) library.

## Supported platforms

Currently tested on Ubuntu 20.04 under under ROS Noetic and ROS2 Galactic.

## How to build
Create a workspace (``~/ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
pkg=apriltag_detector
mkdir -p ~/$pkg/src
cd ~/ws
git clone https://github.com/berndpfrommer/${pkg}.git src/${pkg}
wstool init src src/${pkg}/${pkg}.rosinstall
# to update an existing space:
# wstool merge -t src src/${pkg}/${pkg}.rosinstall
# wstool update -t src
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
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
- ``num_threads``. Number of threads on which the Apriltag library
  will operate. Defaults to 1.


## License

This software is issued under the Apache License Version 2.0.
