# ROS UMich Apriltag detector package

This repository has ROS2 components for detecting Apriltags using the
[UMich Apriltag library](https://april.eecs.umich.edu/software/apriltag).

For documentation also see [the base repository](https://github.com/ros-misc-utilities/apriltag_detector).

## Supported platforms

Should work on Ubuntu under all ROS2 distros starting with Humble.

## Installation

### From packages

```bash
sudo apt-get install ros-${ROS_DISTRO}-apriltag-detector-umich
```

### From source

Set the following shell variables:
```bash
repo=apriltag_detector_umich
url=https://github.com/ros-misc-utilities/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## How to run

```
ros2 launch apriltag_detector_umich detector.launch.py camera:=my_camera_name
ros2 run rqt_image_view rqt_image_view
```
Then publish image under topic ``/my_camera_name/image_raw``.

NOTE: you must subscribe to the tags topic or else the detector will not subscribe to images!

## Topics

- ``~/image``: (subscribe) image topic.
- ``~/tags``: (publish) tag detections.

## Parameters

- ``blur``: Sigma (in pixels) for gaussian blur. Default: 0 (no blur).
- ``decimate_factor``: Decimate image by this amount. Factor 2 will decimate by 2 in both directions.
    Default: 1 (no decimation).
- ``image_qos_profile``: QoS profile for image subscription. Allowed values: ``default``, `` sensor_data``.
    Use this parameter to achieve QoS compatibility when subscribing to image data. Defaults to ``default``. 
- ``image_transport``: The type of image transport to use, e.g. ``compressed``. Default: ``raw``.
- ``max_allowed_hamming_distance``: discard tags with hamming error larger than this.
    Default: 0 (no error allowed).
- ``num_threads``: number of threads to use for processing. Default: 1.
- ``tag_family``: Apriltag family. Allowed values: ``tf16h5``, ``tf25h9``, ``tf36h11``,
  ``tfCircle21h7``, ``tfCircle49h12``, ``tfStandard41h12``, ``tfStandard52h13``, ``tfCustom48h12``.
  Default: ``tf36h11``

## License

This software is issued under the Apache License Version 2.0.
