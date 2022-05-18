# Description
This repository contains solutions to calibrate an RGBD Kinect camera to an arbitrary projector as well as a rudimentary drawing app based on touch tracking. The touch tracker paint app is Windows specific as it injectes detected touches into the OS to allow the user to use the desktop environment.

## How to calibrate
- Build the project
- Start the services with `ros2 launch src/launch/calibration.launch.py`
  - This assumes that a Kinect Camera is connected if a different camera is used the following commands will need to be adjusted.
- Move the grey projector window onto the projector (Windows: Win+Shift+ArrowKeys)
- Start the calibration procedure with: `ros2 action send_goal /calibrate calibration_interfaces/action/Calibrate "{projector_topic: /projector, camera_topic: /rgb/image_raw, camera_info_topic: /rgb/camera_info, depth_topic: /depth_to_rgb/image_raw, grid_width: 9, grid_height: 6, projector_width: 2560, projector_height: 1600, depth_scale_to_m: 0.001, save_path: C:/Projects/ros_ws/calib}"`
  - Note the different topics which might need to be adjusted depending on your setup
  - Adjust the projector resolution to the one in question
  - `depth_scale_to_m` is used to convert the images form `depth_topic` to meter. The Azure Kinect produces a depth image in millimeter.
  - `save_path` is optional and describes the location where two xml files (intrinsics and extrinsics) are saved on successful calibration

## How to run the touch tracker
The touch tracker uses the saved xml files from the calibration as input and posts them on the appropriate topics as a camera_info as well as a tf frame.
- Start all nodes with `ros2 launch src/launch/touch.launch.py`
  - Note that either the calibration or the touch tracker can be started at a time in the current configuration
- The `static_pose_injector` will inject the projector calibration on the appropriate topics
- `degginger_draw` will spawn a window which will listen to published touch events and inject them into windows events, such that they are received on the same monitor/projector on which the window is visible.
- `touch_tracker` uses the Azure Kinect to detect touches and publishes them
  - Internally it uses a leanring background which takes 5-10 seconds to completely refresh on a static scene
  - To fix the background (recommended) you can use a service call `ros2 service call /background_node/set_fix_background std_srvs/srv/SetBool "{data: true}"`

### Typical issues
#### Touch detection is not working
The touch detection has a bunch of settings which can be adjusted in `touch_tracker/inlcude/Settings.h` to get precise and consistent results. For more information please see `DIRECT: Practical Touch Tracking on Surfaces with Hybrid Depth-Infrared Sensing (ISS ’16) - Robert Xiao`

#### Calibration
The calbiration can be checkd via `rviz2` and visualizing the TF frames
