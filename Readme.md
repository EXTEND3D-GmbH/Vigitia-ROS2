
<picture>
  <source media="(prefers-color-scheme: light)" srcset="https://www.extend3d.com/wp-content/themes/extend3d/dist/images/LOGO.svg">
  <source media="(prefers-color-scheme: dark)" srcset="https://user-images.githubusercontent.com/74293493/170223696-34f5027b-a37a-4b78-bd5d-164381ff5b80.svg">
  <img alt="Extend3D GmbH Logo">
</picture>

# Vigitia
![Vigita-Skizze](https://user-images.githubusercontent.com/74293493/169037302-4e572fdb-9c26-44a8-b6fd-9bb88573f824.png)

Im Verbundprojekt VIGITIA untersuchen die Projektpartner, wie projizierte erweiterte Realität (projected augmented reality / PAR) Alltagsinteraktionen rund um Tische unterstützen und bereichern kann.

Bei PAR nehmen eine oder mehrere Kameras in Echtzeit die Tischoberfläche und darauf befindliche Objekte auf. Ein an der Decke oder an einem Schwenkarm befestigter Projektor kann dadurch millimetergenau und ohne Verzerrungen auf die Tischplatte bzw. die Objekte projizieren. Dies erlaubt es, zusätzliche Informationen und Interaktionsmöglichkeiten zu den Objekten einzublenden und analoge Arbeitsprozesse zu unterstützen. Parallel dazu besteht dennoch die Möglichkeit, bereits vorhandene, intelligente Gegenstände wie z.B. Smartphones, Tablets oder Laptops nahtlos in das System zu integrieren, um beispielsweise über die Projektion als Vermittler Daten zwischen den Geräten auszutauschen oder eine größere Interaktionsfläche für mehrere Personen zu bieten, als der kleine Smartphone-Bildschirm es ermöglicht. Hierbei tritt die Technik in den Hintergrund und stattdessen rückt das gemeinsame Erleben in den Vordergrund, welches durch die Technik nur auf Wunsch unterstützt wird.

# Description
This repository contains solutions to calibrate an RGBD Kinect camera to an arbitrary projector as well as a rudimentary drawing app based on touch tracking. The touch tracker paint app is Windows specific as it injectes detected touches into the OS to allow the user to use the desktop environment.

## Needed packages (Windows)
Start an x64 Development Console and source ROS2 e.g.: `C:\opt\ros\foxy\x64\setup.bat`

Install `numpy-quaternion` via: `python -m pip install --upgrade --force-reinstall numpy-quaternion`

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
  - Do not forget to set the correct launch arguments on the command line or in the launch file for the calibration files that were saved during calibration.
- The `static_pose_injector` will inject the projector calibration on the appropriate topics
  - Two launch parameters can be set to define the path to the projector intrinsics and extrinsics
- `degginger_draw` will spawn a window which will listen to published touch events and inject them into windows events, such that they are received on the same monitor/projector on which the window is visible.
- `touch_tracker` uses the Azure Kinect to detect touches and publishes them
  - Internally it uses a leanring background which takes 5-10 seconds to completely refresh on a static scene
  - To fix the background (recommended) you can use a service call `ros2 service call /background_node/set_fix_background std_srvs/srv/SetBool "{data: true}"`

### Typical issues

#### k4a.dll not found
Windows CMake from the AzureKinect Driver sometimes fails to copy the appropriate dlls on install. The needed dlls can be found in `src\Azure_Kinect_ROS_Driver-foxy-devel\install\azure_kinect_ros_driver\lib\azure_kinect_ros_driver\` and be placed next to the `node.exe` e.g. in `install\Lib\azure_kinect_ros_driver\`

#### Touch detection is not working
The touch detection has a bunch of settings which can be adjusted in `touch_tracker/inlcude/Settings.h` to get precise and consistent results. For more information please see `DIRECT: Practical Touch Tracking on Surfaces with Hybrid Depth-Infrared Sensing (ISS ’16) - Robert Xiao`

#### Calibration
The calbiration can be checkd via `rviz2` and visualizing the TF frames
