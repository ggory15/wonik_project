# wonik_project

export GAZEBO_MODEL_PATH=$HOME/wonik_ws/src/wonik_project/wonik_simulation/models:$GAZEBO_MODEL_PATH

[For noetic]
sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-navigation ros-noetic-joy ros-noetic-teleop-twist-keyboard  ros-noetic-amcl ros-noetic-openslam-gmapping  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-sick-scan ros-noetic-robot-localization ros-noetic-hector-gazebo-plugins ros-noetic-hector-gazebo ros-noetic-realsense2-camera ros-noetic-realsense2-description ros-noetic-teb-local-planner -qy 

[For melodic]
sudo apt-get install ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-navigation ros-melodic-joy ros-melodic-teleop-twist-keyboard  ros-melodic-amcl ros-melodic-openslam-gmapping  ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs ros-melodic-sick-scan ros-melodic-robot-localization ros-melodic-hector-gazebo-plugins ros-melodic-hector-gazebo ros-melodic-realsense2-camera ros-melodic-realsense2-description ros-melodic-teb-local-planner -qy


## For simulation
roslaunch wonik_simulation simulation.launch

## For real robot
roslaunch wonik_real bringup.launch

## Save map
rosrun map_server map_saver -f mymap
rm $HOME/wonik_ws/src/wonik_project/wonik_real/maps/mymap.*
mv mymap.* $HOME/wonik_ws/src/wonik_project/wonik_real/maps

## Check Accuracy of Movebase
rosrun tf tf_echo /map /base_link
rostopic echo /move_base/goal

## For realsense
To add usb rules.
https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules

roslaunch realsense2_camera rs_camera.launch camera:=cam_1 serial_no:=233522072716 filters:=spatial,temporal,pointcloud
roslaunch realsense2_camera rs_camera.launch camera:=cam_2 serial_no:=238222070999 filters:=spatial,temporal,pointcloud
roslaunch realsense2_camera rs_multiple_devices.launch
