# wonik_project

export GAZEBO_MODEL_PATH=/home/home/wonik_ws/src/wonik_simulation/models:$GAZEBO_MODEL_PATH

[For noetic]
sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-navigation ros-noetic-joy ros-noetic-teleop-twist-keyboard  ros-noetic-amcl ros-noetic-openslam-gmapping  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-sick-scan ros-noetic-robot-localization ros-noetic-hector-gazebo-plugins -qy 

[For melodic]
sudo apt-get install ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-navigation ros-melodic-joy ros-melodic-teleop-twist-keyboard  ros-melodic-amcl ros-melodic-openslam-gmapping  ros-melodic-tf2-sensor-msgs ros-melodic-tf2-geometry-msgs ros-melodic-sick-scan ros-melodic-robot-localization ros-melodic-hector-gazebo-plugins -qy


## For simulation
roslaunch wonik_simulation simulation.launch

## For real robot
roslaunch wonik_real bringup.launch
