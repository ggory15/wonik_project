# wonik_project

export GAZEBO_MODEL_PATH=/home/home/wonik_ws/src/wonik_simulation/models:$GAZEBO_MODEL_PATH

sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-navigation ros-noetic-joy ros-noetic-teleop-twist-keyboard  ros-noetic-amcl ros-noetic-openslam-gmapping  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs -qy


roslaunch wonik_simulation simulation.launch
