# wonik_project

export GAZEBO_MODEL_PATH=/home/home/wonik_ws/src/wonik_simulation/models:$GAZEBO_MODEL_PATH

sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-navigation ros-noetic-joy ros-noetic-teleop-twist-keyboard  ros-noetic-amcl -qy


roslaunch wonik_simulation simulation.launch
