# turtlebot_mrs
Multi-Robot System for Collaborative Object Transport

Integrated with the following:

1. RPLIDAR A2 for use with the ROS Navigation Stack
2. Orbec Astra Camera for use with the ROS ar-track-alvar package
3. Electromagnets for robot docking
4. State machine with the SMACH state machine package


## Master Setup
- 

## Slave Setup
- In the `src` folder clone the following packages<br />

ar_track_alvar: `git clone https://github.com/ros-perception/ar_track_alvar.git`<br />
rplidar_ros: `git clone https://github.com/Slamtec/rplidar_ros.git`<br />
rosserial: `git clone https://github.com/ros-drivers/rosserial.git`<br />

- Install the following<br />

`sudo apt-get install ros-kinetic-rosserial-arduino`<br />
`sudo apt-get install ros-kinetic-rosserial`<br />

- Go to the catkin_ws folder and run `catkin_make`

## Running the Master
- Run the following `python sm_mrscot_demo.py` to start the state machine

## Running the Slave
- Run the following `roslaunch all.launch scot_name:=robot_1` where `scot_name` is given an arbitrary name that is different from other robots
- This launch file launches the following packages: gmapping, navigation-stack....

