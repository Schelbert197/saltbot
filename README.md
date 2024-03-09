# Saltbot
**Author: Srikanth Schelbert**
NUMSR Winter Project Creating a Salt Distributing Robot

This repository contains two packages that can be used in tandem to use the Clearpath Jackal to salt courtyards. The first package `saltbot_nav` is a general path planner for a lawnmower style path in an open space, and the second package `saltbot_nav_cpp` is an interface node for using Nav2.

## Prerequisites:
- Ubuntu 22.04
- Python3
- C++
- ROS2 Humble
- Clearpath Jackal running the same as above

## Quick Start
1. First, ssh into the robot in two separate terminals using `ssh -oSendEnv=ROS_DOMAIN_ID jackal@jackal-desktop`
2. In the first terminal run `ros2 launch jackal_3d_slam jackal_transform.launch.py use_filtered:=true` to launch RTABmap with filtered point cloud data. Otherwise, run `ros2 launch jackal_3d_slam jackal_transform.launch.py use_unfiltered:=true`.
    - Wait for this to completely load up so that the navigation does not have an issue.
3. In the second ssh terminal run `ros2 launch jackal_3d_slam start_3d_slam.launch.xml filter:=true` to filter the point cloud. Otherwise, run `ros2 launch jackal_3d_slam start_3d_slam.launch.xml`.
4. In another terminal on your machine, run the launchfile with `ros2 launch saltbot_nav saltbot_visual.launch.xml` to start the rviz2 window (set for Nav2), the map_slicer node, and the nav_node Nav2 interface node. 
5. If you are creating a map, move the robot so that RTABmap publishes another map.
6. In a final (fourth) terminal window, run the service call `ros2 service call lean_waypoints std_srvs/srv/Empty` to create the waypoints.
7. Once the waypoints have been published and can be seen in Rviz (this should only take a few second), you can call the service `ros2 service call travel std_srvs/srv/Empty` to start the robot navigation. 
    If you need to cancel at any time, you can call `ros2 service call saltbot_cancel_nav std_srvs/srv/Empty` which will stop the action without killing any nodes or using the E-stop button (which will require re-engaging the motors on the Jackal again).