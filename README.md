# BEV images generator

## Brief overview
This repository is based on the original one by MacAllister Higgins (https://github.com/mjshiggins/ros-examples) ,but provides HSR colorization of the PointCloud or code the encoding with height, density and intensity (Colour visualisation corresponds to the LiDAR values)

# Setup
This is a standard ROS catkin-ized workspace. Don't know what that means? Check out a great intro on ROS [here](http://wiki.ros.org/ROS/Tutorials) or head on over to [Udacity](http://udacity.com) to sign up for the Robotics or Self-Driving Car Engineer Nanodegree.

There's currently only one node for processing LIDAR data. Use "catkin_make" in the root of this repo to build the lidar node, and run with ```rosrun lidar lidar_node``` after installing PCL and ROS (Indigo, preferably).

If you've downloaded any of the datasets for the challenge, you can start using the data with these nodes immediately by running ```rosbag play -l name-of-file.bag```. The "-l" keeps the bag file playing on repeat so that you can keep working on your algorithm without having to mess with the data playback.
