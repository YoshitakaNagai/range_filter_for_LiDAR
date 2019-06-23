# range_filter_for_LiDAR

# Requirements
- PCL(>=1.8)

# Environment
- Ubuntu16.04 & ROS kinetic
- Ubuntu18.04 & ROS melodic

# Nodes
## max_range_filter
### Published topics
- /ranged_filtered_pc (sensor_msgs/PointCloud2)
### Subscribed topics
- /lidar_pc (sensor_msgs/PointCloud2)

# How to use
## with bag data
- /lidar_pc

### type
'''
$ cd ros_catkin_ws
$ catkin_make
$ roslaunch range_filter_for_LiDAR max_range_filter
'''

### you can change border range at the launch file
