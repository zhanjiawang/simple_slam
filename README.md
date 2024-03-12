# simple_slam
This is a simple SLAM code for generating 2D grid maps based on 3D laser data and 3D laser odometry.

## usage
You can try various 3D laser odometers (if it is a 2D laser odometer, then removing the 3D laser to scan data part of simple_slam should also enable 2D mapping work). Simple_slam requires the laser odometer to input TF information, and then combine the laser data to construct a 2D grid map.

## demo
Taking FAST_LIO as an example, this is a 3D laser odometer and also a 3D mapping method. If Livox mid360 is used as the laser input for FAST_LIO, the process of constructing a 2D grid map by running simple_slam based on this is as follows.

### depends
1. FAST_LIO
2. livox_repub (For the livox_ros_driver::CustomiMsg data of Livox LiDAR, if your LiDAR data type is sensor_msgs::PointCloud2, this item may not be required)
3. livox_ros_driver （For Livox LiDAR, if your LiDAR data type is sensor_msgs:: PointCloud2, then this item is not required）

### install
1. Create a src directory and place simple_slam and FAST_LIO, livox_repub, and livox_ros_driver in the src directory
2. catkin_make

### run
1. roslaunch simple_slam start.launch
