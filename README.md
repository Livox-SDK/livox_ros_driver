# Livox ROS Driver

livox_ros_driver is a new ros package, which is designed to gradually become the standard driver package for livox devices in the ros environment.

### Compile & Install Livox SDK

livox_ros_driver depends on Livox-SDK lib. If you have never installed Livox-SDK lib or it is out of date, you must first install Livox-SDK lib. If you have installed the latest version of Livox-SDK, skip this step and go to the next step.

1. Download or clone the [Livox-SDK/Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/) repository on GitHub.
2. Compile and install the Livox-SDK under the ***build*** directory following `README.md` of Livox-SDK/Livox-SDK.

### Clone livox_ros_driver

1. Clone livox_ros_driver package for github:

   `git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src`

2. Build livox_ros_driver package:

```
  cd ws_livox
  catkin_make
```

3. Package environment setup:

​        `source ./devel/setup.sh`

### Run livox ros driver

The driver offers users a wealth of options when using different launch file. There is *bd_list* arg in each launch file. All Livox LiDAR units in your LAN will be connected automatically in default.

e.g.

```
roslaunch livox_ros_driver livox_lidar_rviz.launch
```

If you want to connect to the specific LiDAR uint(s) only, please add the broadcast code into command line. 

e.g.

```
roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
```

Features of *.launch* files are listed as below:

| launch file               | features                                                     |
| ------------------------- | ------------------------------------------------------------ |
| *livox_lidar_rviz.launch* | Connect to Livox LiDAR units<br/>Publish pointcloud2 format point cloud<br/>Automatically load rviz |
| livox_hub_rviz.launch     | Connect to Livox Hub units<br/>Publish pointcloud2 format point cloud<br />Automatically load rviz |
| livox_lidar.launch        | Connect to Livox LiDAR units<br />Publish pointcloud2 format point cloud |
| livox_hub.launch          | Connect to Livox Hub units<br />Publish pointcloud2 format point cloud |
| livox_lidar_msg.launch    | Connect to Livox LiDAR units<br />Publish livox custom format point cloud |
| livox_hub_msg.launch      | Connect to Livox Hub units<br />Publish livox custom format point cloud |

Livox custom msg format：

```
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```
pointcloud format:
```
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar
```
