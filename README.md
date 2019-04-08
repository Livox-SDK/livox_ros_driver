# livox ros driver

### Run livox ros driver

livox_ros_driver is a new ros package under the Livox-SDK/Livox-SDK-ROS/src directory, which is designed to gradually become the standard driver package for livox devices in the ros environment. The driver offers users a wealth of options:

1. Publish pointcloud2 format point cloud and automatically load rviz；

for example：

```
roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
```

or

```
roslaunch livox_ros_driver livox_hub_rviz.launch bd_list:="hub_broadcast_code" 
```

2. Publish pointcloud2 format point cloud only；

for example：

```
roslaunch livox_ros_driver livox_lidar.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
```

or

```
roslaunch livox_ros_driver livox_hub.launch bd_list:="hub_broadcast_code"
```



3. Publish livox custom format point cloud；

for example：

```
roslaunch livox_ros_driver livox_lidar_msg.launch bd_list:="broadcast_code1&broadcast_code2&broadcast_code3"
```

or

```
roslaunch livox_ros_driver livox_hub_msg.launch bd_list:="hub_broadcast_code"
```

livox custom msg format：

```
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 timestep           # Time interval between adjacent point clouds
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```
pointcloud format:
```
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 line              # laser number in lidar
```
