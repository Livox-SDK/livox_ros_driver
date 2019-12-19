# Livox ROS Driver

livox_ros_driver is a new ros package, which is designed to gradually become the standard driver package for livox devices in the ros environment.

## Compile & Install Livox SDK

livox_ros_driver depends on Livox-SDK lib. If you have never installed Livox-SDK lib or it is out of date, you must first install Livox-SDK lib. If you have installed the latest version of Livox-SDK, skip this step and go to the next step.

1. Download or clone the [Livox-SDK/Livox-SDK](https://github.com/Livox-SDK/Livox-SDK/) repository on GitHub.
2. Compile and install the Livox-SDK under the ***build*** directory following `README.md` of Livox-SDK/Livox-SDK.

## Clone livox_ros_driver

1. Clone livox_ros_driver package for github : 

   `git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src`

2. Build livox_ros_driver package :

   ```
   cd ws_livox
   catkin_make
   ```

3. Package environment setup :

   `source ./devel/setup.sh`

## Run livox_ros_driver

##### Run livox_ros_driver using launch file

The command format is : 

 `roslaunch livox_ros_driver [launch file] [param]`

1. Connect LiDAR uint(s) automatically.

​       `roslaunch livox_ros_driver livox_lidar_rviz.launch`

2. Connect to the specific LiDAR uint(s), suppose there are LiDAR(0TFDG3B006H2Z11)  and   LiDAR(1HDDG8M00100191) .

   Specify the LiDAR via command line arguments : 

   ```
   roslaunch livox_ros_driver livox_lidar_rviz.launch bd_list:="0TFDG3B006H2Z11&1HDDG8M00100191"
   ```

​        *Specifying the LiDAR via json file is supported, detailed usage will be explained later.* 

***NOTE:***

Each Livox LiDAR unit owns a unique Broadcast Code . The broadcast code consists of its serial number and an additional number (1,2, or 3). The serial number can be found on the body of the LiDAR unit (below the QR code).The Broadcast Code may be used when you want to connect to the specific LiDAR unit(s).  The detailed format is shown as below : 

![Broadcast Code](images/broadcast_code.png)

#####  Launch file introduction

The driver offers users a wealth of options when using different launch file. The launch file directory    

is "ws_livox/src/livox_ros_driver/launch". All launch files are listed as below : 

| launch file               | features                                                     |
| ------------------------- | ------------------------------------------------------------ |
| livox_lidar_rviz.launch   | Connect to Livox LiDAR units<br/>Publish pointcloud2 format point cloud<br/>Automatically load rviz |
| livox_hub_rviz.launch     | Connect to Livox Hub units<br/>Publish pointcloud2 format point cloud<br />Automatically load rviz |
| livox_lidar.launch        | Connect to Livox LiDAR units<br />Publish pointcloud2 format point cloud |
| livox_hub.launch          | Connect to Livox Hub units<br />Publish pointcloud2 format point cloud |
| livox_lidar_msg.launch    | Connect to Livox LiDAR units<br />Publish livox custom format point cloud |
| livox_hub_msg.launch      | Connect to Livox Hub units<br />Publish livox custom format point cloud |
| lvx_to_rosbag.launch      | Covert lvx file to rosbag file<br />Covert lvx file directly to rosbag file |
| lvx_to_rosbag_rviz.launch | Covert lvx file to rosbag file<br />Read data from lvx file and convert it to pointcloud2, then publish using ros topic |

## Configure livox_ros_driver internal parameter

The livox_ros_driver internal parameters are in the launch file, they are listed as below :

| Parameter name | detail                                                       |
| -------------- | ------------------------------------------------------------ |
| publish_freq   | Set the frequency of publishing pointcloud data <br/>The data type is float, it can be set to 10.0,20.0,50.0,100.0,200, etc. |
| multi_topic    | All lidars share the same topic, or each lidar uses a topic independently<br/>0 -- all lidars share the same topic<br/>1 -- each lidar uses a topic independently |
| xfer_format    | Set publish data format<br/>0 -- livox pointcloud2(PointXYZRTL)<br/>1 -- livox custom msg format<br/>2 -- pcl pointcloud2(pcl::PointXYZI) |

***Notes :*** 

Detailed xfer data format

1.1 The livox point format in pointcloud2 message : 

```
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float intensity         # the value is reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

1.2  Livox custom message format :

```
Header header             # ROS standard message header
uint64 timebase           # The time of first point
uint32 point_num          # Total number of pointclouds
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
```

The Custom Point format in custom message : 

```
uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar
```

1.3 PCL pointcloud2(pcl::PointXYZI) format :

​      Please refer the pcl::PointXYZI structure in Point Cloud Library (point_types.hpp) .

## Configure LiDAR parameter

There are two json files in the directory "ws_livox/src/livox_ros_driver/launch", they are livox_hub_config.json and livox_lidar_config.json.

1. When connect with LiDAR only, we can modify LiDAR parameter in livox_lidar_config.json.

   The content of the file livox_lidar_config.json is as follows : 

   ```
   {
   	"lidar_config": [
   		{
   			"broadcast_code": "0TFDG3B006H2Z11",
   			"enable_connect": true,
   			"enable_fan": true,
   			"return_mode": 0,
   			"coordinate": 0,
   			"imu_rate": 1,
   			"extrinsic_parameter_source": 0
   		}
       ]
   }
   ```

   If you want to config a new LiDAR  by livox_lidar_config.json file. For example , if you want to config a LiDAR (broadcast code : "1HDDG8M00100191") : (1) enable connection；(2) enable fan; (3) select First single return mode; (4) use Cartesian coordinate; (5) use Cartesian coordinate; (6) set imu rate to 200; (7)enable extrinsic parameter. the content  of livox_lidar_config.json  file should be : 

   ```
   {
   	"lidar_config": [
   		{
   			"broadcast_code": "0TFDG3B006H2Z11",
   			"enable_connect": true,
   			"enable_fan": true,
   			"return_mode": 0,
   			"coordinate": 0,
   			"imu_rate": 1,
   			"extrinsic_parameter_source": 1
        	},
   		{
   			"broadcast_code": "1HDDG8M00100191",
   			"enable_connect": true,
   			"enable_fan": true,
   			"return_mode": 0,
   			"coordinate": 0,
   			"imu_rate": 1,
   			"extrinsic_parameter_source": 1
   		}
   	]
   }
   ```

   

2. When connect with Hub, we must modify Hub or LiDAR parameter in livox_hub_config.json.

   The content of the  livox_hub_config.json is as follows :

   ```
   {
   	"hub_config": {
   		"broadcast_code": "13UUG1R00400170",
   		"enable_connect": true,
   		"coordinate": 0
   	},
   	"lidar_config": [
   		{
   			"broadcast_code": "0TFDG3B006H2Z11",
   			"enable_fan": true,
   			"return_mode": 0,
   			"imu_rate": 1
   		}
   	]
   }
   ```

If you want to config a new LiDAR  by the livox_hub_config.json file, For example , if you want to config a LiDAR (broadcast code : "1HDDG8M00100191") :   (1) enable connection；(2) enable fan; (3) select First single return mode; (4) use Cartesian coordinate; (5) use Cartesian coordinate; (6) set imu rate to 200.  the content  of the livox_hub_config.json file should be : 

```
{
	"hub_config": {
		"broadcast_code": "13UUG1R00400170",
		"enable_connect": true,
		"coordinate": 0
	},
	"lidar_config": [
		{
			"broadcast_code": "0TFDG3B006H2Z11",
			"enable_fan": true,
			"return_mode": 0,
			"imu_rate": 1
		},
		{
			"broadcast_code": "1HDDG8M00100191",
			"enable_fan": true,
			"return_mode": 0,
			"imu_rate": 1
		}
	]
}
```

***Notes :*** 

When connect with Hub, the lidar's parameter "enable_connect" and "coordinate" could only be set via hub.

## Convert the lvx data file（v1.0/v1.1） to rosbag file

Livox ros driver support lvx file to rosbag file function, using lvx_to_rosbag.launch file.

Excute the follow command :

 `roslaunch livox_ros_driver lvx_to_rosbag.launch lvx_file_path:="/home/livox/test.lvx"`

Replace the path "/home/livox/test.lvx" to your local path when convert the lvx data file. 





