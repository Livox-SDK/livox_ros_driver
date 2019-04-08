//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include <vector>
#include <chrono>

#include "livox_sdk.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <livox_ros_driver/CustomPoint.h>
#include <livox_ros_driver/CustomMsg.h>

#define kMaxPointPerEthPacket           (100)
#define kMaxStoragePackets              (128)      // must be 2^n
#define KEthPacketHeaderLength          (18)   //(sizeof(LivoxEthPacket) - 1)
#define KEthPacketMaxLength             (1500)
#define KCartesianPointSize             (13)
#define KSphericalPointSzie             (9)

const uint64_t kPacketTimeGap      = 1000000;     // 1ms = 1000000ns
const uint64_t kMaxPacketTimeGap   = 1700000;     // the threshold of packet continuous
const uint64_t kDeviceDisconnectThreshold = 1000000000;     // the threshold of device disconect
const uint64_t kNsPerSecond        = 1000000000;  // 1s  = 1000000000ns
const uint32_t kPublishIntervalMs  = 50;         // unit:ms

#define BD_ARGC_NUM                     (4)
#define BD_ARGV_POS                     (1)
#define COMMANDLINE_BD_SIZE             (15)

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

#pragma pack(1)
typedef struct {
  uint64_t time_rcv;  // used for pps sync only mode
  uint32_t point_num;
  uint8_t  raw_data[KEthPacketMaxLength];
} StoragePacket;

typedef struct {
  uint8_t lidar_id;
  uint8_t rsvd[3];
  uint32_t point_num;
  uint64_t timestamp;  // ns
  LivoxPoint *point;
} PublishPacket;
#pragma pack()

typedef struct {
  StoragePacket *storage_packet[kMaxStoragePackets]; // size must be 2^n
  volatile uint32_t rd_idx;
  volatile uint32_t wr_idx;
  uint32_t mask;
  uint32_t size;  // must be 2^n
} StoragePacketQueue;

typedef struct {
  uint32_t receive_packet_count;
  uint32_t loss_packet_count;
  uint64_t last_timestamp;
  uint64_t timebase;                 // unit:nanosecond
  uint32_t timebase_state;
} LidarPacketStatistic;

typedef enum {
  kCoordinateCartesian = 0,
  kCoordinateSpherical
} CoordinateType;

typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg
} LivoxMsgType;

StoragePacketQueue storage_packet_pool[kMaxLidarCount];

/* for global publisher use */
ros::Publisher cloud_pub;


/* for device connect use ----------------------------------------------------------------------- */
typedef enum {
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct {
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
  LidarPacketStatistic statistic_info;
} DeviceItem;

DeviceItem lidars[kMaxLidarCount];

/* user add broadcast code here */
const char* broadcast_code_list[] = {
  "000000000000001",
};

#define BROADCAST_CODE_LIST_SIZE    (sizeof(broadcast_code_list) / sizeof(intptr_t))

/* total broadcast code, include broadcast_code_list and commandline input */
std::vector<std::string > total_broadcast_code;

/* for pointcloud queue process */
void InitQueue(StoragePacketQueue* queue) {
  queue->rd_idx = 0;
  queue->wr_idx = 0;
  queue->size = kMaxStoragePackets;
  queue->mask = kMaxStoragePackets - 1;
  for (int i=0; i<kMaxStoragePackets; i++) {
    queue->storage_packet[i] = new StoragePacket;
  }
}

void ResetQueue(StoragePacketQueue* queue) {
  queue->rd_idx = 0;
  queue->wr_idx = 0;
}

void InitStoragePacketPool(void) {
  for (int i=0; i<kMaxLidarCount; i++) {
    InitQueue(&storage_packet_pool[i]);
  }
}

static void QueueProPop(StoragePacketQueue* queue, StoragePacket* storage_packet) {
  uint32_t mask = queue->mask;
  uint32_t rd_idx = queue->rd_idx & mask;

  memcpy(storage_packet, queue->storage_packet[rd_idx], sizeof(StoragePacket));
}

static void QueuePopUpdate(StoragePacketQueue* queue) {
  queue->rd_idx++;
}

uint32_t QueuePop(StoragePacketQueue* queue, StoragePacket* storage_packet) {
  QueueProPop(queue, storage_packet);
  QueuePopUpdate(queue);

  return 1;
}

uint32_t QueueUsedSize(StoragePacketQueue *queue) {
  return (queue->wr_idx - queue->rd_idx) & queue->mask;
}

uint32_t QueueIsFull(StoragePacketQueue *queue) {
  return ((queue->wr_idx + 1) == queue->rd_idx);
}

uint32_t QueueIsEmpty(StoragePacketQueue *queue) {
  return (queue->rd_idx == queue->wr_idx);
}

static uint32_t GetEthPacketLen(LivoxEthPacket* eth_packet, uint32_t point_num) {
  if (kCoordinateCartesian == eth_packet->data_type) {
    return (KEthPacketHeaderLength + point_num * KCartesianPointSize);
  } else {
    return (KEthPacketHeaderLength + point_num * KSphericalPointSzie);
  }
}

uint32_t PushEthPacketToStorageQueue(StoragePacketQueue* queue, LivoxEthPacket* eth_packet, \
                                     uint32_t point_num, uint64_t timebase) {
  uint32_t mask = queue->mask;
  uint32_t wr_idx = queue->wr_idx & mask;

  queue->storage_packet[wr_idx]->time_rcv  = timebase;
  queue->storage_packet[wr_idx]->point_num = point_num;
  memcpy(queue->storage_packet[wr_idx]->raw_data, \
         reinterpret_cast<char *>(eth_packet), \
         GetEthPacketLen(eth_packet, point_num));

  queue->wr_idx++;

  return 1;
}

static uint64_t GetStoragePacketTimestamp(StoragePacket* packet) {

  LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(packet->raw_data);
  uint64_t timestamp = *((uint64_t *)(raw_packet->timestamp));
  if (raw_packet->timestamp_type == kTimestampTypePps) {
    return (timestamp + packet->time_rcv);
  } else if (raw_packet->timestamp_type == kTimestampTypeNoSync) {
    return timestamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePtp) {
    return timestamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year  = raw_packet->timestamp[0] + 100; // map 2000 to 1990
    time_utc.tm_mon   = raw_packet->timestamp[1];
    time_utc.tm_mday  = raw_packet->timestamp[2];
    time_utc.tm_hour  = raw_packet->timestamp[3];
    time_utc.tm_min   = 0;
    time_utc.tm_sec   = 0;

    uint64_t time_epoch = mktime(&time_utc);
    time_epoch = time_epoch * 1000000 + *((uint32_t *)(&(raw_packet->timestamp[4]))); // to us
    time_epoch = time_epoch * 1000; // to ns

    return time_epoch;
  } else {
    ROS_INFO("timestamp type invalid");
    return 0;
  }
}

static uint32_t GetPointInterval(uint32_t device_type) {
  if ((kDeviceTypeLidarTele == device_type) || \
      (kDeviceTypeLidarHorizon == device_type)) {
    return 4167; // 4167 ns
  } else {
    return 10000; // ns
  }
}

/* for pointcloud convert process */
static uint32_t PublishPointcloud2(StoragePacketQueue* queue, uint32_t packet_num, \
                                   uint8_t handle) {
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;
  /* init point cloud data struct */
  PointCloud::Ptr cloud (new PointCloud);

  cloud->header.frame_id = "livox_frame";
  cloud->height = 1;
  cloud->width  = 0;

  StoragePacket storage_packet;
  while (published_packet <  packet_num) {
    QueueProPop(queue, &storage_packet);
    LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    LivoxRawPoint* raw_points = reinterpret_cast<LivoxRawPoint *>(raw_packet->data);

    timestamp = GetStoragePacketTimestamp(&storage_packet);
    if (published_packet && \
        ((timestamp - last_timestamp) > kMaxPacketTimeGap)) {
      ROS_INFO("packet loss : %ld", timestamp);
      break;
    }

    if (!cloud->width) {
      cloud->header.stamp = timestamp / 1000; // to us
      ROS_DEBUG("[%d]:%ld us", handle, cloud->header.stamp);
    }
    cloud->width += storage_packet.point_num;

    for (uint32_t i = 0; i < storage_packet.point_num; i++) {
      pcl::PointXYZI point;
      point.x = raw_points->x/1000.0f;
      point.y = raw_points->y/1000.0f;
      point.z = raw_points->z/1000.0f;
      point.intensity = (float) raw_points->reflectivity;
      ++raw_points;
      cloud->points.push_back(point);
    }

    QueuePopUpdate(queue);
    last_timestamp = timestamp;
    ++published_packet;
  }
  //ROS_INFO("%d", cloud->width);
  cloud_pub.publish(cloud);

  return published_packet;
}

/* for pointcloud convert process */
static uint32_t PublishCustomPointcloud(StoragePacketQueue* queue, uint32_t packet_num,\
                                      uint8_t handle) {
  static uint32_t msg_seq = 0;
  uint64_t timestamp = 0;
  uint64_t last_timestamp = 0;
  uint32_t published_packet = 0;
  uint32_t point_interval = GetPointInterval(lidars[handle].info.type);
  uint32_t packet_offset_time = 0; // ns

  /* init livox custom msg */
  livox_ros_driver::CustomMsg livox_msg;

  livox_msg.header.frame_id = "livox_frame";
  livox_msg.header.seq = msg_seq;
  ++msg_seq;
  livox_msg.header.stamp = ros::Time::now();
  livox_msg.timebase = 0;
  livox_msg.point_num = 0;
  livox_msg.lidar_id  = handle;

  StoragePacket storage_packet;
  while (published_packet <  packet_num) {
    QueueProPop(queue, &storage_packet);
    LivoxEthPacket* raw_packet = reinterpret_cast<LivoxEthPacket *>(storage_packet.raw_data);
    LivoxRawPoint* raw_points = reinterpret_cast<LivoxRawPoint *>(raw_packet->data);

    timestamp = GetStoragePacketTimestamp(&storage_packet);
    if (published_packet && \
        ((timestamp - last_timestamp) > kDeviceDisconnectThreshold)) {
      ROS_INFO("packet loss : %ld", timestamp);
      break;
    }
    if (!livox_msg.timebase) {
      livox_msg.timebase = timestamp; // to us
      packet_offset_time = 0;         // first packet
      ROS_DEBUG("[%d]:%ld %d", handle, livox_msg.timebase, point_interval);
    } else {
      packet_offset_time = (uint32_t)(timestamp - livox_msg.timebase);
    }  
    livox_msg.point_num += storage_packet.point_num;

    for (uint32_t i = 0; i < storage_packet.point_num; i++) {
      livox_ros_driver::CustomPoint point;
      point.offset_time = packet_offset_time + i*point_interval;
      point.x = raw_points->x/1000.0f;
      point.y = raw_points->y/1000.0f;
      point.z = raw_points->z/1000.0f;
      point.reflectivity = raw_points->reflectivity;
      point.line = 0;
      ++raw_points;
      livox_msg.points.push_back(point);
    }

    QueuePopUpdate(queue);
    last_timestamp = timestamp;
    ++published_packet;
  }
  //ROS_INFO("%d", livox_msg.point_num);
  cloud_pub.publish(livox_msg);

  return published_packet;
}

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point) {
  p_dpoint->x = p_raw_point->x/1000.0f;
  p_dpoint->y = p_raw_point->y/1000.0f;
  p_dpoint->z = p_raw_point->z/1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num) {
  using namespace std;

  LivoxEthPacket* lidar_pack = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  LidarPacketStatistic *packet_statistic = &lidars[handle].statistic_info;
  uint64_t cur_timestamp = *((uint64_t *)(lidar_pack->timestamp));
  if (lidar_pack->timestamp_type == kTimestampTypePps) {
    if ((cur_timestamp < packet_statistic->last_timestamp) && \
        (cur_timestamp < kPacketTimeGap)) { // sync point

      auto cur_time = chrono::high_resolution_clock::now();
      int64_t sync_time = cur_time.time_since_epoch().count();

      packet_statistic->timebase = sync_time;
      //ROS_DEBUG("sysnc time : %lu %lu %lu", packet_statistic->timebase, cur_timestamp, \
      //                                 packet_statistic->last_timestamp);
    }
  }
  packet_statistic->last_timestamp = cur_timestamp;

  StoragePacketQueue *p_queue = &storage_packet_pool[handle];
  if (!QueueIsFull(p_queue)) {
    if (data_num <= kMaxPointPerEthPacket) {
      PushEthPacketToStorageQueue(p_queue, lidar_pack, data_num, packet_statistic->timebase);
    }
  }
}

void PollPointcloudData(int msg_type) {
  for (int i = 0; i < kMaxLidarCount; i++) {
      StoragePacketQueue *p_queue  = &storage_packet_pool[i];

    if (kDeviceStateSampling != lidars[i].device_state) {
      continue;
    }

    while (!QueueIsEmpty(p_queue)) {
      //ROS_DEBUG("%d %d %d %d\r\n", i, p_queue->rd_idx, p_queue->wr_idx, QueueUsedSize(p_queue));
      uint32_t used_size = QueueUsedSize(p_queue);
      if (kPointCloud2Msg == msg_type) {
        if (used_size == PublishPointcloud2(p_queue, used_size, i)) {
          break;
        }
      } else {
        if (used_size == PublishCustomPointcloud(p_queue, QueueUsedSize(p_queue), i)) {
          break;
        }
      }
    }
  }
}

/** add bd to total_broadcast_code */
void AddBroadcastCode(const char* bd_str) {
  total_broadcast_code.push_back(bd_str);
}

/** add bd in broadcast_code_list to total_broadcast_code */
void AddLocalBroadcastCode(void) {
  for (int i = 0; i < BROADCAST_CODE_LIST_SIZE; ++i) {
    std::string invalid_bd = "000000000";
    ROS_INFO("broadcast code list :%s", broadcast_code_list[i]);
    if ((COMMANDLINE_BD_SIZE == strlen(broadcast_code_list[i])) && \
        (NULL == strstr(broadcast_code_list[i], invalid_bd.c_str()))) {
      AddBroadcastCode(broadcast_code_list[i]);
    } else {
      ROS_INFO("Invalid bd:%s", broadcast_code_list[i]);
    }
  }
}

/** add commandline bd to total_broadcast_code */
void AddCommandlineBroadcastCode(const char* cammandline_str) {
  char* strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char* bd_str  = strtok(strs, pattern.c_str());
  std::string invalid_bd = "000000000";
  while (bd_str != NULL) {
    ROS_INFO("commandline input bd:%s", bd_str);
    if ((COMMANDLINE_BD_SIZE == strlen(bd_str)) && \
        (NULL == strstr(bd_str, invalid_bd.c_str()))) {
      AddBroadcastCode(bd_str);
    } else {
      ROS_INFO("Invalid bd:%s", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete [] strs;
}


/** Callback function of starting sampling. */
void OnSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
  ROS_INFO("OnSampleCallback statue %d handle %d response %d", status, handle, response);
  if (status == kStatusSuccess) {
    if (response != 0) {
      lidars[handle].device_state = kDeviceStateConnect;
    }
  } else if (status == kStatusTimeout) {
    lidars[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(uint8_t status, uint8_t handle, uint8_t response, void *data) {
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(uint8_t status, uint8_t handle, DeviceInformationResponse *ack, void *data) {
  if (status != kStatusSuccess) {
    ROS_INFO("Device Query Informations Failed %d", status);
  }
  if (ack) {
    ROS_INFO("firm ver: %d.%d.%d.%d",
             ack->firmware_version[0],
             ack->firmware_version[1],
             ack->firmware_version[2],
             ack->firmware_version[3]);
  }
}

/** Callback function of changing of device state. */
void OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == NULL) {
    return;
  }

  ROS_INFO("OnDeviceChange broadcast code %s update type %d", info->broadcast_code, type);

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, OnDeviceInformation, NULL);
    if (lidars[handle].device_state == kDeviceStateDisconnect) {
      lidars[handle].device_state = kDeviceStateConnect;
      lidars[handle].info = *info;
    }
  } else if (type == kEventDisconnect) {
    lidars[handle].device_state = kDeviceStateDisconnect;
  } else if (type == kEventStateChange) {
    lidars[handle].info = *info;
  }

  if (lidars[handle].device_state == kDeviceStateConnect) {
    ROS_INFO("Device State status_code %d", lidars[handle].info.status.status_code);
    ROS_INFO("Device State working state %d", lidars[handle].info.state);
    ROS_INFO("Device feature %d", lidars[handle].info.feature);
    if (lidars[handle].info.state == kLidarStateNormal) {
      if (lidars[handle].info.type == kDeviceTypeHub) {
        HubStartSampling(OnSampleCallback, NULL);
      } else {
        LidarStartSampling(handle, OnSampleCallback, NULL);
      }
      lidars[handle].device_state = kDeviceStateSampling;
    }
  }
}


void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL) {
    return;
  }

  ROS_INFO("Receive Broadcast Code %s, please add it to broacast_code_list if want to connect!\n",\
           info->broadcast_code);

  if (total_broadcast_code.size() > 0) {
    bool found = false;
    for (int i = 0; i < total_broadcast_code.size(); ++i) {
      if (strncmp(info->broadcast_code, total_broadcast_code[i].c_str(), kBroadcastCodeSize) == 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      ROS_INFO("Not in the broacast_code_list, please add it to if want to connect!");
      return;
    }
  } else {
    ROS_INFO("In automatic connection mode, will connect %s", info->broadcast_code);
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, GetLidarData);
    lidars[handle].handle = handle;
    lidars[handle].device_state = kDeviceStateDisconnect;
  }
}


int main(int argc, char **argv) {
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_INFO("Livox-SDK ros demo");

  InitStoragePacketPool();
  if (!Init()) {
    ROS_FATAL("Livox-SDK init fail!");
    return -1;
  }

  AddLocalBroadcastCode();
  if (argc >= BD_ARGC_NUM) {
    ROS_INFO("Commandline input %s", argv[BD_ARGV_POS]);
    AddCommandlineBroadcastCode(argv[BD_ARGV_POS]);
  }

  if (total_broadcast_code.size() > 0) {
    ROS_INFO("list all valid bd:");
    for (int i = 0; i < total_broadcast_code.size(); ++i) {
      ROS_INFO("%s", total_broadcast_code[i].c_str());
    }
  } else {
    ROS_INFO("No valid bd input, switch to automatic connection mode!");
  }

  memset(lidars, 0, sizeof(lidars));
  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  if (!Start()) {
    Uninit();
    return -1;
  }

  /* ros related */
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;

  int msg_type;
  livox_node.getParam("livox_msg_type", msg_type);
  if (kPointCloud2Msg == msg_type) {
    cloud_pub = livox_node.advertise<PointCloud>("livox/lidar", kMaxStoragePackets);
    ROS_INFO("Publish PointCloud2");
  } else {
    cloud_pub = livox_node.advertise<livox_ros_driver::CustomMsg>("livox/lidar", \
                                                       kMaxStoragePackets);
    ROS_INFO("Publish Livox Custom Msg");
  }

  ros::Time::init();
  ros::Rate r(1000.0 / kPublishIntervalMs); // 1000.0 / x = hz
  while (ros::ok()) {
    PollPointcloudData(msg_type);
    r.sleep();
  }

  Uninit();
}


