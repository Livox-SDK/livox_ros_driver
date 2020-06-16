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
#ifndef LIVOX_ROS_DRIVER_LDDC_H_
#define LIVOX_ROS_DRIVER_LDDC_H_

#include "lds.h"
#include "livox_sdk.h"

#include <ros/ros.h>
#include <rosbag/bag.h>

namespace livox_ros {

/** Lidar data distribute control */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg
} TransferType;

class Lddc {
public:
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq, std::string frame_id);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributeLidarData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetRosNode(ros::NodeHandle *node) { cur_node_ = node; }

  void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

  Lds *lds_;

private:
  uint32_t PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle);
  uint32_t PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                 uint8_t handle);
  uint32_t PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num,
                                   uint8_t handle);
  uint32_t PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                          uint8_t handle);

  ros::Publisher *GetCurrentPublisher(uint8_t handle);
  ros::Publisher *GetCurrentImuPublisher(uint8_t handle);
  void PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t handle, LidarDevice *lidar);

  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  std::string frame_id_;
  int32_t publish_interval_ms_;
  ros::Publisher *private_pub_[kMaxSourceLidar];
  ros::Publisher *global_pub_;
  ros::Publisher *private_imu_pub_[kMaxSourceLidar];
  ros::Publisher *global_imu_pub_;

  ros::NodeHandle *cur_node_;
  rosbag::Bag *bag_;
};

} // namespace livox_ros
#endif
