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

#include "lds_lvx.h"

#include <functional>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <thread>

#include "lvx_file.h"

namespace livox_ros {

/** Const varible
 * --------------------------------------------------------------------------------
 */
const uint32_t kMaxPacketsNumOfFrame = 8192;

/** For device connect use
 * ---------------------------------------------------------------------- */
LdsLvx::LdsLvx(uint32_t interval_ms) : Lds(interval_ms, kSourceLvxFile) {
  start_read_lvx_ = false;
  is_initialized_ = false;
  lvx_file_ = std::make_shared<LvxFileHandle>();
  packets_of_frame_.buffer_capacity =
      kMaxPacketsNumOfFrame * sizeof(LvxFilePacket);
  packets_of_frame_.packet =
      new uint8_t[kMaxPacketsNumOfFrame * sizeof(LvxFilePacket)];
}

LdsLvx::~LdsLvx() {
  if (packets_of_frame_.packet != nullptr) {
    delete[] packets_of_frame_.packet;
  }
  if (t_read_lvx_->joinable()) {
    t_read_lvx_->join();
  }
}

void LdsLvx::PrepareExit(void) {
  lvx_file_->CloseLvxFile();
  printf("Lvx to rosbag convert complete and exit!\n");
}

int LdsLvx::InitLdsLvx(const char *lvx_path) {
  if (is_initialized_) {
    printf("Livox file data source is already inited!\n");
    return -1;
  }

  int ret = lvx_file_->Open(lvx_path, std::ios::in);
  if (ret) {
    printf("Open %s file fail[%d]!\n", lvx_path, ret);
    return ret;
  }

  if (lvx_file_->GetFileVersion() == kLvxFileV1) {
    ResetLds(kSourceRawLidar);
  } else {
    ResetLds(kSourceLvxFile);
  }

  lidar_count_ = lvx_file_->GetDeviceCount();
  if (!lidar_count_ || (lidar_count_ >= kMaxSourceLidar)) {
    lvx_file_->CloseLvxFile();
    printf("Lidar count error in %s : %d\n", lvx_path, lidar_count_);
    return -1;
  }
  printf("LvxFile[%s] have %d lidars\n", lvx_path, lidar_count_);

  for (int i = 0; i < lidar_count_; i++) {
    LvxFileDeviceInfo lvx_dev_info;
    lvx_file_->GetDeviceInfo(i, &lvx_dev_info);
    lidars_[i].handle = i;
    lidars_[i].connect_state = kConnectStateSampling;
    lidars_[i].info.handle = i;
    lidars_[i].info.type = lvx_dev_info.device_type;
    memcpy(lidars_[i].info.broadcast_code, lvx_dev_info.lidar_broadcast_code,
           sizeof(lidars_[i].info.broadcast_code));

    if (lvx_file_->GetFileVersion() == kLvxFileV1) {
      lidars_[i].data_src = kSourceRawLidar;
    } else {
      lidars_[i].data_src = kSourceLvxFile;
    }

    ExtrinsicParameter *p_extrinsic = &lidars_[i].extrinsic_parameter;
    p_extrinsic->euler[0] = lvx_dev_info.roll * PI / 180.0;
    p_extrinsic->euler[1] = lvx_dev_info.pitch * PI / 180.0;
    p_extrinsic->euler[2] = lvx_dev_info.yaw * PI / 180.0;
    p_extrinsic->trans[0] = lvx_dev_info.x;
    p_extrinsic->trans[1] = lvx_dev_info.y;
    p_extrinsic->trans[2] = lvx_dev_info.z;
    EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
    p_extrinsic->enable = lvx_dev_info.extrinsic_enable;

    uint32_t queue_size = kMaxEthPacketQueueSize * 16;
    InitQueue(&lidars_[i].data, queue_size);
    queue_size = kMaxEthPacketQueueSize;
    InitQueue(&lidars_[i].imu_data, queue_size);
  }

  t_read_lvx_ =
      std::make_shared<std::thread>(std::bind(&LdsLvx::ReadLvxFile, this));
  is_initialized_ = true;

  StartRead();

  return ret;
}

/** Global function in LdsLvx for callback */
void LdsLvx::ReadLvxFile() {
  while (!start_read_lvx_)
    ;
  printf("Start to read lvx file.\n");

  int file_state = kLvxFileOk;
  int progress = 0;
  while (start_read_lvx_) {
    file_state = lvx_file_->GetPacketsOfFrame(&packets_of_frame_);
    if (!file_state) {
      uint32_t data_size = packets_of_frame_.data_size;
      uint8_t *packet_base = packets_of_frame_.packet;
      uint32_t data_offset = 0;
      while (data_offset < data_size) {
        LivoxEthPacket *eth_packet;
        int32_t handle;
        uint8_t data_type;
        if (lvx_file_->GetFileVersion()) {
          LvxFilePacket *detail_packet =
              (LvxFilePacket *)&packet_base[data_offset];
          eth_packet = (LivoxEthPacket *)(&detail_packet->version);
          handle = detail_packet->device_index;
        } else {
          LvxFilePacketV0 *detail_packet =
              (LvxFilePacketV0 *)&packet_base[data_offset];
          eth_packet = (LivoxEthPacket *)(&detail_packet->version);
          handle = detail_packet->device_index;
        }
        data_type = eth_packet->data_type;
        data_offset +=
            (GetEthPacketLen(data_type) + 1); /* packet length + device index */
        if (data_type != kImu) {
          LidarDataQueue *p_queue = &lidars_[handle].data;
          if ((p_queue != nullptr) && (handle < lidar_count_)) {
            while (QueueIsFull(p_queue)) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            QueuePushAny(p_queue, (uint8_t *)eth_packet,
                         GetEthPacketLen(data_type), 0,
                         GetPointsPerPacket(data_type));
          }
        } else {
          LidarDataQueue *p_queue = &lidars_[handle].imu_data;
          if ((p_queue != nullptr) && (handle < lidar_count_)) {
            while (QueueIsFull(p_queue)) {
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            QueuePushAny(p_queue, (uint8_t *)eth_packet,
                         GetEthPacketLen(data_type), 0,
                         GetPointsPerPacket(data_type));
          }
        }
      }
    } else {
      if (file_state != kLvxFileAtEnd) {
        printf("Exit read the lvx file, read file state[%d]!\n", file_state);
      } else {
        printf("Read the lvx file complete!\n");
      }
      break;
    }

    if (progress != lvx_file_->GetLvxFileReadProgress()) {
      progress = lvx_file_->GetLvxFileReadProgress();
      printf("Read progress : %d \n", progress);
    }
  }

  int32_t wait_cnt = 10;
  while (!IsAllQueueEmpty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (IsAllQueueReadStop()) {
      --wait_cnt;
      if (wait_cnt <= 0) {
        break;
      }
    }
  }
  RequestExit();
}

bool LdsLvx::IsAllQueueEmpty() {
  for (int i = 0; i < lidar_count_; i++) {
    LidarDevice *p_lidar = &lidars_[i];
    if (!QueueIsEmpty(&p_lidar->data)) {
      return false;
    }
  }

  return true;
}

bool LdsLvx::IsAllQueueReadStop() {
  static uint32_t remain_size[kMaxSourceLidar];
  for (int i = 0; i < lidar_count_; i++) {
    LidarDevice *p_lidar = &lidars_[i];
    if (remain_size[i] != QueueIsEmpty(&p_lidar->data)) {
      remain_size[i] = QueueIsEmpty(&p_lidar->data);
      return false;
    }
  }

  return true;
}

} // namespace livox_ros
