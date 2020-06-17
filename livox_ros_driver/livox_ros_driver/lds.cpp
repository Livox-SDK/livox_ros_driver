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

#include "lds.h"

#include <chrono>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

namespace livox_ros {

/** Common function ------
 * ----------------------------------------------------------------------- */
bool IsFilePathValid(const char *path_str) {
  int str_len = strlen(path_str);

  if ((str_len > kPathStrMinSize) && (str_len < kPathStrMaxSize)) {
    return true;
  } else {
    return false;
  }
}

uint64_t GetStoragePacketTimestamp(StoragePacket *packet, uint8_t data_src_) {

  LivoxEthPacket *raw_packet =
      reinterpret_cast<LivoxEthPacket *>(packet->raw_data);
  LdsStamp timestamp;
  memcpy(timestamp.stamp_bytes, raw_packet->timestamp, sizeof(timestamp));

  if (raw_packet->timestamp_type == kTimestampTypePps) {
    if (data_src_ != kSourceLvxFile) {
      return (timestamp.stamp + packet->time_rcv);
    } else {
      return timestamp.stamp;
    }
  } else if (raw_packet->timestamp_type == kTimestampTypeNoSync) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePtp) {
    return timestamp.stamp;
  } else if (raw_packet->timestamp_type == kTimestampTypePpsGps) {
    struct tm time_utc;
    time_utc.tm_isdst = 0;
    time_utc.tm_year = raw_packet->timestamp[0] + 100; // map 2000 to 1990
    time_utc.tm_mon  = raw_packet->timestamp[1] - 1;   // map 1~12 to 0~11
    time_utc.tm_mday = raw_packet->timestamp[2];
    time_utc.tm_hour = raw_packet->timestamp[3];
    time_utc.tm_min = 0;
    time_utc.tm_sec = 0;

    //uint64_t time_epoch = mktime(&time_utc);
    uint64_t time_epoch = timegm(&time_utc); // no timezone
    time_epoch = time_epoch * 1000000 + timestamp.stamp_word.high; // to us
    time_epoch = time_epoch * 1000;                                // to ns

    return time_epoch;
  } else {
    printf("Timestamp type[%d] invalid.\n", raw_packet->timestamp_type);
    return 0;
  }
}

uint32_t CalculatePacketQueueSize(uint32_t interval_ms, uint32_t data_type) {
  uint32_t queue_size = (interval_ms * GetPacketNumPerSec(data_type)) / 1000;

  if (queue_size < kMinEthPacketQueueSize) {
    queue_size = kMinEthPacketQueueSize;
  } else if (queue_size > kMaxEthPacketQueueSize) {
    queue_size = kMaxEthPacketQueueSize;
  }

  return queue_size;
}

void ParseCommandlineInputBdCode(const char *cammandline_str,
                                 std::vector<std::string> &bd_code_list) {
  char *strs = new char[strlen(cammandline_str) + 1];
  strcpy(strs, cammandline_str);

  std::string pattern = "&";
  char *bd_str = strtok(strs, pattern.c_str());
  std::string invalid_bd = "000000000";
  while (bd_str != NULL) {
    printf("Commandline input bd:%s\n", bd_str);
    if ((kBdCodeSize == strlen(bd_str)) &&
        (NULL == strstr(bd_str, invalid_bd.c_str()))) {
      bd_code_list.push_back(bd_str);
    } else {
      printf("Invalid bd:%s!\n", bd_str);
    }
    bd_str = strtok(NULL, pattern.c_str());
  }

  delete[] strs;
}

void EulerAnglesToRotationMatrix(EulerAngle euler, RotationMatrix matrix) {
  double cos_roll = cos(static_cast<double>(euler[0]));
  double cos_pitch = cos(static_cast<double>(euler[1]));
  double cos_yaw = cos(static_cast<double>(euler[2]));
  double sin_roll = sin(static_cast<double>(euler[0]));
  double sin_pitch = sin(static_cast<double>(euler[1]));
  double sin_yaw = sin(static_cast<double>(euler[2]));

  matrix[0][0] = cos_pitch * cos_yaw;
  matrix[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
  matrix[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

  matrix[1][0] = cos_pitch * sin_yaw;
  matrix[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
  matrix[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

  matrix[2][0] = -sin_pitch;
  matrix[2][1] = sin_roll * cos_pitch;
  matrix[2][2] = cos_roll * cos_pitch;

  /*
    float rotate[3][3] = {
    {
      std::cos(info.pitch) * std::cos(info.yaw),
      std::sin(info.roll) * std::sin(info.pitch) * std::cos(info.yaw) -
    std::cos(info.roll) * std::sin(info.yaw), std::cos(info.roll) *
    std::sin(info.pitch) * std::cos(info.yaw) + std::sin(info.roll) *
    std::sin(info.yaw) },
    {
      std::cos(info.pitch) * std::sin(info.yaw),
      std::sin(info.roll) * std::sin(info.pitch) * std::sin(info.yaw) +
    std::cos(info.roll) * std::cos(info.yaw), std::cos(info.roll) *
    std::sin(info.pitch) * std::sin(info.yaw) - std::sin(info.roll) *
    std::cos(info.yaw) },
    {
      -std::sin(info.pitch),
      std::sin(info.roll) * std::cos(info.pitch),
      std::cos(info.roll) * std::cos(info.pitch)
    }
    };
  */
}

void PointExtrisincCompensation(PointXyz *dst_point, const PointXyz &src_point,
                                ExtrinsicParameter &extrinsic) {
  dst_point->x = src_point.x * extrinsic.rotation[0][0] +
                 src_point.y * extrinsic.rotation[0][1] +
                 src_point.z * extrinsic.rotation[0][2] + extrinsic.trans[0];
  dst_point->y = src_point.x * extrinsic.rotation[1][0] +
                 src_point.y * extrinsic.rotation[1][1] +
                 src_point.z * extrinsic.rotation[1][2] + extrinsic.trans[1];
  dst_point->z = src_point.x * extrinsic.rotation[2][0] +
                 src_point.y * extrinsic.rotation[2][1] +
                 src_point.z * extrinsic.rotation[2][2] + extrinsic.trans[2];
}

/** Livox point procees for different raw data format
 * --------------------------------------------*/
uint8_t *LivoxPointToPxyzrtl(uint8_t *point_buf, LivoxEthPacket *eth_packet,
                             ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxPoint *raw_point = reinterpret_cast<LivoxPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxRawPointToPxyzrtl(uint8_t *point_buf, LivoxEthPacket *eth_packet,
                                ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxRawPoint *raw_point =
      reinterpret_cast<LivoxRawPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxSpherPointToPxyzrtl(uint8_t *point_buf,
                                  LivoxEthPacket *eth_packet,
                                  ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxSpherPoint *raw_point =
      reinterpret_cast<LivoxSpherPoint *>(eth_packet->data);

  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = 0;
    dst_point->line = 0;
    ++raw_point;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxExtendRawPointToPxyzrtl(uint8_t *point_buf,
                                      LivoxEthPacket *eth_packet,
                                      ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint *raw_point =
      reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxRawPoint *)raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    dst_point->line = line_id;
    dst_point->line = dst_point->line % 6;
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxExtendSpherPointToPxyzrtl(uint8_t *point_buf,
                                        LivoxEthPacket *eth_packet,
                                        ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendSpherPoint *raw_point =
      reinterpret_cast<LivoxExtendSpherPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxSpherPoint *)raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    dst_point->line = line_id;
    dst_point->line = dst_point->line % 6;
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxDualExtendRawPointToPxyzrtl(uint8_t *point_buf,
                                          LivoxEthPacket *eth_packet,
                                          ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxExtendRawPoint *raw_point =
      reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point, (LivoxRawPoint *)raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag;
    dst_point->line =
        line_id / 2; /* LivoxDualExtendRawPoint = 2*LivoxExtendRawPoint */
    dst_point->line = dst_point->line % 6;
    ++raw_point;
    ++dst_point;
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxDualExtendSpherPointToPxyzrtl(uint8_t *point_buf,
                                            LivoxEthPacket *eth_packet,
                                            ExtrinsicParameter &extrinsic) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = GetPointsPerPacket(eth_packet->data_type);
  LivoxDualExtendSpherPoint *raw_point =
      reinterpret_cast<LivoxDualExtendSpherPoint *>(eth_packet->data);

  uint8_t line_id = 0;
  while (points_per_packet) {
    RawPointConvert((LivoxPointXyzr *)dst_point,
                    (LivoxPointXyzr *)(dst_point + 1),
                    (LivoxDualExtendSpherPoint *)raw_point);
    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag1;
    dst_point->line = line_id;
    dst_point->line = dst_point->line % 6;
    ++dst_point;

    if (extrinsic.enable) {
      PointXyz src_point = *((PointXyz *)dst_point);
      PointExtrisincCompensation((PointXyz *)dst_point, src_point, extrinsic);
    }
    dst_point->tag = raw_point->tag2;
    dst_point->line = line_id;
    dst_point->line = dst_point->line % 6;
    ++dst_point;

    ++raw_point; /* only increase one */
    ++line_id;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

uint8_t *LivoxImuDataProcess(uint8_t *point_buf, LivoxEthPacket *eth_packet) {
  memcpy(point_buf, eth_packet->data, sizeof(LivoxImuPoint));
  return point_buf;
}

const PointConvertHandler to_pxyzi_handler_table[kMaxPointDataType] = {
    LivoxRawPointToPxyzrtl,
    LivoxSpherPointToPxyzrtl,
    LivoxExtendRawPointToPxyzrtl,
    LivoxExtendSpherPointToPxyzrtl,
    LivoxDualExtendRawPointToPxyzrtl,
    LivoxDualExtendSpherPointToPxyzrtl,
    nullptr};

PointConvertHandler GetConvertHandler(uint8_t data_type) {
  if (data_type < kMaxPointDataType)
    return to_pxyzi_handler_table[data_type];
  else
    return nullptr;
}

uint8_t *FillZeroPointXyzrtl(uint8_t *point_buf, uint32_t num) {
  LivoxPointXyzrtl *dst_point = (LivoxPointXyzrtl *)point_buf;
  uint32_t points_per_packet = num;

  while (points_per_packet) {
    dst_point->x = 0;
    dst_point->y = 0;
    dst_point->z = 0;
    dst_point->reflectivity = 0;
    dst_point->tag = 0;
    dst_point->line = 0;
    ++dst_point;
    --points_per_packet;
  }

  return (uint8_t *)dst_point;
}

#if 0

static void PointCloudConvert(LivoxPoint *p_dpoint, LivoxRawPoint *p_raw_point) {
  p_dpoint->x = p_raw_point->x/1000.0f;
  p_dpoint->y = p_raw_point->y/1000.0f;
  p_dpoint->z = p_raw_point->z/1000.0f;
  p_dpoint->reflectivity = p_raw_point->reflectivity;
}

#endif

/* Member function ------
 * ----------------------------------------------------------------------- */

Lds::Lds(uint32_t buffer_time_ms, uint8_t data_src)
    : buffer_time_ms_(buffer_time_ms), data_src_(data_src) {
  lidar_count_ = kMaxSourceLidar;
  request_exit_ = false;
  ResetLds(data_src_);
};

Lds::~Lds() {
  lidar_count_ = 0;
  ResetLds(0);
};

void Lds::ResetLidar(LidarDevice *lidar, uint8_t data_src) {
  DeInitQueue(&lidar->data);
  DeInitQueue(&lidar->imu_data);

  memset(lidar, 0, sizeof(LidarDevice));

  /** unallocated state */
  lidar->handle = kMaxSourceLidar;
  lidar->data_src = data_src;
  lidar->data_is_pubulished = false;
  lidar->connect_state = kConnectStateOff;
  lidar->raw_data_type = 0xFF;
}

void Lds::SetLidarDataSrc(LidarDevice *lidar, uint8_t data_src) {
  lidar->data_src = data_src;
}

void Lds::ResetLds(uint8_t data_src) {
  lidar_count_ = kMaxSourceLidar;
  for (uint32_t i = 0; i < kMaxSourceLidar; i++) {
    ResetLidar(&lidars_[i], data_src);
  }
}

uint8_t Lds::GetDeviceType(uint8_t handle) {
  if (handle < kMaxSourceLidar) {
    return lidars_[handle].info.type;
  } else {
    return kDeviceTypeHub;
  }
}

void Lds::PrepareExit(void) {}

} // namespace livox_ros
