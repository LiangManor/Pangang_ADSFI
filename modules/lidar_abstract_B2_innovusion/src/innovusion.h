#include <getopt.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include "inno_lidar_api.h"
#include "inno_lidar_other_api.h"
#include "inno_lidar_packet_utils.h"
#include "inno_lidar_log.h"
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



static size_t kmaxPointNumberOneFrame = 500000;
static const double kUsInSecond = 1000000.0;
static const double k10UsInSecond = 100000.0;
bool enable = false;
struct LidarOption {
  std::string lidar_ip = "192.168.1.23";
  uint16_t lidar_port = 8010;
  uint16_t lidar_udp_port = 8028;
};

struct PcdPoint {
  float x;
  float y;
  float z;
  uint16_t reflectivity;
  uint16_t facet;
  uint16_t is_2nd_return;
  uint16_t multi_return;
  uint16_t confid_level;
  double timestamp;
  uint16_t scanline;
  uint16_t scan_idx;
  uint32_t frame_id;
};



class CallbackProcessor {
 public:
  explicit CallbackProcessor();

  void set_done() { done_ = true; }
  bool is_done() { return done_; }

  void process_message(const enum InnoMessageLevel level, const enum InnoMessageCode code, const char *error_message);

  int process_data(int handler, const InnoDataPacket &pkt);

  int process_status(const InnoStatusPacket &pkt);

  int recorder_data(int lidar_handle, void *context, enum InnoRecorderCallbackType type,
                                      const char *buffer, int len);

 public:
  CallbackProcessor(const CallbackProcessor &) = delete;
  CallbackProcessor &operator=(const CallbackProcessor &) = delete;

 public:
  int64_t current_frame_ = -1;
  int64_t frame_so_far_ = -1;
  volatile bool done_ = false;
  std::vector<PcdPoint> frame_data_;
  bool enable = false;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
};

CallbackProcessor::CallbackProcessor():cloud(new pcl::PointCloud<pcl::PointXYZI>)
{
  frame_data_.reserve(kmaxPointNumberOneFrame);
}

void CallbackProcessor::process_message(const enum InnoMessageLevel level, const enum InnoMessageCode code,
                                        const char *error_message) {
}

int CallbackProcessor::process_status(const InnoStatusPacket &pkt) 
{
  return 0;
}

int CallbackProcessor::process_data(int handler, const InnoDataPacket &pkt) {
  // iterate each point in the pkt
  for (int i = 0; i < pkt.item_number; i++) 
  {
    if (pkt.type == INNO_ITEM_TYPE_XYZ_POINTCLOUD) 
    {
      if (current_frame_ != pkt.idx) 
      {
        this->enable = false;
        this->cloud->points.resize(0);
        for(int i = 0; i < frame_data_.size();i++)
        {
            pcl::PointXYZI pt;
            pt.x =  frame_data_[i].z;
            pt.y =  -frame_data_[i].y;
            pt.z =  frame_data_[i].x;
            pt.intensity =  frame_data_[i].reflectivity;
            this->cloud->points.push_back(pt);
        }
        this->enable = true;
        frame_data_.clear();
        frame_so_far_++;
        current_frame_ = pkt.idx;
      }
      InnoXyzPoint *point = reinterpret_cast<InnoXyzPoint *>(const_cast<char *>(pkt.payload));
      PcdPoint pcd_point;
      pcd_point.x = point[i].x;
      pcd_point.y = point[i].y;
      pcd_point.z = point[i].z;
      pcd_point.reflectivity = point[i].refl;
      double frame_timestamp_sec = pkt.common.ts_start_us / kUsInSecond + point[i].ts_10us / k10UsInSecond;
      pcd_point.timestamp = frame_timestamp_sec;
      pcd_point.facet = point[i].facet;
      pcd_point.is_2nd_return = point[i].is_2nd_return;
      pcd_point.multi_return = point[i].multi_return;
      frame_data_.emplace_back(pcd_point);

    } 
    else if (CHECK_EN_XYZ_POINTCLOUD_DATA(pkt.type)) 
    {
      if (current_frame_ != pkt.idx) 
      {
        this->enable = false;
        this->cloud->points.resize(0);
        for(int i = 0; i < frame_data_.size();i++)
        {
            pcl::PointXYZI pt;
            pt.x =  frame_data_[i].z;
            pt.y =  -frame_data_[i].y;
            pt.z =  frame_data_[i].x;
            pt.intensity =  frame_data_[i].reflectivity;
            this->cloud->points.push_back(pt);
        }
        this->enable = true;
        frame_data_.clear();
        frame_so_far_++;
        current_frame_ = pkt.idx;
      }
      InnoEnXyzPoint *point = reinterpret_cast<InnoEnXyzPoint *>(const_cast<char *>(pkt.payload));
      PcdPoint pcd_point;
      pcd_point.x = point[i].x;
      pcd_point.y = point[i].y;
      pcd_point.z = point[i].z;
      pcd_point.reflectivity = point[i].reflectance;
      double frame_timestamp_sec = pkt.common.ts_start_us / kUsInSecond + point[i].ts_10us / k10UsInSecond;
      pcd_point.timestamp = frame_timestamp_sec;
      pcd_point.facet = point[i].facet;
      frame_data_.emplace_back(pcd_point);
    }
  }
  // 
  return 0;
}

int CallbackProcessor::recorder_data(int lidar_handle, void *context, enum InnoRecorderCallbackType type,
                                    const char *buffer, int len) {

  return 0;
}

void usage(const char *arg0) {

  return;
}

void parse_command(int argc, char **argv, LidarOption *lidar_option) 
{
}

