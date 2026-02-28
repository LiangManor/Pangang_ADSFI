/* *
 * Description:  Define multisensor_fusion.h
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * */

#ifndef SAMPLE_MULTISENSOR_FUSION_H
#define SAMPLE_MULTISENSOR_FUSION_H
#include "adsf/fusion_base.h"
#include "core/core.h"
#include "udp.h"
#include "PointProcessing.h"

class MultisensorFusion {
public:
    explicit MultisensorFusion(std::string configFile) : node(configFile) {};
    ~MultisensorFusion();
    Adsfi::HafStatus Init();
    void Stop()
    {
        node.Stop();
        return;
    };
    bool IsError() const
    {
        return errorStatus;
    }
    void Process();
    void generateLidarToPixelMapping(std::string lidarID, std::vector<std::vector<pcl::PointXYZ>>& mapping, 
                                    const std::vector<float>& xValues, const std::vector<float>& yValues);
    pcl::PointXYZ findLidarCoordinates(float imgX, float imgY, const std::vector<std::vector<pcl::PointXYZ>>& mapping,
                                    const std::vector<float>& xValues, const std::vector<float>& yValues);
    void RecvLidar(const uint32_t insInx);
    void RecvCamera(const uint32_t insInx);
    void FuseData();
    void FuseLoop();
    void RecvLocation();
    pcl::PointXYZ pixel2lidar(std::string lidarID, float imgX, float imgY, float targetZ);
    bool IsMatched(const Adsfi::Haf3dDetectionOut<Adsfi::float32_t> &lidar_obj, const Adsfi::Haf3dDetectionOut<Adsfi::float32_t> &camera_obj);
    void SendResult(std::list<Adsfi::HafFusionOut<float>>& fusion_data);
    void ShowMarkArray();
    void lidar2pixel(std::string lidarID, float lidarX, float lidarY, float lidarZ, 
                    float & imgX, float & imgY);
    float centerPtsDistance(float lidarX, float lidarY, float lidarZ, float camX, float camY);
    void SendError(uint32_t erSeqID, std::string erFrameID);
    void getDirect();

private:
    bool errorStatus { false };
    // void GetObj(std::shared_ptr<Adsfi::Haf3dDetectionOutArray<Adsfi::float32_t>> &data, const std::string &source);
    Adsfi::FusionBase node;
    std::string frameID = "fusion";
    uint32_t seq = 1U;
    std::vector<std::thread> pool;

    pcl::PointXYZ Point_Location;
    float Yaw_Location = 0.0;

    std::uint64_t last_camera_ts_sec = 0;
    std::uint64_t last_camera_ts_nsec = 0;
    std::uint64_t last_lidar_ts_sec = 0;
    std::uint64_t last_lidar_ts_nsec = 0;

    // std::ofstream min_x_file;        //打开文件，将障碍物距离写入
    bool flag = false;
    size_t obj_count = 0;       // 去除短时间的漏检问题
    float last_obj_dis = 50;    // 记录上一帧障碍物距离
    
    Eigen::VectorXd coeffs;
    Eigen::VectorXd img_coeffs;
    float distance_safe = 0.8;
    uint32_t lidarError = 2;
    uint32_t cameraError = 3;
    int model = 0;
    int ID_road = 0;
    int history_ID_road_start = 0;
    int history_ID_road_last = 0;
    int ID_road_start = 0;
    int ID_road_last = 0;
    UdpMulticastReceiver receiver;
};

#endif
