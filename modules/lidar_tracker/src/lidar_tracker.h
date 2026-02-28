/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  lidar目标跟踪demo头文件
 */

#ifndef SAMPLE_LIDAR_OBSTACLE_TRACKING_H
#define SAMPLE_LIDAR_OBSTACLE_TRACKING_H

#include "core/core.h"
#include "adsf/lidar_tracker_base.h"

class LidarObstacleTracking {
public:
    explicit LidarObstacleTracking(std::string configFile) : node(configFile) {};
    ~LidarObstacleTracking();
    Adsfi::HafStatus Init()
    {
        return node.Init();
    };
    void Stop()
    {
        node.Stop();
        return;
    };
    bool IsStop()
    {
        return node.IsStop();
    };
    void Process();
    void SendResult();
    void PrintDetectionObstacles();
    void PrintPointCloud(const uint32_t instanceId);
    void PrintLocation();
private:
    Adsfi::LidarTrackerBase node;
    std::string frameID = "lidar_tracker";
    uint32_t seq = 1U;
    std::vector<std::thread> pool;
};
#endif
