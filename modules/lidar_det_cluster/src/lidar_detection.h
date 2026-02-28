/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description:  lidar_det demo
 */

#ifndef SAMPLE_LIDAR_DETECTION_H
#define SAMPLE_LIDAR_DETECTION_H

#include "core/core.h"
#include "adsf/lidar_det_base.h"
#include "udp.h"
#include <iostream>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <utility>
#include <unordered_map>
#include <algorithm>
#include "PointProcessing.h"
#include "ground_filter.h"

using namespace std;
using namespace Adsfi;


class LidarDetection {
public:
    explicit LidarDetection(std::string configFile, const bool isLidarReArrange) : 
    node(configFile, isLidarReArrange), isLidarReArrange_(isLidarReArrange) ,counter(4, 0)
    {
        this->receiver.initSock();
		//读取配置文件
		auto config = Adsfi::HafYamlNode("Config.yaml");
		config.GetValue<decltype(this->mvizVision)>(("mvizVision"), (this->mvizVision));
        config.GetValue<decltype(this->model)>(("model"), (this->model));
        config.GetValue<decltype(this->ground_roi_z)>(("ground_roi_z"), (this->ground_roi_z));
        config.GetValue<decltype(this->ID_road)>(("ID_road"), (this->ID_road));


    };
    ~LidarDetection();

    Adsfi::HafStatus Init()
    {
        return node.Init();
    };

    bool IsStop() const
    {
        return node.IsStop();
    }

    void Stop()
    {
        node.Stop();
        return;
    };
    void Process();
    std::vector<int> counter;// 元素大小为4，初始值设为0
    // 实例化sock
    UdpMulticastReceiver receiver;
    float ground_roi_z = 0.3f;
    int model = 0;
    int mvizVision = 0;
    float curX = 0.;
    float curY = 0.;
    float curZ = 0.;
public:
    Adsfi::LidarDetBase node;
    bool isLidarReArrange_{false};
    void SubLidar();
    void SubLidarReArrange();
    void SubLocation();
    void PubObjects();
    void getDirect();
    void RecvLocation();
    bool drivingDiretion(int model, int lidarID);
    void cleanPcdDirectoryThread();
    GroundFilter ground_filter_;

    const uint32_t getLidarID(int model_)
    {
        const uint32_t id_4 = 4;
        const uint32_t id_6 = 6;
    	if(model_ == 1) // 往1端行驶
    	{
    		return id_4;
    	}
    	else if(model_ == 2)// 往2端行驶
    	{
    		return id_6;
    	}
    	else // 静止
    	{
    		return 0;// 表示不进行激光雷达检测
		}
    }
private:
    std::mutex point_cloud_mutex;
    std::vector<std::thread> pool;
    pcl::PointXYZ Point_Location;
    int history_ID_road_start = 0;
    int history_ID_road_last = 0;
    int ID_road_start = 0;
    int ID_road_last = 0;
    int ID_road = 0;
    int have_obj = 0;
};
#endif









