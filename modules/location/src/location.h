/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  location多源融合定位demo头文件
 */

#ifndef ADSFI_SAMPLE_FUSION_LOCATION_H
#define ADSFI_SAMPLE_FUSION_LOCATION_H

#include <ctime>
#include <sys/time.h>
#include "adsf/fusion_location_base.h"
#include "core/core.h"
#include "dnn/dnn.h"
#include "PointProcessing.h"
#include "udp.h"
#include <atomic>

class Location {
public:
    explicit Location(const std::string& configFile) : node(configFile) 
    {
        this->receiver.initSock();
    		//读取配置文件
        auto config = Adsfi::HafYamlNode("Config.yaml");
        config.GetValue<decltype(this->save_file)>(("save_file"), (this->save_file));

    };
    ~Location() {};

    Adsfi::HafStatus Init()
    {
        return node.Init(); // 初始化Location框架对象
    };
    void Stop()
    {
        node.Stop(); // 停止Location框架的收发子线程
        return;
    };

    void Process();
    void CheckSensor() const;
    void CheckModule() const;
    void PcakageSendLocation();
    void getDirect();
    void Location_main();
        // 实例化sock
    UdpMulticastReceiver receiver;

public:
    int save_file = 0;

    struct CurveProjectionResult {
        pcl::PointXYZ project_point; // 投影点
        float yaw;                   // 切线角(rad)
        float curvature;             // 曲率
        float radius;                // 曲率半径
    };
    
private:
    bool fitQuadraticCurve(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points, Eigen::Vector3f& coeff);
    CurveProjectionResult projectPointToCurve(const Eigen::Vector3f& coeff, const pcl::PointXYZ& target);
    std::vector<std::thread> pool;
    Adsfi::FusionLocationBase node; // 实例化Location框架
    int history_ID_road_start = 0;
    int history_ID_road_last = 0;
    int ID_road_start = 0;
    int ID_road_last = 0;
    int ID_road = 0;
    int model = 0;
    int model_history = 0;
    pcl::PointXYZ Point_Location;
    std::mutex point_list_mutex;
};
#endif // ADSFI_SAMPLE_FUSION_LOCATION_H
