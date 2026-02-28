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

class Location {
public:
    explicit Location(const std::string& configFile) : node(configFile) 
    {
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

public:
    int save_file = 0;
private:
    Adsfi::FusionLocationBase node; // 实例化Location框架
};
#endif // ADSFI_SAMPLE_FUSION_LOCATION_H
