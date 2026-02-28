/*
 * Description: Define RadarTrackArrayperception Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */

#ifndef SAMPLE_RADAR_TRACK_PERCEPTION_H
#define SAMPLE_RADAR_TRACK_PERCEPTION_H

#include <sys/time.h>
#include "core/core.h"
#include "adsf/radar_track_array_det_base.h"

class RadarTrackArrayPerception {
public:
    explicit RadarTrackArrayPerception(std::string configFile) : node(configFile) {};
    ~RadarTrackArrayPerception();
    Adsfi::HafStatus Init()
    {
        return node.Init();
    };
    void Stop()
    {
        node.Stop();
        return;
    }
    bool IsError() const
    {
        return errorStatus;
    }
    void Process();
    void SendResult();
    void PrintRadar();
    void PrintLocation();
private:
    bool errorStatus{false};
    Adsfi::RadarTrackArrayDetBase node;
    std::string frameID = "radar_track_det";
    uint32_t seq = 1U;
    std::vector<std::thread> pool;
};
#endif
