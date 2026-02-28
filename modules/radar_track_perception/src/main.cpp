/*
 * Description: Define RadarTrackArrayperception main
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */

#include <iostream>
#include <csignal>
#include "radar_track_array_perception.h"
using namespace Adsfi;

namespace {
sig_atomic_t g_stopFlag = 0;
void INTSigHandler(int32_t num)
{
    (void)num;
    g_stopFlag = 1;
    std::cout << "  Signal Interactive attention received " << std::endl;
    return;
}
}

int32_t main()
{
    signal(SIGINT, INTSigHandler);
    auto rtp = std::make_unique<RadarTrackArrayPerception>("Config.yaml");
    HafStatus ret = rtp->Init();
    if (ret == HAF_SUCCESS) {
        rtp->Process();
    } else {
        g_stopFlag = true;
    }
    while (!g_stopFlag && !rtp->IsError()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    rtp->Stop();
    return 0;
}
