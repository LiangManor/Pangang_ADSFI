/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  lidar demo main
 */
#include <iostream>
#include <csignal>
#include "lidar_tracker.h"
#include "core/core.h"
using namespace Adsfi;

namespace {
sig_atomic_t g_stopFlag = 0;
void INTSigHandler(int32_t num)
{
    (void)num;
    g_stopFlag = 1;
    std::cout << "Signal Interactive attention received.\n";
    return;
}
}
int main()
{
    signal(SIGINT, INTSigHandler);
    signal(SIGPIPE, SIG_IGN);
    auto lot = std::make_unique<LidarObstacleTracking>("Config.yaml");
    HafStatus ret = lot->Init();
    if (ret == HAF_SUCCESS) {
        lot->Process();
    } else {
        g_stopFlag = true;
    }
    while ((!g_stopFlag) && (!lot->IsStop())) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    lot->Stop();
    return 0;
}
