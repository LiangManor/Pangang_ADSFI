/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description:  lidar_det demo main()
 */
#include <csignal>
#include "core/core.h"
#include "lidar_detection.h"


using namespace Adsfi;
namespace {
const bool IS_LIDAR_REARRANGE = false;
sig_atomic_t g_stopFlag = 0;
void INTSigHandler(int32_t num)
{
    g_stopFlag = 1;
    // std::cout << "  Signal Interactive attention" << num << "received." << std::endl;
    return;
}
}

int main()
{
    signal(SIGINT, INTSigHandler);
    auto ldt = std::make_unique<LidarDetection>("Config.yaml", IS_LIDAR_REARRANGE);
    HafStatus ret = ldt->Init();
    if (ret != HAF_SUCCESS) {
        // std::cout << "Lidar detection sample init failed!" << std::endl;
        return -1;
    }
    ldt->Process();
    while ((!g_stopFlag) && (!ldt->IsStop())) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    ldt->Stop();
    return 0;
}
