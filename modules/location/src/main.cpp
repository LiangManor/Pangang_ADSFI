/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Description:  location 多源融合定位demo 主函数
 */
#include <iostream>
#include <csignal>
#include "location.h"
using namespace Adsfi;
namespace {
    sig_atomic_t g_stopFlag = 0;
    void INTSigHandler(int32_t num)
    {
        (void)num;
        g_stopFlag = 1;
        // std::cout << "  Signal Interactive attention received." << std::endl;
        return;
    }
}

int main()
{
    signal(SIGINT, INTSigHandler);
    auto location = std::make_unique<Location>("Config.yaml");
    HafStatus ret = location->Init();
    if (ret != HAF_SUCCESS) {
        // std::cout << "location init failed!" << std::endl;
        return -1;
    }
    const uint32_t sleepTime = 10000U;
    while (!g_stopFlag) 
    {
        location->Process();
        static_cast<void>(usleep(sleepTime));// 10ms
    }
    location->Stop();
    return 0;
}
