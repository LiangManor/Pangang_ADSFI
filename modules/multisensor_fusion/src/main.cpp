/*
 * Description: Define multisensor_fusion main
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */
#include <csignal>
#include "multisensor_fusion.h"
#include "core/core.h"


using namespace Adsfi;
namespace {
sig_atomic_t g_stopFlag = 0;
void INTSigHandler(int32_t num)
{
    (void)num;
    g_stopFlag = 1;
    std::cout << "  Signal Interactive attention received." << std::endl;
    return;
}
}
int32_t main()
{
    signal(SIGINT, INTSigHandler);
    auto msf = std::make_unique<MultisensorFusion>("Config.yaml");
    HafStatus ret = msf->Init();
    if (ret == HAF_SUCCESS) {
        msf->Process();
    } else {
        g_stopFlag = true;
    }
    while (!g_stopFlag && !msf->IsError()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout<<"This is main.cpp msf-> Stop !"<< msf->IsError() << "::"<< g_stopFlag <<std::endl;
    msf->Stop();
    return 0;
}
