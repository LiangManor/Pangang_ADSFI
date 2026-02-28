/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  camera跟踪sample main文件
 */
#include <csignal>
#include "core/core.h"
#include "tl_perception.h"
using namespace Adsfi;

namespace {
sig_atomic_t g_stopFlag = 0;
void INTSigHandler(int32_t num)
{
    (void)num;
    g_stopFlag = 1;
    std::cout << "  Signal Interactive attention received "<< num << std::endl;
    return;
}
}
int main()
{
    signal(SIGINT, INTSigHandler);
    signal(SIGPIPE, SIG_IGN);
    auto tl = std::make_unique<TlPerception>("Config.yaml");
    HafStatus ret = tl->Init();
    if (ret == HAF_SUCCESS) {
        tl->Process();
    } else {
        g_stopFlag = true;
    }

    while (!g_stopFlag && !tl->IsError()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    tl->Stop();
    return 0;
}

