/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  小工具
 */

#ifndef CAMERA_DET_UTIL_H
#define CAMERA_DET_UTIL_H

#pragma once

#include <chrono>
#include <csignal>
namespace CameraDetection {
/**
 * @brief 这个class用于管理停止信号
 *
 */
class StopUtility {
public:
    static StopUtility& Instance()
    {
        static StopUtility s{};
        return s;
    }
    void Stop()
    {
        stopFlag = 1;
        return;
    }
    bool IsStop() const
    {
        return stopFlag != 0;
    }
    StopUtility(const StopUtility&) = delete;
    StopUtility& operator=(const StopUtility&) = delete;

private:
    StopUtility() = default;
    ~StopUtility() = default;

private:
    volatile sig_atomic_t stopFlag{0};
};

class Timer {
public:
    Timer()
    {
        Tic();
    }
    void Tic()
    {
        start = std::chrono::steady_clock::now();
        return;
    }
    /**
     * @brief 计算从上一个起点开始的时间消耗（微秒μs）
     *
     * @param reset 重置，重新开始计时
     * @return int64_t 返回时间消耗（微秒μs）
     */
    int64_t Toc(bool reset = false)
    {
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        if (reset) {
            Tic();
        }
        return duration.count();
    }

private:
    std::chrono::time_point<std::chrono::steady_clock> start;
};
}  // namespace CameraDetection

#endif
