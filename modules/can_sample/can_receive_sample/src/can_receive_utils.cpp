/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: can receive utils definitoin
*/

#include "can_receive_utils.h"
#include <iomanip>

namespace mdc {
namespace canReceive {
std::string CanReceiveUtils::FormatTime(const timeval &tstamp)
{
    tm tmTime = {};
    localtime_r(&tstamp.tv_sec, &tmTime);
    std::stringstream ss;
    const std::uint32_t yearData = 1900U;
    const std::uint32_t wide = 2U;
    const std::uint32_t wideUsec = 6U;
    ss << tmTime.tm_year + yearData << "-" <<
        std::setfill('0') << std::setw(wide) << (tmTime.tm_mon + 1) << "-" <<
        std::setfill('0') << std::setw(wide) << tmTime.tm_mday << " " <<
        std::setfill('0') << std::setw(wide) << tmTime.tm_hour << ":" <<
        std::setfill('0') << std::setw(wide) << tmTime.tm_min << ":" <<
        std::setfill('0') << std::setw(wide) << tmTime.tm_sec << "." <<
        std::setfill('0') << std::setw(wideUsec) << tstamp.tv_usec;
    return ss.str();
}

void CanReceiveUtils::FormatCanId(std::stringstream &ss, const std::uint32_t canId)
{
    const std::uint32_t wide = 3U;
    ss << std::uppercase << std::setfill('0') << std::setw(wide) << std::hex << canId;
}

void CanReceiveUtils::FormatCanFrameData(std::stringstream &ss, const can_frame &canFrame)
{
    const std::uint32_t byteWidth = 2;
    for (std::uint8_t i = 0; i < canFrame.can_dlc; ++i) {
        ss << std::uppercase << std::setfill('0') << std::setw(byteWidth) << std::hex <<
              static_cast<std::uint32_t>(canFrame.data[i]) << " ";
    }
}

void CanReceiveUtils::FormatCanfdFrameData(std::stringstream &ss, const canfd_frame &canfdFrame)
{
    const std::uint32_t byteWidth = 2;
    for (std::uint8_t i = 0; i < canfdFrame.len; ++i) {
        ss << std::uppercase << std::setfill('0') << std::setw(byteWidth) << std::hex <<
              static_cast<std::uint32_t>(canfdFrame.data[i]) << " ";
    }
}
}
}