/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: can receive utils declaration
*/

#ifndef CAN_RECEIVE_UTILS
#define CAN_RECEIVE_UTILS

#include <string>
#include <sstream>
#include <linux/can.h>

namespace mdc {
namespace canReceive {
class CanReceiveUtils {
public:
    /*
    * 格式化时间为字符串
    */
    static std::string FormatTime(const timeval &tstamp);

    /*
    * 格式化canId为字符串
    */
    static void FormatCanId(std::stringstream &ss, const std::uint32_t canId);

    /*
    * 格式化can数据为字符串
    */
    static void FormatCanFrameData(std::stringstream &ss, const can_frame &canFrame);

    /*
    * 格式化canfd数据为字符串
    */
    static void FormatCanfdFrameData(std::stringstream &ss, const canfd_frame &canfdFrame);
};
}
}
#endif // CAN_RECEIVE_UTILS
