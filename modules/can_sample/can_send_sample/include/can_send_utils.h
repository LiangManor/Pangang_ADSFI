/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: can send utils declaration
*/

#ifndef CAN_SEND_UTILS
#define CAN_SEND_UTILS

#include <vector>
#include <string>

namespace mdc {
namespace canSend {
struct CanSendParam {
    std::string canName;
    std::uint32_t canId = {0};
    std::vector<std::uint8_t> canData;
    std::uint32_t sendGap = {0};
    bool canfdFlag = {false};
};

class CanSendUtils {
public:
    /*
    * 将字符串转化为二进制数据
    */
    static bool ParseCanData(std::vector<std::uint8_t> &canData, const std::string &strData);

    /*
    * 检查参数个数的合法性
    */
    static bool CheckArgcNum(const std::int32_t argc);

    /*
    * 提取传入的参数：canId，can数据、发送间隔和canfd标志位
    */
    static bool ParseArgs(const std::vector<std::string> &args, CanSendParam &sendParam);
};
}
}
#endif // CAN_RECEIVE_UTILS
