/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: can send utils definitoin
*/

#include "can_send_utils.h"

namespace mdc {
namespace canSend {
bool CanSendUtils::ParseCanData(std::vector<std::uint8_t> &canData, const std::string &strData)
{
    std::uint32_t dataSize = 0;
    while ((dataSize < strData.size()) && isxdigit(strData[dataSize])) {
        dataSize++;
    }

    const std::uint32_t evenNum = 2;
    if ((dataSize == 0) || ((dataSize % evenNum) != 0)) {
        return false;
    }

    const std::uint32_t step = 2;
    for (std::uint32_t i = 0; i < dataSize; i += step) {
        std::string strTemp = strData.substr(i, step);
        const auto value = strtoul(strTemp.c_str(), nullptr, 16);
        canData.push_back(value);
    }
    return true;
}

bool CanSendUtils::CheckArgcNum(const std::int32_t argc)
{
    bool ret = true;
    const int32_t argcMin = 8;
    const int32_t argcMax = 9;
    if ((argc < argcMin) || (argc > argcMax)) {
        ret = false;
    }
    return ret;
}

bool CanSendUtils::ParseArgs(const std::vector<std::string> &args, CanSendParam &sendParam)
{
    const std::uint32_t canNameIndex = 1;
    sendParam.canName = args[canNameIndex];
    std::uint32_t argIndex = 2;
    while (argIndex < args.size()) {
        if ((args[argIndex][0] != '-') || (args[argIndex][1] == '\0')) {
            return false;
        }
        switch (args[argIndex][1]) {
            case 'I':
                argIndex++;
                if (argIndex < args.size()) {
                    const std::uint32_t canIdBase = 16;
                    sendParam.canId = strtoul(args[argIndex].c_str(), nullptr, canIdBase);
                }
                break;

            case 'D':
                argIndex++;
                if (argIndex < args.size()) {
                    ParseCanData(sendParam.canData, args[argIndex]);
                }
                break;

            case 'g':
                argIndex++;
                if (argIndex < args.size()) {
                    const std::uint32_t gapBase = 10;
                    sendParam.sendGap = strtoul(args[argIndex].c_str(), nullptr, gapBase);
                }
                break;

            case 'f':
                sendParam.canfdFlag = true;
                break;

            default:
                return false;
                break;
        }
        argIndex++;
    }

    if ((sendParam.canId == 0) || sendParam.canData.empty() || (sendParam.sendGap == 0)) {
        return false;
    }
    return true;
}
}
}