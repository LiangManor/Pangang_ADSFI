/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: main function definition
*/

#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <iomanip>
#include "ara/log/logging.h"
#include "ara/exec/execution_client.h"
#include "socket_can_interface.h"
#include "can_send_utils.h"

using mdc::canSend::CanSendParam;
using mdc::canSend::CanSendUtils;

/*
 * 使用说明打印
 */
static void PrintUsage()
{
    std::cout << "\nUsage: can_send_sample <CAN interface> <options>\n"
                 "Options: -I <canId>\n"
                 "         -D <canData>\n"
                 "         -g <sendGap>\n"
                 "         -f\n"
                 "Examples:\n"
                 "can_send_sample can0 -I 300 -D 0000 -g 20\n"
                 "can_send_sample can0 -I 300 -D 0000 -g 20 -f\n";
}

/*
 * 将字符串转化为二进制数据
 * 无传入参数，则使用默认值，否则解析传入的参数
 */
static bool InitSendParam(CanSendParam &sendParam,
                          const std::vector<std::string> &args,
                          ara::log::Logger& printLog)
{
    if (args.size() == 1) {
        const std::uint32_t defaultCanId = 0x111;
        const std::uint32_t defaultSendGap = 1000;
        sendParam.canName = "can0";
        sendParam.canId = defaultCanId;
        sendParam.canData = {0x00, 0x00};
        sendParam.sendGap = defaultSendGap;
        sendParam.canfdFlag = false;
    } else {
        if (!CanSendUtils::CheckArgcNum(args.size())) {
            PrintUsage();
            return false;
        }
        if (!CanSendUtils::ParseArgs(args, sendParam)) {
            PrintUsage();
            return false;
        }
    }
    return true;
}

int main(int argc, char* argv[])
{
    InitLogging("CANS", "CAN_SEND_DEMO", ara::log::LogLevel::kVerbose,
                (ara::log::LogMode::kRemote));
    ara::exec::ExecutionClient execClient;
    execClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    ara::log::Logger& mainLog {ara::log::CreateLogger("can send", "can send sample context",
                                                      ara::log::LogLevel::kVerbose)};
    mainLog.LogInfo() << "Can send sample begin...";

    CanSendParam sendParam;
    if (!InitSendParam(sendParam, std::vector<std::string>(argv, argv + argc), mainLog)) {
        return -1;
    }

    mdc::canDemo::SocketCanInterface socketCan;
    if (!socketCan.Init(sendParam.canName)) {
        mainLog.LogError() << sendParam.canName << "Init failed!";
        return -1;
    }

    bool printFlag = true;;
    std::uint32_t execTimes = 100000;
    while (execTimes--) {
        std::this_thread::sleep_for(std::chrono::milliseconds(sendParam.sendGap));
        if (sendParam.canfdFlag) {
            canfd_frame frame = {};
            frame.can_id = sendParam.canId;
            frame.len = sendParam.canData.size();
            if (memcpy_s(frame.data, sizeof(frame.data), sendParam.canData.data(), sendParam.canData.size()) != 0) {
                mainLog.LogError() << "copy data failed! data size:" << sendParam.canData.size();
                break;
            }
            socketCan.WriteCanfd(frame);
        } else {
            can_frame frame = {};
            frame.can_id = sendParam.canId;
            frame.can_dlc = sendParam.canData.size();
            if (memcpy_s(frame.data, sizeof(frame.data), sendParam.canData.data(), sendParam.canData.size()) != 0) {
                mainLog.LogError() << "copy data failed! data size:" << sendParam.canData.size();
                break;
            }
            socketCan.WriteCan(frame);
        }

        if (printFlag) {
            printFlag = false;
            mainLog.LogInfo() << "Can data is send...";
        }
    }

    mainLog.LogInfo() << "Can send sample done...";
    execClient.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
    return 0;
}
