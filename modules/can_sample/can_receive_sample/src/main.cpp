/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: main function definition
*/

#include <string>
#include <sstream>
#include <iomanip>
#include "ara/log/logging.h"
#include "ara/exec/execution_client.h"
#include "socket_can_interface.h"
#include "can_receive_utils.h"

using mdc::canReceive::CanReceiveUtils;

/*
 * 打印can数详细信息
 */
static void PrintCanData(const timeval &tstamp, const std::string &canName, const can_frame &canFrame)
{
    std::stringstream ssData;
    CanReceiveUtils::FormatCanFrameData(ssData, canFrame);
    std::stringstream ssCanId;
    CanReceiveUtils::FormatCanId(ssCanId, canFrame.can_id);
    std::cout << " (" << CanReceiveUtils::FormatTime(tstamp) << ")  " << canName << "  " << std::hex <<
                 std::uppercase << ssCanId.str() << std::dec << "  [" <<
                 static_cast<std::uint32_t>(canFrame.can_dlc) << "]  " << ssData.str() << std::endl;
}

/*
 * 打印canfd数详细信息
 */
static void PrintCanfdData(const timeval &tstamp, const std::string &canName, const canfd_frame canfdFrame)
{
    const std::uint32_t lenBase = 10;
    std::stringstream ssData;
    CanReceiveUtils::FormatCanfdFrameData(ssData, canfdFrame);
    std::stringstream ssCanId;
    CanReceiveUtils::FormatCanId(ssCanId, canfdFrame.can_id);
    std::cout << " (" << CanReceiveUtils::FormatTime(tstamp) << ")  " << canName << "  " << std::hex <<
                 std::uppercase << ssCanId.str() << std::dec << "  [" << canfdFrame.len / lenBase <<
                 canfdFrame.len % lenBase << "]  " << ssData.str() << std::endl;
}

int main(int argc, char* argv[])
{
    InitLogging("CANR", "CAN_RECEIVE_DEMO", ara::log::LogLevel::kVerbose,
        (ara::log::LogMode::kRemote));
    ara::exec::ExecutionClient execClient;
    execClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);

    ara::log::Logger& mainLog {ara::log::CreateLogger("can receive", "can receive sample context",
                                                      ara::log::LogLevel::kVerbose)};
    mainLog.LogInfo() << "Can receive sample begin...";

    std::string canName = "can0";
    const std::int32_t expectArgc = 2;
    if (argc == expectArgc) {
        canName = argv[1];
    }
    mdc::canDemo::SocketCanInterface socketCan;
    if (!socketCan.Init(canName)) {
        mainLog.LogError() << canName << "Init failed!";
        return -1;
    }
    timeval tv = {2, 0};
    socketCan.SetRecvTimeout(tv);

    std::uint32_t execTimes = 100000;
    while (execTimes--) {
        canfd_frame receiveFrame;
        struct timeval tstamp;
        std::int32_t readBytes = 0;
        const auto ret = socketCan.ReadCan(receiveFrame, tstamp, readBytes);
        if (ret == -1) {
            // 读取超时
            continue;
        }
        if (readBytes == sizeof(can_frame)) {
            PrintCanData(tstamp, canName, *(reinterpret_cast<can_frame*>(&receiveFrame)));
        } else {
            PrintCanfdData(tstamp, canName, receiveFrame);
        }
    }

    mainLog.LogInfo() << "Can receive sample done...";
    execClient.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
    return 0;
}
