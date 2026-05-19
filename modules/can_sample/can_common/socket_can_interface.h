/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: socket can interface declaration
*/

#ifndef SOCKET_CAN_INTERFACE_H
#define SOCKET_CAN_INTERFACE_H

#include <string>
#include <vector>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace mdc {
namespace canDemo {
class SocketCanInterface {
public:
    SocketCanInterface() = default;
    ~SocketCanInterface() = default;

    bool Init(const std::string &canDevice);
    bool SetRecvTimeout(const struct timeval &tv) const;
    bool SetCanFiLter(const struct can_filter &filter) const;
    bool SetCanFilters(const std::vector<can_filter> &filters) const;
    std::int32_t ReadCan(canfd_frame &receiveFrame, struct timeval &tstamp, std::int32_t &readBytes) const;
    std::int32_t WriteCan(const can_frame &sendFrame) const;
    std::int32_t WriteCanfd(const canfd_frame &sendFrame) const;
    void CloseSocketCan();

private:
    int32_t m_sockCan = -1;
};
}
}
#endif
