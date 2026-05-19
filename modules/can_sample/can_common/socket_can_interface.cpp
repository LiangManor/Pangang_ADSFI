/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
* Description: socket can interface definition
*/

#include "socket_can_interface.h"
#include "securec.h"

namespace mdc {
namespace canDemo {
bool SocketCanInterface::Init(const std::string &canDevice)
{
    m_sockCan = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 创建套接字
    if (m_sockCan < 0) {
        return false;
    }
    struct ifreq ifr = {};
    int32_t ret = strcpy_s(ifr.ifr_name, sizeof(ifr.ifr_name), canDevice.c_str());
    if (ret != EOK) {
        CloseSocketCan();
        return false;
    }
    ret = ioctl(m_sockCan, SIOCGIFINDEX, &ifr); // 指定 can 设备
    if (ret < 0) {
        CloseSocketCan();
        return false;
    }

    const int32_t canFdFlag = 1;
    ret = setsockopt(m_sockCan, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canFdFlag, static_cast<socklen_t>(sizeof(canFdFlag)));
    if (ret < 0) {
        CloseSocketCan();
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = static_cast<__kernel_sa_family_t>(AF_CAN);
    addr.can_ifindex = ifr.ifr_ifindex;
    // 将套接字与 can 设备绑定
    ret = bind(m_sockCan, reinterpret_cast<sockaddr*>(&addr), static_cast<socklen_t>(sizeof(addr)));
    if (ret < 0) {
        CloseSocketCan();
        return false;
    }

    const int32_t sendBuffLen = 32768;
    ret = setsockopt(m_sockCan, SOL_SOCKET, SO_TIMESTAMP, &sendBuffLen, static_cast<socklen_t>(sizeof(sendBuffLen)));
    if (ret < 0) {
        CloseSocketCan();
        return false;
    }

    return true;
}

bool SocketCanInterface::SetRecvTimeout(const struct timeval &tv) const
{
    const int32_t ret = setsockopt(m_sockCan, SOL_SOCKET, SO_RCVTIMEO, &tv, static_cast<socklen_t>(sizeof(tv)));
    if (ret < 0) {
        return false;
    }
    return true;
}

int32_t SocketCanInterface::ReadCan(canfd_frame &receiveFrame, struct timeval &tstamp, std::int32_t &readBytes) const
{
    iovec iov = {
        .iov_base = static_cast<void *>(&receiveFrame),
        .iov_len = sizeof(receiveFrame)
    };

    const std::uint32_t controlSize = 512U;
    char controlBuf[CMSG_SPACE(controlSize)];
    msghdr canMsg = {
        .msg_name = nullptr,
        .msg_namelen = 0U,
        .msg_iov = &iov,
        .msg_iovlen = 1U,
        .msg_control = controlBuf,
        .msg_controllen = sizeof(controlBuf),
        .msg_flags = 0,
    };

    readBytes = static_cast<int32_t>(recvmsg(m_sockCan, &canMsg, 0));
    if (readBytes < 0) {
        return -1;
    }
    struct cmsghdr *cmsg = CMSG_FIRSTHDR(&canMsg);
    if (cmsg != nullptr) {
        tstamp = *(reinterpret_cast<timeval*>(CMSG_DATA(cmsg)));
    }
    return 0;
}

int32_t SocketCanInterface::WriteCan(const can_frame &sendFrame) const
{
    const auto bytes = static_cast<int32_t>(write(m_sockCan, &sendFrame, sizeof(sendFrame))); // 发送 sendFrame 报文
    if (bytes != static_cast<int32_t>(sizeof(sendFrame))) {
        return -1;
    }
    return 0;
}

int32_t SocketCanInterface::WriteCanfd(const canfd_frame &sendFrame) const
{
    const auto bytes = static_cast<int32_t>(write(m_sockCan, &sendFrame, sizeof(sendFrame))); // 发送 sendFrame 报文
    if (bytes != static_cast<int32_t>(sizeof(sendFrame))) {
        return -1;
    }
    return 0;
}

bool SocketCanInterface::SetCanFiLter(const struct can_filter &filter) const
{
    const auto filterSize = static_cast<socklen_t>(sizeof(filter));
    const int32_t ret = setsockopt(m_sockCan, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, filterSize);
    if (ret < 0) {
        return false;
    }
    return true;
}

bool SocketCanInterface::SetCanFilters(const std::vector<can_filter> &filters) const
{
    if (filters.empty()) {
        return false;
    }
    const auto itemSize = static_cast<socklen_t>(sizeof(can_filter));
    const auto filterSize = static_cast<socklen_t>(static_cast<socklen_t>(filters.size()) * itemSize);
    const int32_t ret = setsockopt(m_sockCan, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filterSize);
    if (ret < 0) {
        return false;
    }
    return true;
}

void SocketCanInterface::CloseSocketCan()
{
    if (m_sockCan > 0) {
        (void)close(m_sockCan);
        m_sockCan = -1;
    }
}
}
}