// file: multicast_demo.cpp
#include <iostream>
#include <string>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>

class UdpMulticastReceiver {
public:
    UdpMulticastReceiver(const std::string& maddr, uint16_t port, const std::string& ifname)
        : multicastAddress(maddr), port(port), interfaceName(ifname), sockfd(-1) {}

    ~UdpMulticastReceiver() {
        if (sockfd >= 0) {
            close(sockfd);
            sockfd = -1;
        }
    }

    bool initSock() {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            perror("socket");
            return false;
        }

        int reuse = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
            perror("setsockopt SO_REUSEADDR");
            close(sockfd);
            sockfd = -1;
            return false;
        }

        struct sockaddr_in localAddr;
        memset(&localAddr, 0, sizeof(localAddr));
        localAddr.sin_family = AF_INET;
        localAddr.sin_port = htons(port);
        localAddr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(sockfd, (struct sockaddr*)&localAddr, sizeof(localAddr)) < 0) {
            perror("bind");
            close(sockfd);
            sockfd = -1;
            return false;
        }

        struct in_addr localInterface;
        if (!getInterfaceAddress(localInterface)) {
            std::cerr << "Failed to get interface address for " << interfaceName << std::endl;
            close(sockfd);
            sockfd = -1;
            return false;
        }

        struct ip_mreqn mreq;
        memset(&mreq, 0, sizeof(mreq));
        mreq.imr_multiaddr.s_addr = inet_addr(multicastAddress.c_str());
        mreq.imr_address = localInterface;
        // try to set ifindex too
        mreq.imr_ifindex = if_nametoindex(interfaceName.c_str());

        if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
            perror("setsockopt IP_ADD_MEMBERSHIP");
            close(sockfd);
            sockfd = -1;
            return false;
        }

        return true;
    }

    // blocking receive - returns a vector<int> of bytes (size 3 expected)
    std::vector<int> receiveMessage() {
        char buffer[512];
        struct sockaddr_in srcAddr;
        socklen_t addrlen = sizeof(srcAddr);
        ssize_t r = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&srcAddr, &addrlen);
        if (r < 0) {
            perror("recvfrom");
            return {-1, -1, -1};
        }
        if (r < 3) {
            std::cerr << "Got short packet: " << r << " bytes from "
                      << inet_ntoa(srcAddr.sin_addr) << ":" << ntohs(srcAddr.sin_port) << std::endl;
            return {-1, -1, -1};
        }
        // Example: take first 3 bytes
        return { static_cast<uint8_t>(buffer[0]), static_cast<uint8_t>(buffer[1]), static_cast<uint8_t>(buffer[2]) };
    }

private:
    bool getInterfaceAddress(struct in_addr& addr) {
        struct ifaddrs* ifaddr = nullptr;
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            return false;
        }
        bool found = false;
        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) continue;
            if (ifa->ifa_addr->sa_family == AF_INET && interfaceName == ifa->ifa_name) {
                struct sockaddr_in* sa = (struct sockaddr_in*)ifa->ifa_addr;
                addr = sa->sin_addr;
                found = true;
                break;
            }
        }
        freeifaddrs(ifaddr);
        return found;
    }

public:
    std::string multicastAddress;
    uint16_t port;
    std::string interfaceName;
    int sockfd;
};

// 发送类
class UdpMulticastSender {
public:
    UdpMulticastSender(const std::string& maddr, uint16_t port, const std::string& ifname)
        : multicastAddress(maddr), port(port), interfaceName(ifname), sockfd(-1) {}

    ~UdpMulticastSender() {
        if (sockfd >= 0) close(sockfd);
    }

    bool initSock() {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            perror("sender socket");
            return false;
        }

        // 指定本地发送接口（以 IP 地址）
        struct in_addr localInterface;
        if (!getInterfaceAddress(localInterface)) {
            std::cerr << "Sender: failed to get interface address for " << interfaceName << std::endl;
            close(sockfd);
            sockfd = -1;
            return false;
        }

        if (setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_IF, &localInterface, sizeof(localInterface)) < 0) {
            perror("setsockopt IP_MULTICAST_IF");
            close(sockfd); sockfd = -1; return false;
        }

        // 可选：设置 TTL（跨子网时>1），以及是否回环
        unsigned char ttl = 1;
        setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
        unsigned char loop = 1;
        setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

        // 目标地址
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port);
        inet_pton(AF_INET, multicastAddress.c_str(), &destAddr.sin_addr);

        return true;
    }

    bool sendBytes(const uint8_t* data, size_t len) {
        ssize_t s = sendto(sockfd, data, len, 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
        return (s == (ssize_t)len);
    }

private:
    bool getInterfaceAddress(struct in_addr& addr) {
        struct ifaddrs* ifaddr = nullptr;
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            return false;
        }
        bool found = false;
        for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) continue;
            if (ifa->ifa_addr->sa_family == AF_INET && interfaceName == ifa->ifa_name) {
                struct sockaddr_in* sa = (struct sockaddr_in*)ifa->ifa_addr;
                addr = sa->sin_addr;
                found = true;
                break;
            }
        }
        freeifaddrs(ifaddr);
        return found;
    }

public:
    std::string multicastAddress;
    uint16_t port;
    std::string interfaceName;
    int sockfd;
    struct sockaddr_in destAddr;
};

// 示例主程序：创建两路 sender + receiver，并运行
int main() {
    // 路 1：组播 239.0.0.1:8848，接口 eth0
    UdpMulticastReceiver recv1("239.0.0.1", 8848, "eth0");
    UdpMulticastSender   send1("239.0.0.1", 8848, "eth0");

    // 路 2：组播 239.0.0.2:8850，接口 eth0.12 （示例）
    UdpMulticastReceiver recv2("239.0.0.2", 8850, "eth0.12");
    UdpMulticastSender   send2("239.0.0.2", 8850, "eth0.12");

    if (!recv1.initSock() || !recv2.initSock() || !send1.initSock() || !send2.initSock()) {
        std::cerr << "Failed to initialize sockets" << std::endl;
        return -1;
    }

    std::atomic<bool> running{true};

    // 接收线程 1
    std::thread t_recv1([&](){
        while (running) {
            auto v = recv1.receiveMessage();
            if (v.size() >= 3) {
                // std::cout << "[R1] got: " << v[0] << "," << v[1] << "," << v[2] << std::endl;
            }
        }
    });

    // 接收线程 2
    std::thread t_recv2([&](){
        while (running) {
            auto v = recv2.receiveMessage();
            if (v.size() >= 3) {
                // std::cout << "[R2] got: " << v[0] << "," << v[1] << "," << v[2] << std::endl;
            }
        }
    });

    // 发送线程（示例：每 1 秒发送一条消息到各自组播）
    std::thread t_send([&](){
        uint8_t msg1[3] = {1,2,3};
        uint8_t msg2[3] = {10,11,12};
        while (running) {
            send1.sendBytes(msg1, sizeof(msg1));
            send2.sendBytes(msg2, sizeof(msg2));
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // 运行 10 秒示例，然后退出
    std::this_thread::sleep_for(std::chrono::seconds(10));
    running = false;

    // 为了让 recv 退出（如果 recvfrom 被阻塞），可以关闭 socket 或 send 一个数据触发返回。
    // 简单做法：close sockets（析构会 close）
    if (t_send.joinable()) t_send.join();
    // To unblock recvfrom you could send dummy packet to the bound port or set SO_RCVTIMEO, but for demo just sleep shortly
    if (t_recv1.joinable()) t_recv1.detach();
    if (t_recv2.joinable()) t_recv2.detach();

    // std::cout << "Exiting main." << std::endl;
    return 0;
}
