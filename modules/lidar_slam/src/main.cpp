#include <iostream>
#include <csignal>
#include "lidar_slam.h"
#include <thread>
using namespace Adsfi;
namespace {
    sig_atomic_t g_stopFlag = 0;
    void INTSigHandler(int32_t num)
    {
        (void)num;
        g_stopFlag = 1;
        std::cout << "  Signal Interactive attention received." << std::endl;
        return;
    }
}

int main()
{
    // cm服务注册
    signal(SIGINT, INTSigHandler);
    // 读取配置文件
    auto lidarSlam = std::make_unique<LidarSlam>("Config.yaml");
    // 启动定位服务
    HafStatus ret = lidarSlam->Init();
    if (ret != HAF_SUCCESS) {
        std::cout << "lidarSlam init failed!" << std::endl;
        return -1;
    }
#if 1 // 设置为1表示为debug模式，用于mviz可视化定位结果
    // 连接Mviz
    bool result = mdc::visual::Connect();
    if(result)
       std::cout << "mdc::visual::Connect Yes!" << std::endl;
    else
       std::cout << "mdc::visual::Connect No!" << std::endl;
#endif 
    // 话题注册
    lidarSlam->pcPointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("alignedCloud"));   // 配准后的点云  topic
    lidarSlam->mapPub          = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("mapCloud"));       // 点云地图      topic
    lidarSlam->slamPosturePub  = mdc::visual::Publisher::Advertise<mdc::visual::PoseStamped>(ara::core::String("slamPosture"));       // 点云地图      topic
    lidarSlam->tailPosturePub  = mdc::visual::Publisher::Advertise<mdc::visual::PoseStamped>(ara::core::String("tailPosture"));       // 点云地图      topic
    lidarSlam->lidarTrajectPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("lidarTrajectory"));// LIDAR历史轨迹 topic
    lidarSlam->imuTrajectPub   = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("imuTrajectory"));  // IMU历史轨迹   topic
    lidarSlam->gnssTrajectPub  = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("gnssTrajectory")); // GNSS历史轨迹  topic
    std::cout<<"定位系统开始运行"<<std::endl;
    const uint32_t sleepTime = 1000U;

    while (!g_stopFlag) 
    {
        if(lidarSlam->workingMode == 2)// 普通定位模式：2
        {
            lidarSlam->Process();
        }
        static_cast<void>(usleep(sleepTime));// 休眠1毫秒
    }
    lidarSlam->Stop();
    return 0;
}





