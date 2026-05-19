#include "lidar_huawei_sock.h"
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <ara/lidar/lidarserviceinterface_skeleton.h>
#include <ara/lidar/lidarserviceinterface_common.h>
#include <ara/core/instance_specifier.h>
#include <ara/log/logging.h>
#include <ara/exec/execution_client.h>
#include <publisher.h>
#include <chrono>
#include <fstream>
#include <cstring>
#include <arpa/inet.h>
#include "ara/tsync/synch_master_tb.h"

using DataClock = ara::tsync::SynchMasterTB<ara::tsync::SynchMasterIdentity::k0>;
using ManageClock = ara::tsync::SynchMasterTB<ara::tsync::SynchMasterIdentity::k1>;
using namespace std;
#define DEBUG


using namespace ara::com;
using namespace ara::core;
using Skeleton = ara::lidar::skeleton::LidarServiceInterfaceSkeleton;


ara::log::Logger& log_ {ara::log::CreateLogger("act", "cm dds event async server sample context",ara::log::LogLevel::kVerbose)};
mdc::visual::Publisher pcPointXYZIRPub {};
std::shared_ptr<Skeleton> m_Skeleton;
uint32_t counter = 0;
float xAngle = 0.;
float yAngle = 0.;
float zAngle = 0.;
int isVision = 0;
struct Parameters {
    double x_angle = 0.;
    double y_angle = 0.;
    double z_angle = 0.;
    int isVision   = 0;
};
// 从文件中读取参数并赋值
bool readParameters(const string& filename, Parameters& params) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return false;
    }
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string key;
        string value;
        
        // 忽略空行
        if (line.empty()) continue;
        
        // 按 '=' 分割参数名和参数值
        if (getline(ss, key, '=') && getline(ss, value)) {
            // 去掉两边的空格
            key = key.substr(key.find_first_not_of(" \t"));
            value = value.substr(value.find_first_not_of(" \t"));

            // 解析并赋值
            if (key == "x_angle") {
                params.x_angle = stod(value);
            } else if (key == "y_angle") {
                params.y_angle = stod(value);
            } else if (key == "z_angle") {
                params.z_angle = stod(value);
            } else if (key == "isVision") {
                params.isVision = stod(value);
            }
        }
    }
    file.close();
    return true;
}


typedef struct PointXYZITR {
	float x;
	float y;
	float z;
	uint64_t time;
	float distance;
	float pitch;
	float yaw;
	uint16_t intensity;
	uint16_t ring;
}VPoint;


void LidarParsing()
{
	std::vector<VPoint> points;
	for (int i = 0; i < 96; i++)
	{
		for (int j = 0; j < (int) lidar_angle[i].size(); j++)
		{
			if (lidar_dist[i][j] < 0.2 || lidar_dist[i][j] > 200)
			{
				continue;
			}
			float angle_degree = PI/180.f;
			float rotation_azimuth = angle_degree * lidar_angle[i][j];
			if(cos_scan_altitude_caliration[i] == 0.0) return;
			double x = lidar_dist[i][j] * cos_scan_altitude_caliration[i]  * cosf(rotation_azimuth);
			double y = lidar_dist[i][j] * cos_scan_altitude_caliration[i]  * sinf(rotation_azimuth);
			double z = lidar_dist[i][j] * sin_scan_altitude_caliration[i];
			double time = lidar_mtimestamp[i][j];
			int intensity = lidar_inst[i][j];
			VPoint point;
			point.x = (float)x;
			point.y = (float)y;
			point.z = (float)z;
			point.ring = i;
			point.intensity = intensity;
			point.time = (uint64_t)(time * 1e9);      // 转换为纳秒
			points.push_back(point);
		}
	}

    uint32_t lidar_sec  = points[0].time / 1000000000ULL;
    uint32_t lidar_nsec = points[0].time % 1000000000ULL;

	// 填值
	auto sampleLidar = m_Skeleton->mdcEvent.Allocate();
	sampleLidar->header.frameId = "A3";  //坐标
	sampleLidar->isBigEndian = 1;
	sampleLidar->width  = points.size()/96;
	sampleLidar->height = 96;
	sampleLidar->pointStep = sizeof (VPoint);
	sampleLidar->rowStep = sampleLidar->pointStep * sampleLidar->width;
	sampleLidar->isDense = 1;
	counter++;
    sampleLidar->header.seq = counter;

    // ⭐⭐⭐（雷达时间戳）
    sampleLidar->header.stamp.sec  = lidar_sec;
    sampleLidar->header.stamp.nsec = lidar_nsec;

    // printf("Lidar    : %u.%09u\n", lidar_sec, lidar_nsec);

    // // ⭐⭐⭐（数据面时间戳）
    // auto tp_dp = DataClock::now();

    // auto ns_dp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //             tp_dp.time_since_epoch()).count();

    // uint32_t sec_dp  = ns_dp / 1000000000ULL;
    // uint32_t nsec_dp = ns_dp % 1000000000ULL;

    // printf("DATA_CLK : %u.%09u\n", sec_dp, nsec_dp);

    // // ⭐⭐⭐（管理面时间戳）
    // auto tp_mp = ManageClock::now();

    // auto ns_mp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //             tp_mp.time_since_epoch()).count();

    // uint32_t sec_mp  = ns_mp / 1000000000ULL;
    // uint32_t nsec_mp = ns_mp % 1000000000ULL;

    // printf("MP_CLK  : %u.%09u\n", sec_mp, nsec_mp);

    // // ⭐⭐⭐（计算数据面和雷达时间差）
    // int64_t lidar_ns =
    // (int64_t)lidar_sec * 1000000000LL + lidar_nsec;

    // int64_t sys_ns =
    //     (int64_t)sec_dp * 1000000000LL + nsec_dp;

    // int64_t diff_us = (lidar_ns - sys_ns) / 1000LL;

    // printf("DIFF     : %ld us\n", diff_us);

    // // /////

    if(counter == 10)
    {
        counter = 0;
    }
	int ptNum = sampleLidar->height * sampleLidar->width;
	size_t data_size = sizeof (VPoint) * ptNum;
	sampleLidar->data.resize(data_size);
	if (data_size)
	{
		memcpy(&sampleLidar->data[0], &points[0], data_size);
	}
	m_Skeleton->mdcEvent.Send(std::move(sampleLidar));
    if(isVision)
    {
	    if(points.size() == 0) return;
        // 向Mviz发送话题
        mdc::visual::PointCloud<mdc::visual::PointXYZI> data;
        data.header.frameId = "map";// ---------点云坐标
        const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
        data.isDense = false;
        data.header.stamp = mdc::visual::Times { lidar_sec, lidar_nsec };
        for(int i = 0; i <= points.size(); i++)
        {
            mdc::visual::PointXYZI ptMviz;
            ptMviz.x = points[i].x;
            ptMviz.y = points[i].y;
            ptMviz.z = points[i].z;
            ptMviz.intensity = points[i].intensity;
            data.points.push_back(ptMviz);
        }
        //发布点云数据
        bool isPublishpt = pcPointXYZIRPub.Publish(data);
    }
    //清空点云
	points.clear();
}



int errorCounter = 0;

int main() 
{
	bool result = mdc::visual::Connect();
	// if(result)
	// std::cout << " ----------------- mdc::visual::Connect Yes!" << std::endl;
	// else
	// std::cout << " ----------------- mdc::visual::Connect No!" << std::endl;
	ara::log::InitLogging("DEAS", "CM_DDS_EVENT_ASYNC_SERVER_SAMPLE", ara::log::LogLevel::kVerbose,(ara::log::LogMode::kConsole | ara::log::LogMode::kRemote));
	ara::exec::ExecutionClient execClient;
	execClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);
	ara::log::Logger& mainLog {ara::log::CreateLogger("main", "cm sample context", ara::log::LogLevel::kVerbose)};
	mainLog.LogInfo() << "Let's produce some camera data...";

	/*
	*InstanceSpecifier("liblidar_a_cm/liblidar_a_cm/LidarCmToAppPPort"),描述了AUTOSAR实例说明符的类，基本上是AUTOSAR简称路径包装器
	*InstanceSpecifier（參數1，參數2）
	*參數1：创建的服务实例ID
	*參數2：服务端处理method请求的模式，默认kEvent模式
	**/
	m_Skeleton = std::make_shared<Skeleton>(InstanceSpecifier("liblidar_a_cm/liblidar_a_cm/LidarCmToAppPPort"), MethodCallProcessingMode::kEvent);
	// std::cout << " ------------------------- m_Skeleton!" << std::endl;
	m_Skeleton->OfferService();// 函數功能:发布服务
	// std::cout << " ------------------------- OfferService!" << std::endl;

	InitSocket();    //udp初始
	// std::cout << " ------------------------- InitSocket!" << std::endl;
	// 注册ros发布者
	pcPointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("hwPtClound_A3"));// 原始点云的话题
    Parameters params;
    readParameters("para.txt", params);
    xAngle   = params.x_angle;
    yAngle   = params.y_angle;
    zAngle   = params.z_angle;
    isVision = params.isVision;
    // std::cout<<"程序开始运行"<<std::endl;
   while (true)
   {
      if (lidar_angle.size())
      {
         LidarParsing();
         lidar_angle.clear();
         errorCounter = 0 ;
         usleep(80000);//80毫秒
      }
      else if(errorCounter == 200)// 驱动启动之后，200ms收不到数据，则上报故障
      {
        std::vector<VPoint> points;
        // 填值
        auto sampleLidar = m_Skeleton->mdcEvent.Allocate();
        sampleLidar->header.frameId = "A3";  //坐标
        sampleLidar->isBigEndian = 1;
        sampleLidar->width  = 0/96;
        sampleLidar->height = 96;
        sampleLidar->pointStep = sizeof (VPoint);
        sampleLidar->rowStep = sampleLidar->pointStep * sampleLidar->width;
        sampleLidar->isDense = 1;
        int ptNum = sampleLidar->height * sampleLidar->width;
        size_t data_size = sizeof (VPoint) * ptNum;
        sampleLidar->data.resize(data_size);
        if (data_size)
        {
	        memcpy(&sampleLidar->data[0], &points[0], data_size);
        }
        m_Skeleton->mdcEvent.Send(std::move(sampleLidar));
        usleep(50000);//五十毫秒
      }
      else
      {
         usleep(50000);//五十毫秒
         errorCounter++;
      }
   }
   m_Skeleton->StopOfferService();
   mainLog.LogInfo() << "Done.";
   execClient.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
   return 0;
}
   
   
   
   
   
   
