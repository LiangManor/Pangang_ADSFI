#include "innovusion.h"
// MDC相关头文件
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
#include <cstring>
#include <arpa/inet.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
using namespace std;
using namespace ara::com;
using namespace ara::core;
using Skeleton = ara::lidar::skeleton::LidarServiceInterfaceSkeleton;


ara::log::Logger& log_ {ara::log::CreateLogger("act", "cm dds event async server sample context",ara::log::LogLevel::kVerbose)};
mdc::visual::Publisher pcPointXYZIRPub {};
std::shared_ptr<Skeleton> m_Skeleton;

typedef struct PointXYZITR 
{
	float x;
	float y;
	float z;
	int32_t time;
	float distance;
	float pitch;
	float yaw;
	uint16_t intensity;
	uint16_t ring;
}VPoint;

uint32_t counter = 0;// 激光雷达心跳值（用于感知节点判断是否是新数据）
int errorCounter = 0;// 激光雷达异常计数器（用于判断丢失多少帧 ，一旦达到上限则上报异常）
void sendPtCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
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



int main() 
{
    LidarOption lidar_option;
    CallbackProcessor processor;
    // 读取实时的雷达数据,函数返回值为雷达句柄。后续关于雷达操作均使用该值
    int handle = inno_lidar_open_live("live",  // name of lidar instance
                                      lidar_option.lidar_ip.c_str(), 
                                      lidar_option.lidar_port, 
                                      INNO_LIDAR_PROTOCOL_PCS_TCP,
                                      lidar_option.lidar_udp_port);
    inno_lidar_set_attribute_string(handle, "force_xyz_pointcloud", "1");

    int ret = inno_lidar_set_callbacks(
                                        handle,
                                        // 消息回调函数，接收雷达运行时的调试/错误消息
                                        [](const int lidar_handle, void *ctx, const uint32_t from_remote, const enum InnoMessageLevel level,
                                        const enum InnoMessageCode code, const char *error_message) {
                                        return reinterpret_cast<CallbackProcessor *>(ctx)->process_message(level, code, error_message);
                                        },
                                        // 数据回调函数，接收点云数据
                                        [](const int lidar_handle, void *ctx, const InnoDataPacket *pkt) -> int {
                                        inno_log_verify(pkt, "pkt");
                                        return reinterpret_cast<CallbackProcessor *>(ctx)->process_data(lidar_handle, *pkt);
                                        },
                                        // 状态回调函数，接收雷达操作状态信息
                                        [](const int lidar_handle, void *ctx, const InnoStatusPacket *pkt) -> int {
                                        inno_log_verify(pkt, "pkt");
                                        return reinterpret_cast<CallbackProcessor *>(ctx)->process_status(*pkt);
                                        },
                                        // 默认获取主机时间回调
                                        NULL, &processor);

    // 记录数据回调
    while(inno_lidar_start(handle) == -1) // 开始读取数据
    {
        std::cout<<"----------- 图达通启动失败:start failed ---------------"<<std::endl;
        sleep(2);
    }
    std::cout<<"----------- 图达通启动成功:start success ---------------"<<std::endl;
	bool result = mdc::visual::Connect();
	ara::log::InitLogging("DEAS", "CM_DDS_EVENT_ASYNC_SERVER_SAMPLE", ara::log::LogLevel::kVerbose,(ara::log::LogMode::kConsole | ara::log::LogMode::kRemote));
	ara::exec::ExecutionClient execClient;
	execClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);
	ara::log::Logger& mainLog {ara::log::CreateLogger("main", "cm sample context", ara::log::LogLevel::kVerbose)};
	mainLog.LogInfo() << "Let's produce some camera data...";
	m_Skeleton = std::make_shared<Skeleton>(InstanceSpecifier("liblidar_a_cm/liblidar_a_cm/LidarCmToAppPPort"), MethodCallProcessingMode::kEvent);
	m_Skeleton->OfferService();// 函數功能:发布服务
	// 注册ros发布者
	pcPointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("innoPtClound_A4"));// 原始点云的话题
    Parameters params;
    readParameters("para.txt", params);
    xAngle = params.x_angle;
    yAngle = params.y_angle;
    zAngle = params.z_angle;
    isVision = params.isVision;
    std::cout<<"debug  ------------ xAngle = "<<xAngle<<" yAngle = "<<yAngle<<" zAngle = "<<zAngle<<" isVision = "<<isVision<<std::endl;
    // 等待数据处理完成
    while (!processor.is_done()) 
    {
        if(!processor.enable)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // processor.cloud
        sendPtCloud(processor.cloud);
    }

    ret = inno_lidar_stop(handle);//停止读取数据
    inno_log_verify(ret == 0, "stop failed %d", ret);
    ret = inno_lidar_close(handle);// 停止与雷达的连接
    inno_log_verify(ret == 0, "close failed %d", ret);
    
    m_Skeleton->StopOfferService();
    mainLog.LogInfo() << "Done.";
    execClient.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
    return 0;
}


void sendPtCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAdjust(new pcl::PointCloud<pcl::PointXYZI>);
    // Eigen::Translation3f init_translation(1, 1, 1);
    // std::cout<<"xAngle = "<<xAngle<<std::endl;
    // std::cout<<"yAngle = "<<yAngle<<std::endl;
    // std::cout<<"zAngle = "<<zAngle<<std::endl;
    // Eigen::AngleAxisf init_rotation_x(xAngle*3.1415926/180., Eigen::Vector3f::UnitX());//roll（rad）
    // Eigen::AngleAxisf init_rotation_y(yAngle*3.1415926/180., Eigen::Vector3f::UnitY());//pitch
    // Eigen::AngleAxisf init_rotation_z(zAngle*3.1415926/180., Eigen::Vector3f::UnitZ());//yaw
    // Eigen::Matrix4f RT = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
    // pcl::transformPointCloud(*cloud, *cloudAdjust, RT);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAdjust(new pcl::PointCloud<pcl::PointXYZI>);
    // 1. 定义变换（内旋顺序：X → Y → Z → 平移）
    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::AngleAxisf init_rotation_x(xAngle * M_PI/180., Eigen::Vector3f::UnitX()); // roll
    Eigen::AngleAxisf init_rotation_y(yAngle * M_PI/180., Eigen::Vector3f::UnitY()); // pitch
    Eigen::AngleAxisf init_rotation_z(zAngle * M_PI/180., Eigen::Vector3f::UnitZ()); // yaw
    Eigen::Matrix4f RT = (init_translation * init_rotation_x * init_rotation_y * init_rotation_z).matrix();
    // 2. 应用变换
    pcl::transformPointCloud(*cloud, *cloudAdjust, RT);

    std::vector<VPoint> points;
    for (int i = 0; i < cloudAdjust->points.size(); i++)
    {
        VPoint point;
        point.x = cloudAdjust->points[i].x;
        point.y = cloudAdjust->points[i].y;
        point.z = cloudAdjust->points[i].z;
        point.intensity = cloudAdjust->points[i].intensity;
        points.push_back(point);
    }
	// 填值
	auto sampleLidar = m_Skeleton->mdcEvent.Allocate();
	sampleLidar->header.frameId = "A4";  //坐标
	sampleLidar->isBigEndian = 1;
	sampleLidar->width  = points.size()/160;
	sampleLidar->height = 160;
	sampleLidar->pointStep = sizeof (VPoint);
	sampleLidar->rowStep = sampleLidar->pointStep * sampleLidar->width;
	sampleLidar->isDense = 1;
	counter++;
    sampleLidar->header.seq = counter;
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
        mdc::visual::PointCloud<mdc::visual::PointXYZI> data;
        data.header.frameId = "map";// ---------点云坐标
        const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
        uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000UL;
        data.isDense = false;
        data.header.stamp = mdc::visual::Times { sec, nsec };
        for(int i = 0; i < points.size(); i++)
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



















