#ifndef SAMPLE_LIDAR_SLAM_H
#define SAMPLE_LIDAR_SLAM_H

#include "adsf/lidar_slam_base.h"
#include "core/core.h"
#include "dnn/dnn.h"
#include <ctime>
#include <sys/time.h>
#include "yaml-cpp/yaml.h"
#include <unistd.h>
#include <thread>
#include <sstream>
// pcl库头文件
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// omp加速库
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include <fstream>//c++文件操作的头文件
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <stdexcept>

//
#include "AN.h"
#include <publisher.h> //点云Mviz可视化
#include "ekf.h"
#include "slidingwindowfilter.h"
#include "rfid.h"

// UDP通信
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace Adsfi;

// ==================== ABC重定位功能 - 新增部分开始 ====================
// 重定位状态枚举
enum class RelocState {
    IDLE,                    // 空闲状态，正常运行
    WAITING_FOR_TRIGGER,     // 等待UDP触发信号
    RELOCATING,              // 正在重定位
	LEAVING                  // 离开重定位点
};

// 重定位点结构
struct RelocPoint {
    double x;
    double y;
    std::string name;

    RelocPoint(double _x, double _y, const std::string& _name)
        : x(_x), y(_y), name(_name) {}
};
// ==================== ABC重定位功能 - 新增部分结束 ====================

struct ONLINEPTS
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
};

class LidarSlam {
public:
    // 构造函数，用于初始化参数
    explicit LidarSlam(std::string configFile) : node(configFile) 
    {
        // 初始化flag
        this->initLocation = true;
         //读取配置文件 
        auto config = Adsfi::HafYamlNode("Config.yaml");
        // 配置初始原点以及方位角
        config.GetValue<decltype(this->initX)>(("initX"), (this->initX));// 笛卡尔坐标系三轴坐标
        config.GetValue<decltype(this->initY)>(("initY"), (this->initY));
        config.GetValue<decltype(this->initZ)>(("initZ"), (this->initZ));
        config.GetValue<decltype(this->initThetaX)>(("initThetaX"), (this->initThetaX));// 笛卡尔坐标系三轴旋转角
        config.GetValue<decltype(this->initThetaY)>(("initThetaY"), (this->initThetaY));
        config.GetValue<decltype(this->initThetaZ)>(("initThetaZ"), (this->initThetaZ));
        
        // 配置体素边长
        // 用于当前帧下采样
        config.GetValue<decltype(this->voxelGridSize)>(("voxelGridSize"), (this->voxelGridSize));
        this->voxelgrid.setLeafSize(this->voxelGridSize, this->voxelGridSize, this->voxelGridSize);
        // 用于点云地图下采样
        config.GetValue<decltype(this->voxelGridSize2)>(("voxelGridSize2"), (this->voxelGridSize2));
        this->voxelgrid2.setLeafSize(this->voxelGridSize2, this->voxelGridSize2, this->voxelGridSize2);

        // 配置ndt定位器参数
        config.GetValue<decltype(this->ndtGridResolution)>(("ndtGridResolution"), (this->ndtGridResolution));// ndt体素边长
        config.GetValue<decltype(this->precision)>(("precision"), (this->precision));// 迭代精度（米）
        config.GetValue<decltype(this->step)>(("step"), (this->step));// ndt体素移动步长
        config.GetValue<decltype(this->iterations)>(("iterations"), (this->iterations));// 迭代次数
        config.GetValue<decltype(this->threadNum)>(("threadNum"), (this->threadNum));// 启用的线程数
        
        // 是否启用RTK初始位姿
        config.GetValue<decltype(this->useRTK)>(("useRTK"), (this->useRTK));//
        // 大地坐标系 到 点云地图坐标系转换
        config.GetValue<decltype(this->longitudeOfOrigin)>(("longitudeOfOrigin"), (this->longitudeOfOrigin));// 原点经度
        config.GetValue<decltype(this->latitudeOfOrigin)>(("latitudeOfOrigin"), (this->latitudeOfOrigin));// 原点纬度
        
        config.GetValue<decltype(this->xParam)>(("xParam"), (this->xParam));// 大地坐标系 到 点云地图坐标系的标定参数:x
        config.GetValue<decltype(this->yParam)>(("yParam"), (this->yParam));// 大地坐标系 到 点云地图坐标系的标定参数:y
        config.GetValue<decltype(this->zAngle)>(("zAngle"), (this->zAngle));// 大地坐标系 到 点云地图坐标系的标定参数: z轴转角
        
        // 工作模式
        config.GetValue<decltype(this->workingMode)>(("workingMode"), (this->workingMode));// 切换工作模式
        config.GetValue<decltype(this->filePath)>(("filePath"), (this->filePath));// 轨迹记录路径
        config.GetValue<decltype(this->latLonOriginPath)>(("latLonOriginPath"), (this->latLonOriginPath));// 轨迹记录路径
        // 是否可视化定位效果 
        config.GetValue<decltype(this->isVision)>(("isVision"), (this->isVision));// 表示进行可视化
        config.GetValue<decltype(this->isManualInit)>(("isManualInit"), (this->isManualInit));// 表示人为初始化

        // rfid上线点文件,文件格式如下：
        // 第一行 (站位行):0,0,0
        // 第二行 (工位点1在点云地图下面的坐标：经度-纬度-航向角 或 x-y-航向角):*,*,*
        // 第三行 (工位点2在点云地图下面的坐标):*,*,*
        config.GetValue<decltype(this->rfidPath)>(("rfidPath"), (this->rfidPath));// 获取存放 rfid 上线点的文件路径
        
        // 获取地图路径
        config.GetValue<decltype(this->mapPath)>(("mapPath"), (this->mapPath));

    };
    
    ~LidarSlam()
    {
        // ==================== ABC重定位功能 - 清理UDP开始 ====================
        closeUDP();
        // ==================== ABC重定位功能 - 清理UDP结束 ====================

        for (auto &it : pool) {
            if (it.joinable()) {
                it.join();
            }
        }
    };

    Adsfi::HafStatus Init();
    void Stop()
    {
        node.Stop(); // 停止LidarSlam框架的收发子线程
        return;
    };
    Eigen::Vector3d axis_aligned_variances(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void Process();
    bool calInitPosture(const pcl::PointCloud<pcl::PointXYZ>::Ptr& currentScanPC);
    void sendLocation();
    void mvizView(pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC, vehicleState egoState, GPSDATA gpsData);
    bool getGpsStatus(uint16_t status);
    void getOnlineX_Y_Yaw(std::string path);
    float normalizeYaw(float yaw);
    // 该函数用于更新 rfid 的数值
    void threadGetRfid();
    void importMap();//导入全局地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud_ ,float & score); //返回转化后的点云
    std::vector<std::string> split(const std::string &readData);
    
public:
    mdc::visual::Publisher pcPointXYZIRPub {}; //发布坐标变换后的点云
    mdc::visual::Publisher mapPub {};          // 发布地图
    mdc::visual::Publisher slamPosturePub {};          // 发布slamPosture
    mdc::visual::Publisher tailPosturePub {};          // 发布车尾Posture
    mdc::visual::Publisher lidarTrajectPub {};          // 发布lidar轨迹
    mdc::visual::Publisher imuTrajectPub {};          // 发布imu推理轨迹
    mdc::visual::Publisher gnssTrajectPub {};          // 发布gnss轨迹
    int sum = 0;
    // 需要上报的rfid参数
    int id = -1;
    int intensity = -1;
    int lossLocNum = 0;

// 算法参数
public:
    bool initLocation;//初始化flag标志位
    int useRTK;
    Eigen::Matrix4f lastLocation;//上一时刻位姿
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud;
    
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;//体素滤波器（当前帧）
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid2;//体素滤波器（地图）
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt_omp;//ndt 定位器(为共享指针，同上)
    float initX, initY, initZ, initThetaX, initThetaY, initThetaZ;
    float voxelGridSize, voxelGridSize2 ,ndtGridResolution, precision, step;
    int iterations, threadNum, workingMode;
    double longitudeOfOrigin, latitudeOfOrigin;
    float xParam, yParam, zAngle;
    std::string mapPath, filePath, latLonOriginPath, rfidPath;
    // 声明 ofstream 变量   
    std::ofstream file_write, xyzyawFile;
    // rfid通讯
    RFID rfid;
    // 判断是否可视化定位效果
    int isVision = 0;// 默认不可视化
    int isManualInit = 0;// 默认不手动进行初始化
    ONLINEPTS olPts;
    std::vector<std::thread> pool;
    AN an;

private:
    Adsfi::LidarSlamBase node; // 实例化LidarSlam框架
    std::shared_ptr<HafLocation> out = std::make_shared<HafLocation>();
    // ==================== ABC重定位功能 - 新增成员变量开始 ====================
    // 重定位相关变量
	std::vector<RelocPoint> reloc_points_;           // ABC三个重定位点
	RelocState reloc_state_;                         // 当前重定位状态
	std::mutex reloc_state_mutex_;                   // 状态互斥锁
	std::mutex pose_mutex_;                          // 位置互斥锁
	int current_reloc_point_index_;                  // 当前接近的重定位点索引

	// UDP通信相关
	int udp_socket_;                                 // UDP socket
	struct sockaddr_in udp_server_addr_;             // UDP服务器地址（接收端）
	struct sockaddr_in udp_client_addr_;             // UDP客户端地址（发送端）
	std::thread udp_receive_thread_;                 // UDP接收线程
	std::atomic<bool> udp_running_;                  // UDP线程运行标志
	std::atomic<int> reloc_trigger_received_;       // 重定位触发信号标志

	// 重定位参数（硬编码）
	double reloc_distance_threshold_;                // 触发重定位的距离阈值（米）
	double leave_distance_threshold_;                // 判断离开的距离阈值（米）
	int udp_server_port_;                            // UDP接收端口
	std::string udp_client_ip_;                      // UDP发送目标IP
	int udp_client_port_;                            // UDP发送目标端口
	uint8_t trainDirection = 0;

	// 重定位相关函数
	void initRelocSystem();                          // 初始化重定位系统
	void setupUDP();                                 // 设置UDP通信
	void closeUDP();                                 // 关闭UDP通信
	void udpReceiveThread();                         // UDP接收线程函数
	double calculateDistance(double x1, double y1, double x2, double y2);  // 计算距离
	int findNearestRelocPoint(double& min_distance); // 查找最近的重定位点
	// ==================== ABC重定位功能 - 新增成员变量结束 ====================
};

#endif // SAMPLE_LIDAR_SLAM_H













