/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  location 多源融合定位demo源文件
 */

#include "location.h"
#include <fstream>


using namespace std;
using namespace Adsfi;
void Location::PcakageSendLocation()
{}

void Location::CheckSensor() const
{}
void Location::CheckModule() const
{}

pcl::PointCloud<pcl::PointXYZ>::Ptr point_list(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr road_point_list(new pcl::PointCloud<pcl::PointXYZ>());

bool Location::fitQuadraticCurve(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
    Eigen::Vector3f& coeff)
{
    if (!points || points->size() < 3)
        return false;

    int n = points->size();

    Eigen::MatrixXf A(n, 3);
    Eigen::VectorXf Y(n);

    for (int i = 0; i < n; ++i) {
        float x = points->points[i].x;
        float y = points->points[i].y;

        A(i, 0) = x * x;
        A(i, 1) = x;
        A(i, 2) = 1.0f;

        Y(i) = y;
    }

    coeff = (A.transpose() * A).ldlt().solve(A.transpose() * Y);

    return true;
}

Location::CurveProjectionResult Location::projectPointToCurve(
    const Eigen::Vector3f& coeff,
    const pcl::PointXYZ& target)
{
    CurveProjectionResult result;

    float a = coeff[0];
    float b = coeff[1];
    float c = coeff[2];

    auto curveY = [&](float x) {
        return a * x * x + b * x + c;
    };

    //-----------------------------------
    // 1. 粗搜索
    //-----------------------------------

    float best_x = target.x;
    float min_dist = 1e9;

    for (float x = target.x - 5.0f;
         x <= target.x + 5.0f;
         x += 0.01f)
    {
        float y = curveY(x);

        float dist =
            (x - target.x) * (x - target.x) +
            (y - target.y) * (y - target.y);

        if (dist < min_dist) {
            min_dist = dist;
            best_x = x;
        }
    }

    //-----------------------------------
    // 2. 牛顿迭代优化
    //-----------------------------------

    float x = best_x;

    for (int i = 0; i < 10; ++i)
    {
        float y = curveY(x);

        float dy = 2 * a * x + b;
        float ddy = 2 * a;

        // 距离函数一阶导
        float f =
            2 * (x - target.x) +
            2 * (y - target.y) * dy;

        // 距离函数二阶导
        float df =
            2 +
            2 * dy * dy +
            2 * (y - target.y) * ddy;

        if (fabs(df) < 1e-6)
            break;

        x = x - f / df;
    }

    //-----------------------------------
    // 3. 计算投影点
    //-----------------------------------
    float y = curveY(x);
    result.project_point.x = x;
    result.project_point.y = y;
    result.project_point.z = 0;
    //-----------------------------------
    // 4. 切线角
    //-----------------------------------
    float slope = 2 * a * x + b;
    result.yaw = atan2(slope, 1.0f);
    //-----------------------------------
    // 5. 曲率
    //-----------------------------------
    result.curvature =
        fabs(2 * a) /
        pow(1 + slope * slope, 1.5);
    //-----------------------------------
    // 6. 曲率半径
    //-----------------------------------

    if (result.curvature > 1e-6)
        result.radius = 1.0f / result.curvature;
    else
        result.radius = 1e6;

    return result;
}

void Location::Process()
{

    auto curLidarSlam = node.GetLidarSlam(1);
    // Mock output data
    auto out = std::make_shared<HafLocation>();

    		// --- 静态文件句柄，只打开一次 ---
	static std::ofstream ofs("location_points.txt", std::ios::app);
	if (!ofs.is_open())
	{
		std::cerr << "无法打开文件 location_points.txt,用于保存定位结果!" << std::endl;
	}

    Eigen::Vector3f coeff;

    if (!curLidarSlam.empty())
    {
        timeval tv;
        gettimeofday(&tv, nullptr);
        out->header.timestamp.sec      = curLidarSlam.front()->header.timestamp.sec;
        out->header.timestamp.nsec     = curLidarSlam.front()->header.timestamp.nsec;
        out->header.seq                = curLidarSlam.front()->header.seq;// 组合导航 异常值： 【0表示都正常】 【10表示组合惯导异常，A3激光雷达正常】 【1表示组合惯导正常常，A3激光雷达异常】 【11表示都异常】
        out->pose.pose.position.x      = curLidarSlam.front()->pose.pose.position.x;// 2号机柜坐标x
        out->pose.pose.position.y      = curLidarSlam.front()->pose.pose.position.y;
        out->pose.pose.position.z      = curLidarSlam.front()->pose.pose.position.z;
        out->pose.pose.orientation.z   = curLidarSlam.front()->pose.pose.orientation.z;// 2号机柜航向角 ------------
        out->pose.pose.orientation.x   = curLidarSlam.front()->pose.pose.orientation.x;// rfid 的 ID号
        out->pose.pose.orientation.y   = curLidarSlam.front()->pose.pose.orientation.y;// rfid 的 强度值

        Point_Location.x = out->pose.pose.position.x;
        Point_Location.y = out->pose.pose.position.y;
        Point_Location.z = out->pose.pose.position.z;
        float Yaw_Location = out->pose.pose.orientation.z;
        // std::cout<<"组合导航与A3激光雷达状态    = "<<out->header.seq<<std::endl;
        // std::cout<<"rfid的ID                   = "<<out->pose.pose.orientation.x<<std::endl;
        // std::cout<<"rfid的强度值               = "<<out->pose.pose.orientation.y<<std::endl;
        std::cout<<"x坐标                      = "<<out->pose.pose.position.x<<std::endl;
        std::cout<<"y坐标                      = "<<out->pose.pose.position.y<<std::endl;
        // std::cout<<"z坐标                      = "<<out->pose.pose.position.z<<std::endl;
        // std::cout<<"yaw                        = "<<out->pose.pose.orientation.z<<std::endl;

        if (!node.IsStop())
        {
            if (node.SendLocation(out) != HAF_SUCCESS)
            {
                HAF_LOG_ERROR << "send fusion locaiton data failed!";
            }
            else
            {
                //HAF_LOG_INFO << "Location Pub OK";
            }
        }

        //<<<<<<<<<<<<<<<<<<  拟合曲线，计算投影点和线角度  >>>>>>>>>>>>>>>>>>
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_point_list;
        {
            std::lock_guard<std::mutex> lock(point_list_mutex);
            local_point_list = point_list;  // 拷贝智能指针引用
        }
        if (local_point_list->empty()){
            // HAF_LOG_WARN << "point_list is Empty, Please check Road_file.";
            return;
        }

        road_point_list->clear();
        //根据定位点，加载前方一段距离内的轨迹点 road_point_list （是从 point_list 中提取出来的）
        PointProcessing::findClosestAndFit(local_point_list, Point_Location, Yaw_Location, model, road_point_list);

        // 拟合二次曲线
        if (!fitQuadraticCurve(road_point_list, coeff))
        {
            // HAF_LOG_WARN << "Failed to fit quadratic curve.";
            return;
        }

        // 计算投影点
        Location::CurveProjectionResult proj_result = projectPointToCurve(coeff, Point_Location);
        std::cout << "Mapping  Yaw: " << out->pose.pose.orientation.z << std::endl;
        std::cout << "Pointcut Yaw: " << proj_result.yaw << std::endl;
        
        //<<<<<<<<<<<<<<<<<<  拟合曲线，计算投影点和线角度  >>>>>>>>>>>>>>>>>>

        // --- 写入坐标到文件 ---
        // std::cout<<"-----------save_file_flag : " << save_file << std::endl;
        if (ofs.is_open() && save_file)
        {
            ofs << out->pose.pose.position.x << ","
                << out->pose.pose.position.y << ","
                << out->pose.pose.position.z << ","
                << proj_result.project_point.x << ","
                << proj_result.project_point.y << ","
                << proj_result.project_point.z << ","
                << out->pose.pose.orientation.z << ","
                << proj_result.yaw << std::endl;
            ofs.flush();  // 立即写入磁盘
        }

        // if (!node.IsStop())
        // {
        //     if (node.SendLocation(out) != HAF_SUCCESS)
        //     {
        //         HAF_LOG_ERROR << "send fusion locaiton data failed!";
        //     }
        //     else
        //     {
        //         //HAF_LOG_INFO << "Location Pub OK";
        //     }
        // }
    }
    return;
}


////************************************************************************************************************* */
enum class TrajectoryType {
    UNKNOWN,
    TRAJ_8_1,
    TRAJ_9_1,
    TRAJ_8_2,
    TRAJ_9_2,
    TRAJ_8_3,
    TRAJ_9_3
};

// 定义工位点到轨迹类型的映射（不再区分中间点和终点）
static const std::unordered_multimap<int, TrajectoryType> stationToTraj = {
    // 工位点1-11对应的轨迹类型
    //{0, TrajectoryType::TRAJ_8_1}, {0, TrajectoryType::TRAJ_8_2}, {0, TrajectoryType::TRAJ_8_3}, {0, TrajectoryType::TRAJ_9_1}, {0, TrajectoryType::TRAJ_9_2}, {0, TrajectoryType::TRAJ_9_3},
    {1, TrajectoryType::TRAJ_8_1}, {1, TrajectoryType::TRAJ_9_1},
    {2, TrajectoryType::TRAJ_8_2}, {2, TrajectoryType::TRAJ_8_3}, {2, TrajectoryType::TRAJ_9_2}, {2, TrajectoryType::TRAJ_9_3},
    {3, TrajectoryType::TRAJ_8_1}, {3, TrajectoryType::TRAJ_9_1},
    {4, TrajectoryType::TRAJ_8_2}, {4, TrajectoryType::TRAJ_9_2},
    {5, TrajectoryType::TRAJ_8_3}, {5, TrajectoryType::TRAJ_9_3},
    {6, TrajectoryType::TRAJ_8_1}, {6, TrajectoryType::TRAJ_9_1},
    {7, TrajectoryType::TRAJ_8_2}, {7, TrajectoryType::TRAJ_9_2},
    {8, TrajectoryType::TRAJ_8_3}, {8, TrajectoryType::TRAJ_9_3},
    {9, TrajectoryType::TRAJ_8_1}, {9, TrajectoryType::TRAJ_9_1},
    {10, TrajectoryType::TRAJ_8_2}, {10, TrajectoryType::TRAJ_9_2},
    {11, TrajectoryType::TRAJ_8_3}, {11, TrajectoryType::TRAJ_9_3},
    
    // 工位点12-17对应的轨迹类型
    {12, TrajectoryType::TRAJ_8_1}, {12, TrajectoryType::TRAJ_8_2}, {12, TrajectoryType::TRAJ_8_3},
    {13, TrajectoryType::TRAJ_9_1}, {13, TrajectoryType::TRAJ_9_2}, {13, TrajectoryType::TRAJ_9_3},
    {14, TrajectoryType::TRAJ_9_1}, {14, TrajectoryType::TRAJ_9_2}, {14, TrajectoryType::TRAJ_9_3},
    {15, TrajectoryType::TRAJ_8_1}, {15, TrajectoryType::TRAJ_9_1},
    {16, TrajectoryType::TRAJ_8_2}, {16, TrajectoryType::TRAJ_9_2},
    {17, TrajectoryType::TRAJ_8_3}, {17, TrajectoryType::TRAJ_9_3}
};

// 获取轨迹类型（不区分中间点和终点）
TrajectoryType getTrajectoryTypeByIDs(int id1, int id2) {
    // 获取两个工位点的所有可能轨迹类型
    auto range1 = stationToTraj.equal_range(id1);
    auto range2 = stationToTraj.equal_range(id2);
    
    // 查找两个工位点共有的轨迹类型
    for (auto it1 = range1.first; it1 != range1.second; ++it1) {
        for (auto it2 = range2.first; it2 != range2.second; ++it2) {
            if (it1->second == it2->second) {
                return it1->second;
            }
        }
    }
    
    // std::cout << "No matching trajectory type for IDs: " << id1 << " and " << id2 << std::endl;
    return TrajectoryType::UNKNOWN;
}

////************************************************************************************************************* */
// 获取轨迹文件名称
std::string getTrajectoryFileNameByIDs(int history_ID, int now_ID) {
    // 获取轨迹类型
    TrajectoryType trajType = getTrajectoryTypeByIDs(history_ID, now_ID);

    // 验证ID有效性
    if (trajType == TrajectoryType::UNKNOWN) {
        HAF_LOG_ERROR << "Invalid road file IDs combination: history_ID=" << history_ID 
                     << ", now_ID=" << now_ID;
        return "";
    }

    // 轨迹类型到文件名的映射
    switch (trajType) {
        case TrajectoryType::TRAJ_8_1: return "/home/sshuser/lidar_det_cluster/8_1.bin";
        case TrajectoryType::TRAJ_8_2: return "/home/sshuser/lidar_det_cluster/8_2.bin";
        case TrajectoryType::TRAJ_8_3: return "/home/sshuser/lidar_det_cluster/8_3.bin";
        case TrajectoryType::TRAJ_9_1: return "/home/sshuser/lidar_det_cluster/9_1.bin";
        case TrajectoryType::TRAJ_9_2: return "/home/sshuser/lidar_det_cluster/9_2.bin";
        case TrajectoryType::TRAJ_9_3: return "/home/sshuser/lidar_det_cluster/9_3.bin";
        default: return "/home/sshuser/lidar_det_cluster/8_1.bin";
    }
}

///*******************************************************************************///

void Location::getDirect()
{
    // std::cout<<"getDirect thread start"<<std::endl;

    while(!node.IsStop())
    {
        // std::cout << "++++++++++++ getDirect +++++++++++++"<<std::endl;

        auto msg = receiver.receiveMessage();

        // std::cout<<"G2 size=" <<msg.size() <<std::endl;

        if (msg.size() < 3)
        {
            std::this_thread::sleep_for( std::chrono::milliseconds(5));
            return;
        }
        if(msg[0] == -1 || msg[1] == -1 || msg[2] == -1)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000)); // 5000微秒 = 5毫秒
            return;
        }
        this->model      = msg[0];
        ID_road_start    = msg[1];
        ID_road_last     = msg[2];

        std::cout << "model: " << this->model << " ID_road_start: " << ID_road_start << " ID_road_last: " << ID_road_last  << std::endl;

        if (this->model > 0 && ID_road_start > 0 && ID_road_last > 0 && (ID_road_start != history_ID_road_start || ID_road_last != history_ID_road_last))
        {

            std::string road_path = getTrajectoryFileNameByIDs(ID_road_start, ID_road_last);

            std::cout << "loadPoints--------------------" << road_path <<std::endl;
            {
                std::lock_guard<std::mutex> lock(point_list_mutex);
                point_list->clear();
                PointProcessing::loadPoints(road_path, point_list);

                std::sort(point_list->points.begin(), point_list->points.end(), [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                    return a.x < b.x;  // 按x值升序排序
                });
            }

            history_ID_road_start = ID_road_start;
            history_ID_road_last = ID_road_last;

            std::cout << "point_list  size  :" << point_list->size() <<std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));      // 10毫秒
    }
}
