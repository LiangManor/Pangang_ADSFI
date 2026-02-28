/*
 * Description:  Define multisensor_fusion.cpp
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */

#include "multisensor_fusion.h"
#include <sys/time.h>
#include "core/core.h"
#include "core/logger.h"
#include "object/object.h"
#include <mutex>
#include "hungarian_optimizer.h"
#include "secure_matrix.h"
#include <yaml-cpp/yaml.h> // 使用 yaml-cpp 库

// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/dnn.hpp>
// #include <opencv2/imgcodecs.hpp>

#include <cstring>
#include <publisher.h>

using namespace std;
using namespace Adsfi;

std::mutex data_mutex;  // 定义一个互斥锁来保护数据
std::shared_ptr<Haf3dDetectionOutArray<float32_t>> lidar_data;
std::shared_ptr<Haf3dDetectionOutArray<float32_t>> camera_data;
mdc::visual::Publisher pcPointXYZIRPub {};  //轨迹点
mdc::visual::Publisher objPublisher {};    //聚类结果发布

struct lidarPT
{
    float vel;
    float imgx;
    float imgy;
} lidarPt;

pcl::PointCloud<pcl::PointXYZ>::Ptr point_list(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr road_point_list(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr img_point_list(new pcl::PointCloud<pcl::PointXYZ>());

MultisensorFusion::~MultisensorFusion()
{
    for (auto &it : pool) {
        if (it.joinable()) {
            it.join();
        }
    }
}

/*<><><><><><><><><><><><><><><><><><><><><><><><><>*/
// struct CalibrationData
// {
//     cv::Mat RT;           // 4x4
//     cv::Mat intrinsic;    // 3x3
//     cv::Mat distortion;   // 1x8
// };

CalibrationData A4_calib =
    PointProcessing::loadCalibrationFromYAML("para.yaml", "A4");        //lidar to camera

CalibrationData B2_calib =
    PointProcessing::loadCalibrationFromYAML("para.yaml", "B2");

/*<><><><><><><><><><><><><><><><><><><><><><><><><>*/

Adsfi::HafStatus MultisensorFusion::Init()
{
    if (node.Init() != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Init detection frameword failed.";
        return HAF_ERROR;
    }

    try {
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile("para.yaml");

        // 获取参数
        model = config["model"].as<int>();
        distance_safe = config["distance_safe"].as<float>();
        ID_road = config["ID_road"].as<int>();

        // 打印参数
        std::cout << "model: " << model << std::endl;
        std::cout << "distance_safe: " << distance_safe << std::endl;
        std::cout << "ID_road: " << ID_road << std::endl;
    } catch (const YAML::Exception &e) {
        std::cerr << "Error reading YAML file: " << e.what() << std::endl;
    }

    PointProcessing::loadTransformFromYaml("para.yaml");                //lidar to lidar
    // 写入文件
    // min_x_file.open("min_x_log.txt", std::ios::app);
    // if (!min_x_file.is_open()) {
    //     std::cerr << "Failed to open min_x_log.txt" << std::endl;
    // }

    receiver.initSock();        //UDP通信，初始化
    pcPointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("fussion_path_points"));    // 点云可视化 前方轨迹
    objPublisher    = mdc::visual::Publisher::Advertise<mdc::visual::MarkerArray>(ara::core::String("fussion_lidar_MarkerArray"));      // 点云可视化 聚类框
    mdc::visual::Connect();
    // 生成点云到图像的映射关系
    // generateLidarToPixelMapping("A4", mapping_A4, xValues, yValues);
    // generateLidarToPixelMapping("B2", mapping_B2, xValues, yValues);

    return HAF_SUCCESS;
}

/*
主程序入口，开启多线程***
*/
void MultisensorFusion::Process()
{
    pool.push_back(std::thread(&MultisensorFusion::getDirect, this));               // 获取车量前进方向、加载轨迹点
    pool.push_back(std::thread(&MultisensorFusion::RecvLocation, this));            // 获取定位
    std::vector<uint32_t> lidarInsInxVec = node.GetLidarInsIdx();
    for (auto insInx : lidarInsInxVec) {
        pool.push_back(std::thread(&MultisensorFusion::RecvLidar, this, insInx));   // 获取雷达检测结果
    }
    std::vector<uint32_t> cameraInsInxVec = node.GetCameraInsIdx();
    for (auto insInx : cameraInsInxVec) {
        pool.push_back(std::thread(&MultisensorFusion::RecvCamera, this, insInx));  // 获取相机检测结果
    }
    pool.push_back(std::thread(&MultisensorFusion::FuseLoop, this));  // 添加数据融合的线程
    // pool.push_back(std::thread(&MultisensorFusion::SendResult, this));
    return;
}


// 打印时间戳并计算与当前时间的差值
void printAndCompareTimestamp(std::shared_ptr<Haf3dDetectionOutArray<float32_t>>& data) {
    // 获取当前系统时间（C++11 chrono）
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto now_nsec = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    // 转换为秒和纳秒
    uint64_t current_sec = now_sec.time_since_epoch().count();
    uint64_t current_nsec = now_nsec.time_since_epoch().count() % 1000000000;

    // 打印 data 的时间戳
    std::cout << "Data Timestamp: " << data->timestamp.sec << " sec, " 
              << data->timestamp.nsec << " nsec" << std::endl;

    // 打印当前系统时间戳
    std::cout << "Current System Timestamp: " << current_sec << " sec, " 
              << current_nsec << " nsec" << std::endl;

    // 计算差值（秒和纳秒）
    int64_t diff_sec = static_cast<int64_t>(current_sec) - static_cast<int64_t>(data->timestamp.sec);
    int64_t diff_nsec = static_cast<int64_t>(current_nsec) - static_cast<int64_t>(data->timestamp.nsec);

    // 处理纳秒借位（如果 current_nsec < data_timestamp.nsec）
    if (diff_nsec < 0) {
        diff_nsec += 1000000000;
        diff_sec -= 1;
    }

    // 打印时间差
    std::cout << "Time Difference: " << diff_sec << " sec, " 
              << diff_nsec << " nsec" << std::endl;

    // 可选：转换为浮点秒（更直观）
    double total_diff_sec = diff_sec + diff_nsec / 1e9;
    std::cout << "Time Difference (seconds): " << std::fixed << std::setprecision(9) 
              << total_diff_sec << " sec" << std::endl;
}


// // 保存点云到图像的映射关系
// void MultisensorFusion::generateLidarToPixelMapping(std::string lidarID, std::vector<std::vector<pcl::PointXYZ>>& mapping, 
//                                                     const std::vector<float>& xValues, const std::vector<float>& yValues) {
//     // 计算每个点的像素坐标并保存映射关系
//     for (float lidarY : yValues) {
//         std::vector<pcl::PointXYZ> rowMapping;
//         for (float lidarX : xValues) {
//             float imgX, imgY;
//             lidar2pixel(lidarID, lidarX, lidarY, 0, imgX, imgY);  // z = 0 平面
//             rowMapping.push_back(pcl::PointXYZ(imgX, imgY, 0.0f));
//         }
//         mapping.push_back(rowMapping);
//     }
//     std::cout << "mapping finish size  : "  << mapping.size() <<std::endl;
// }

void MultisensorFusion::RecvLocation()
{
    while (!node.IsStop()) {
        // std::cout<<"*********** RecvLocation ***********"<<std::endl;
        std::shared_ptr<HafLocation> data;
        if (node.GetLocation(data) != HAF_SUCCESS) {
            errorStatus = true;
            continue;
        }
        Point_Location.x = data->pose.pose.position.x;
        Point_Location.y = data->pose.pose.position.y;
        Yaw_Location = data->pose.pose.orientation.z;

        if (point_list->empty()){
            // HAF_LOG_ERROR << "point_list is Empty, Please check Road_file.";
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex);  // 确保线程安全
            road_point_list->clear();
            PointProcessing::findClosestAndFit(point_list, Point_Location, Yaw_Location, this->model, road_point_list);
        }

        // FuseData();         //获取定位信息，然后计算前方轨迹点，然后才进行感知结果的判断和融合
        // std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

/*
获取 激光雷达 的检测结果
*/
void MultisensorFusion::RecvLidar(const uint32_t insInx)
{
    while (!node.IsStop()) {
        // std::cout<< "++++++++++ RecvLidar +++++++++++ " << std::endl;
        std::shared_ptr<Haf3dDetectionOutArray<float32_t>> data;
        if (node.GetLidar(data, insInx) != HAF_SUCCESS) {
            errorStatus = true;
            HAF_LOG_ERROR << "GetLidar Data defilt ! ";
            lidar_data.reset();                // 防止假数据残留
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        if (data == nullptr) {
            HAF_LOG_ERROR << "lidar Data ptr is null";
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            lidar_data = data;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return;
}

/*
获取 相机 的检测结果
*/
void MultisensorFusion::RecvCamera(const uint32_t insInx)
{
    while (!node.IsStop()) {
        // std::cout<< "----------- RecvCamera ----------- " << std::endl;
        std::shared_ptr<Haf3dDetectionOutArray<float32_t>> data;
        if (node.GetCamera(data, insInx) != HAF_SUCCESS) {
            errorStatus = true;
            HAF_LOG_ERROR << "GetCamera Data defilt ! ";
            camera_data.reset();                // 防止假数据残留
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        if (data == nullptr) {
            HAF_LOG_ERROR << "Camera Data ptr is null";
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            camera_data = data;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return;
}

void MultisensorFusion::FuseLoop()
{
    while (!node.IsStop()) {
        FuseData();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void MultisensorFusion::FuseData()
{
    // 快照当前数据指针与路点，避免在处理过程中被其他线程修改
    std::shared_ptr<Haf3dDetectionOutArray<float32_t>> local_lidar;
    std::shared_ptr<Haf3dDetectionOutArray<float32_t>> local_camera;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_road_points(new pcl::PointCloud<pcl::PointXYZ>());

    {
        std::lock_guard<std::mutex> lock(data_mutex);
        local_lidar = lidar_data;
        local_camera = camera_data;
        // shallow copy road_point_list content to local copy to avoid concurrent mutation
        if (road_point_list && !road_point_list->empty()) {
            *local_road_points = *road_point_list; // 深拷贝点云数据
        }
    }

    if (local_lidar == nullptr && local_camera == nullptr) {
        HAF_LOG_INFO << "Waiting for both lidar and camera data.";
        std::this_thread::sleep_for(std::chrono::milliseconds(30));  // 稍微等待数据
        return;
    }
    HAF_LOG_INFO << "Fusing lidar and camera data...";

    // ==================【没有生成前方轨迹之前，直接透传激光雷达感知结果】==================
    bool road_ready = (local_road_points && !local_road_points->empty());

    if (!road_ready && local_lidar != nullptr && !local_lidar->detectionOut3d.empty()) {
        HAF_LOG_WARN << "Road points not ready, passthrough lidar results.";

        std::list<HafFusionOut<float>> fusion_data = {};

        for (auto &det : local_lidar->detectionOut3d) {
            HafFusionOut<float> one_data;
            one_data.cls = 0;                          // 默认危险目标
            one_data.confidence = det.confidence;

            one_data.rect.center.z = 999.0f;          // 无法计算轨迹距离，给出0
            one_data.rect.size.x  = 20;
            one_data.rect.size.y  = 20;

            one_data.absRect.center.x = det.rect.center.x;      // lidar x
            one_data.absRect.center.y = det.rect.center.y;
            one_data.absRect.center.z = det.rect.center.z;

            one_data.absRect.size.x = det.rect.size.x;
            one_data.absRect.size.y = det.rect.size.y;
            one_data.absRect.size.z = det.rect.size.z;

            fusion_data.emplace_back(one_data);
        }

        SendResult(fusion_data);
        return;
    }
    // =================================================

    std::list<HafFusionOut<float>> fusion_data = {};
    int count = 0;
    float distance = 0;
    uint cls_safe = 0;

    if (local_camera != nullptr && !local_camera->detectionOut3d.empty()) {
        HAF_LOG_INFO << "Have camera data.";
        list<Haf3dDetectionOut<float32_t>>::iterator iter;
        for (iter = local_camera->detectionOut3d.begin(); iter != local_camera->detectionOut3d.end(); iter++)
        {   
            if (iter->cls > 3) continue;
            pcl::PointXYZ img_lidar;
            if(this->model == 1){
                img_lidar = pixel2lidar("A4", iter->rect.center.x, iter->rect.center.y + (iter->rect.size.y/2), -0.6);       // 默认检测框最低，在激光雷达坐标系的 z = -0.6 高度
            }else{
                img_lidar = pixel2lidar("B2", iter->rect.center.x, iter->rect.center.y + (iter->rect.size.y/2), -0.6);
            }

            if (!std::isfinite(img_lidar.x) || !std::isfinite(img_lidar.y)) {
                // 跳过不可用的投影点，避免产生 NaN/定值打印
                continue;
            }

            // 考虑图像投影到世界坐标系时的误差，为了防止误检测，距离超出轨迹终点即舍弃。
            float last_point_x = local_road_points->back().x;
            if(img_lidar.x > (last_point_x - 1)) continue;

            distance = PointProcessing::pointToRoadDistance(img_lidar, local_road_points);
            cls_safe = (distance < distance_safe) ? 0 : 1;

            std::cout << "camera_object_distances_y   : " << distance << std::endl;
            std::cout << "camera_object_distances_x   : " << img_lidar.x << std::endl;
            
            // 通过迭代器访问元素
            HafFusionOut<float> one_data;
            one_data.cls = uint(cls_safe);                     // 图像检测种类
            one_data.confidence = iter->confidence;            // 种类置信度
            one_data.rect.center.x = iter->rect.center.x;      // 图像检测物体中心坐标 x
            one_data.rect.center.y = iter->rect.center.y;      // 中心坐标 y
            one_data.rect.center.z = distance;                 // 障碍物距离车道线距离
            one_data.rect.size.x = iter->rect.size.x;          // 图像宽
            one_data.rect.size.y = iter->rect.size.y;          // 高
            one_data.absRect.center.x = img_lidar.x;        // lidar x
            one_data.absRect.center.y = img_lidar.y;        // lidar y
            one_data.absRect.center.z = 0.8;                  // lidar z
            one_data.absRect.size.x = 0.2;        // lidar chang
            one_data.absRect.size.y = 0.2;        // lidar kuan
            one_data.absRect.size.z = 2;        // lidar gao

            fusion_data.emplace_back(one_data);
            count += 1;

        }
    }

    if (local_lidar != nullptr && !local_lidar->detectionOut3d.empty()) {
        HAF_LOG_INFO << "Have lidar data.";
        list<Haf3dDetectionOut<float32_t>>::iterator iter1;
        for (iter1 = local_lidar->detectionOut3d.begin(); iter1 != local_lidar->detectionOut3d.end(); iter1++)    // 遍历列
        {
             // 判断激光雷达检测到的障碍物是否在危险区域
            distance = PointProcessing::pointToRoadDistance(pcl::PointXYZ(iter1->rect.center.x, iter1->rect.center.y, 0.0f), local_road_points);
            cls_safe = (distance < distance_safe) ? 0 : 1;

            std::cout << "lidar_object_distance_y   : " << distance << std::endl;
            std::cout << "lidar_object_distance_x   : " << iter1->rect.center.x << std::endl;

            // 通过迭代器访问元素
            HafFusionOut<float> one_data;
            one_data.cls = uint(cls_safe);              // 激光雷达检测目标是否在危险区域
            one_data.confidence = iter1->confidence;    // 种类置信度
            one_data.rect.center.z = distance;          // 障碍物距离车道线距离
            one_data.rect.size.x = 20;                  // 目标宽
            one_data.rect.size.y = 20;                  // 目标高
            one_data.absRect.center.x = iter1->rect.center.x;        // lidar x
            one_data.absRect.center.y = iter1->rect.center.y;        // lidar y
            one_data.absRect.center.z = iter1->rect.center.z;        // lidar z
            one_data.absRect.size.x = iter1->rect.size.x;        // lidar chang
            one_data.absRect.size.y = iter1->rect.size.y;        // lidar kuan
            one_data.absRect.size.z = iter1->rect.size.z;        // lidar gao

            fusion_data.emplace_back(one_data);
            count += 1;
        }
    }
    std::cout<< "障碍物总数量 :   " << count <<std::endl;
    SendResult(fusion_data);
    return;
}

// ============================
// 使用 8 个切向畸变系数计算映射关系
// ============================
pcl::PointXYZ MultisensorFusion::pixel2lidar(std::string lidarID, float imgX, float imgY, float targetZ) 
{
    // ----------------------------
    // 1. 相机标定参数选择
    // ----------------------------
    cv::Mat localeRT = B2_calib.RT;
    cv::Mat K = B2_calib.intrinsic;
    cv::Mat dist = B2_calib.distortion;

    if(lidarID == "A4"){
        localeRT = A4_calib.RT;
        K = A4_calib.intrinsic;
        dist = A4_calib.distortion;
    }

    // ----------------------------
    // 2. 去畸变：像素坐标 → 归一化相机坐标
    // ----------------------------
    std::vector<cv::Point2f> src, undistorted;
    src.emplace_back(imgX, imgY);

    cv::undistortPoints(src, undistorted, K, dist);

    float xn = undistorted[0].x;
    float yn = undistorted[0].y;

    // ----------------------------
    // 3. 构造相机系射线方向
    // 相机坐标系射线： (xn, yn, 1)
    // ----------------------------
    cv::Mat rayCam = (cv::Mat_<float>(3,1) << xn, yn, 1.0f);

    // ----------------------------
    // 4. 取得外参：相机姿态（世界→相机）
    // 此处计算其逆：相机→世界
    // ----------------------------
    cv::Mat R = localeRT(cv::Rect(0, 0, 3, 3));
    cv::Mat t = localeRT(cv::Rect(3, 0, 1, 3));

    cv::Mat R_inv = R.t();
    cv::Mat t_inv = -R_inv * t;

    // ----------------------------
    // 5. 求射线在世界坐标中表达
    // X_world = R^-1 * (ray * s) + t_inv
    // 求与 Z = targetZ 的交点
    // ----------------------------

    // 相机中心在世界坐标：
    cv::Mat Cw = t_inv;

    // 世界坐标下射线方向
    cv::Mat rayWorld = R_inv * rayCam;   

    float rwx = rayWorld.at<float>(0);
    float rwy = rayWorld.at<float>(1);
    float rwz = rayWorld.at<float>(2);

    float Cx = Cw.at<float>(0);
    float Cy = Cw.at<float>(1);
    float Cz = Cw.at<float>(2);

    // ----------------------------
    // 6. 求交点参数 s：满足 Z = targetZ
    // Cz + s * rwz = targetZ
    // ----------------------------
    if (std::abs(rayWorld.at<float>(2)) < 1e-6f) {
    // 射线平行于平面，无法求交点，返回一个无效点或做一个保守值
    pcl::PointXYZ invalid;
    invalid.x = std::numeric_limits<float>::quiet_NaN();
    invalid.y = std::numeric_limits<float>::quiet_NaN();
    invalid.z = targetZ;
    return invalid;
    }
    float s = (targetZ - Cz) / rwz;

    // ----------------------------
    // 7. 求交点世界坐标
    // ----------------------------
    pcl::PointXYZ world;
    world.x = Cx + s * rwx;
    world.y = Cy + s * rwy;
    world.z = targetZ;

    return world;
}


void MultisensorFusion::SendResult(std::list<Adsfi::HafFusionOut<float>>& fusion_data)
{
    // std::cout<< "Come in SendResult  :" << std::endl;

    // 筛选最近的一个目标
    HafFusionOut<float> min_x_fussion = {};
    float min_x = 60;
    for(auto det : fusion_data) {
        if (det.absRect.center.x < min_x && det.cls == 0){
            min_x_fussion.cls = uint(2);
            min_x_fussion.confidence = det.confidence;
            min_x_fussion.rect.center.x = det.absRect.center.x;
            min_x_fussion.rect.center.y = det.absRect.center.y;
            min_x = det.absRect.center.x;
        }
    }

    //  消除连续帧中的个别漏检，让信号连续。此处会引入 1s 的延时，人走开后 1s 内会判断存在目标
    if (min_x < 60)
    {
        flag = true;
        obj_count += 1;
        if(obj_count > 20) obj_count = 20;      // 去除短时间漏检
        last_obj_dis = min_x;
    }else if(obj_count > 0){
        min_x_fussion.cls = uint(2);
        min_x_fussion.confidence = 0.8f;
        min_x_fussion.rect.center.x = last_obj_dis - 0.1f;          // 手动插值，慢速前进
        min_x_fussion.rect.center.y = 1.0f;
        obj_count -= 1;
    }

//////////////////////////////////////////////////////////////////////
    // MarkArray 可视化
    mdc::visual::MarkerArray vizMarkerArray;// 聚类物队列
    int i = 1;
    for(auto det_m : fusion_data)
    {
        mdc::visual::Marker oneObj;
        //设置物体的坐标系
        oneObj.header.frameId = "map";// 聚类框的坐标系
        //设置物体形状
        oneObj.type = mdc::visual::MarkerType::CUBE;
        //设置物体的ID
        oneObj.id   = i;
        oneObj.ns = "multisensor_obj";  // 设置命名空间
        oneObj.lifetime.sec  = 0;
        oneObj.lifetime.nsec = 100000000;  // 0.1 秒 = 1e8 纳秒
        //设置物体的大小
        oneObj.scale.x = det_m.absRect.size.x;//长
        oneObj.scale.y = det_m.absRect.size.y;//宽
        oneObj.scale.z = det_m.absRect.size.z;//高
        //物体的颜色
        if(det_m.cls == 0){
            oneObj.color.r = 1;//红色
            oneObj.color.g = 0;
            oneObj.color.b = 0;
        }
        else if (det_m.cls == 1){
            oneObj.color.r = 0;//绿色
            oneObj.color.g = 1;
            oneObj.color.b = 0;
        }

        //设置透明程度  0：透明    1：不透明
        oneObj.color.a = 0.8;
        //物体的坐标(物体中心为质点)
        oneObj.pose.position.x =  det_m.absRect.center.x;//3DBox的质心坐标xyz
        oneObj.pose.position.y =  det_m.absRect.center.y;
        oneObj.pose.position.z =  det_m.absRect.center.z;
        //压入一个检测目标
        vizMarkerArray.markers.emplace_back(oneObj);

        i += 1;
    }

    //往Mviz发送聚类结果
    // 坐标系名称：map   话题：/lidar_obj
    bool isPublishob = objPublisher.Publish(vizMarkerArray);
//////////////////////////////////////////////////////////////////////
    // 定位激光雷达坐标，轨迹可视化
    if(!road_point_list->empty()) {
        // 向Mviz发送话题
        mdc::visual::PointCloud<mdc::visual::PointXYZI> data;
        data.header.frameId = "map";// ---------点云坐标
        const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
        uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000UL;
        data.isDense = false;
        data.header.stamp = mdc::visual::Times { sec, nsec };
        for(size_t i = 0; i < road_point_list->size(); i++)
        {
            mdc::visual::PointXYZI ptMviz;
            ptMviz.x = road_point_list->points[i].x;
            ptMviz.y = road_point_list->points[i].y;
            ptMviz.z = 0;
            ptMviz.intensity = (i*10+50)%250;
            data.points.push_back(ptMviz);
        }

        //发布点云数据
        bool isPublishpt = pcPointXYZIRPub.Publish(data);
    }

    // 清空检测结果，只传入x方向最近的一个障碍物
    fusion_data.clear();
    fusion_data.emplace_back(min_x_fussion);
    auto data_result = std::make_shared<HafFusionOutArray<float32_t>>();
    data_result->fusionOut = fusion_data;
    // std::cout << "Fusion_data.size()  : " << data_result->fusionOut.size() << std::endl;
    for(auto det_fu : data_result->fusionOut){
        std::cout << "cls: " << static_cast<unsigned int>(det_fu.cls) << std::endl;
        std::cout << "confidence: " << det_fu.confidence << std::endl;
        std::cout << "distance_X: " << det_fu.rect.center.x << std::endl;
        std::cout << "distance_Y: " << det_fu.rect.center.y << std::endl;
    }

    //     // ========== 写入文件（持续追加） ==========
    // if (min_x_file.is_open() && flag ) {
    //     min_x_file << min_x_fussion.rect.center.x << "\n";   // 写一行
    //     min_x_file.flush();            // 立即落盘
    // }

    // 发送感知结果到下一个节点
    if (node.SendObject(data_result) != HAF_SUCCESS) {
        errorStatus = true;
        HAF_LOG_ERROR << "SendFusedObjectArray failed";
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 控制发送频率
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
    
    std::cout << "No matching trajectory type for IDs: " << id1 << " and " << id2 << std::endl;
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

void MultisensorFusion::getDirect()
{
    while(!node.IsStop())
    {
        // // std::cout << "++++++++++++ getDirect +++++++++++++"<<std::endl;

        // this->model = receiver.receiveMessage()[0];
        // ID_road_start = receiver.receiveMessage()[1];
        // ID_road_last = receiver.receiveMessage()[2];

        /////////////////////     手动修改配置参数     ///////////////////////
        if (ID_road == 0)
        {
            ID_road_start = 12;
            ID_road_last = 15;
        }
        else if (ID_road == 1)
        {
            ID_road_start = 12;
            ID_road_last = 16;
        
        }
        else if (ID_road == 2)
        {
            ID_road_start = 12;
            ID_road_last = 17;
        
        }
        else if (ID_road == 3)
        {
            ID_road_start = 13;
            ID_road_last = 15;
        
        }
        else if (ID_road == 4)
        {
            ID_road_start = 13;
            ID_road_last = 16;
        
        }
        else if (ID_road == 5)
        {
            ID_road_start = 13;
            ID_road_last = 17;
        }
        ///////////////////////////////////////////

        if (this->model >= 0 && ID_road_start >= 0 && ID_road_last >= 0) {  // 检查是否接收成功
            // std::cout << "Received integers: " << this->model << " and " << ID_road_start << " and " << ID_road_last << std::endl;
        } else {
            std::cerr << "Failed to receive valid data" << std::endl;
        }

        if (this->model > 0 && ID_road_start > 0 && ID_road_last > 0 && (ID_road_start != history_ID_road_start || ID_road_last != history_ID_road_last))
        {
            std::cout << "Received integers: " << this->model << " and " << ID_road_start << " and " << ID_road_last << std::endl;

            std::string road_path = getTrajectoryFileNameByIDs(ID_road_start, ID_road_last);
            std::cout << "loadPoints--------------------" << road_path <<std::endl;
            point_list->clear();
            PointProcessing::loadPoints(road_path, point_list);
            std::sort(point_list->points.begin(), point_list->points.end(), [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                return a.x < b.x;  // 按x值升序排序
            });
            history_ID_road_start = ID_road_start;
            history_ID_road_last = ID_road_last;

            std::cout << "point_list  size  :" << point_list->size() <<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}