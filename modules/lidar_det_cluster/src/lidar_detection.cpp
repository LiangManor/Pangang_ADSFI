/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
 * Description:  lidar_det demo
 */
#include "lidar_detection.h"
#include "dbscan_inwinic.h"
#include "tracker.h"
#include "weed_removal.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <publisher.h>  //点云Mviz可视化

#include <pcl/io/pcd_io.h>
#include <chrono>
#include <ctime>
#include <string>
#include <iostream>

#include <filesystem>
#include <vector>
#include <algorithm>
#include <thread>

using namespace std;
using namespace Adsfi;
using namespace mdc::visual;
#pragma pack(1)
    struct LidarPointFieldLite {
        int16_t x;
        int16_t y;
        int16_t z;
        int32_t time;
        uint8_t intensity;
        uint16_t ring;
    };
    struct LidarPointField {
        float32_t x;
        float32_t y;
        float32_t z;
        int32_t time;
        float32_t distance;
        float32_t pitch;
        float32_t yaw;
        uint16_t intensity;
        uint16_t ring;
    };
#pragma pack()

void PrintFirstPoint(const std::shared_ptr<ara::lidar::LidarPointCloud> &lidarData)
{}

void PrintFirstPointLite(const std::shared_ptr<ara::lidar::LidarPointCloud> &lidarData)
{}

void PrintPointsInfo(const std::shared_ptr<ara::lidar::LidarPointCloud> &lidarData)
{}

pcl::PointCloud<pcl::PointXYZ>::Ptr point_list(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr road_point_list(new pcl::PointCloud<pcl::PointXYZ>());

LidarDetection::~LidarDetection()
{
    for (auto &iter : pool) {
        if (iter.joinable()) {
            iter.join();
        }
    }
}


struct QuaternionInwinic
{
   double w, x, y, z;
};

QuaternionInwinic ToQuaternionInwinic(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
   double cy = cos(yaw * 0.5);
   double sy = sin(yaw * 0.5);
   double cp = cos(pitch * 0.5);
   double sp = sin(pitch * 0.5);
   double cr = cos(roll * 0.5);
   double sr = sin(roll * 0.5);
   QuaternionInwinic q;
   q.w = cy * cp * cr + sy * sp * sr;
   q.x = cy * cp * sr - sy * sp * cr;
   q.y = sy * cp * sr + cy * sp * cr;
   q.z = sy * cp * cr - cy * sp * sr;
   return q;
}
mdc::visual::Publisher pc1PointXYZIRPub {}; //点云
mdc::visual::Publisher pc2PointXYZIRPub {}; //点云
mdc::visual::Publisher objPublisher {};    //聚类结果发布者
mdc::visual::Publisher guolv_path_Pub {}; //点云
// mdc::visual::Publisher road_path_Pub {}; //点云


void LidarDetection::Process()
{   
    pool.push_back(std::thread(&LidarDetection::SubLidar, this));   // 点云检测子线程
    // pool.push_back(std::thread(&LidarDetection::SubLocation, this));// 获取定位结果子线程
    pool.push_back(std::thread(&LidarDetection::RecvLocation, this));
    pool.push_back(std::thread(&LidarDetection::getDirect, this));
    pool.push_back(std::thread(&LidarDetection::cleanPcdDirectoryThread, this));    //定时删除存放的检测到的障碍物数据
    return;
}

/*
定时删除存放的检测到的障碍物数据
*/
void LidarDetection::cleanPcdDirectoryThread()
{
    using namespace std::chrono_literals;
    const std::string dirPath{"have_obj_data"};
    size_t maxFiles = 5000;


    while (true) 
    {
        try {
            // ---- 1. 等待 30 分钟 ----
            std::this_thread::sleep_for(30min);

            // ---- 2. 检查目录是否存在 ----
            if (!std::filesystem::exists(dirPath)) {
                // std::cout << "[PCD Cleaner] Directory not found: " << dirPath << std::endl;
                continue;
            }

            std::vector<std::filesystem::directory_entry> pcdFiles;

            // ---- 3. 读取所有 .pcd 文件 ----
            for (auto& entry : std::filesystem::directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                    pcdFiles.push_back(entry);
                }
            }

            size_t fileCount = pcdFiles.size();
            if (fileCount <= maxFiles) {
                // 文件数量正常
                // std::cout << "[PCD Cleaner] File count: " << fileCount << " (OK)" << std::endl;
                continue;
            }

            // ---- 4. 按修改时间排序 → 最早的排前面 ----
            std::sort(pcdFiles.begin(), pcdFiles.end(),
                [](auto& a, auto& b) {
                    return std::filesystem::last_write_time(a) <
                           std::filesystem::last_write_time(b);
                });

            // ---- 5. 需要删除的数量 ----
            size_t toDelete = fileCount - maxFiles;

            // std::cout << "[PCD Cleaner] Found " << fileCount
            //           << " files, deleting " << toDelete
            //           << " oldest files..." << std::endl;

            // ---- 6. 删除最老的文件 ----
            for (size_t i = 0; i < toDelete; i++) {
                try {
                    std::filesystem::remove(pcdFiles[i]);
                } catch (std::exception& e) {
                    std::cerr << "[PCD Cleaner] Failed to delete "
                              << pcdFiles[i].path() << ": " << e.what() << std::endl;
                }
            }

            // std::cout << "[PCD Cleaner] Cleanup completed. Remaining: "
            //           << (fileCount - toDelete) << std::endl;

        } catch (std::exception& e) {
            std::cerr << "[PCD Cleaner] Exception: " << e.what() << std::endl;
        }
    }
}


/*
获取定位信息，使用定位信息截取前方轨迹，使用轨迹过滤保留轨道上危险区域的点云进行聚类。
*/
void LidarDetection::RecvLocation()
{
    // std::cout<<"*************** get RecvLocation data ***********"<<std::endl;
    while (!node.IsStop()) {
        std::shared_ptr<HafLocation> data;
        if (node.GetLocation(data) != HAF_SUCCESS) {
            HAF_LOG_ERROR <<"*************** GetLocation Failed ***********";
            continue;
        }
        Point_Location.x = data->pose.pose.position.x;
        Point_Location.y = data->pose.pose.position.y;
        Point_Location.z = data->pose.pose.position.z;
        float Yaw_Location = data->pose.pose.orientation.z;

        // std::cout << "Point_Location.x :" << Point_Location.x << std::endl;
        // std::cout << "Point_Location.y :" << Point_Location.y << std::endl;
        // std::cout << "Yaw_Location :" << Yaw_Location << std::endl;

        if (point_list->empty()){
            // HAF_LOG_WARN << "point_list is Empty, Please check Road_file.";
            continue;
        }
        road_point_list->clear();
        //根据定位点，加载前方一段距离内的轨迹点 road_point_list （是从 point_list 中提取出来的）
        PointProcessing::findClosestAndFit(point_list, Point_Location, Yaw_Location, model, road_point_list);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void saveCloudAsPCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudROI)
{
    static std::chrono::steady_clock::time_point lastSaveTime =
        std::chrono::steady_clock::now() - std::chrono::seconds(10);

    // ---- 1. 判断是否小于 2 秒 ----
    auto now = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - lastSaveTime);
    if (diff.count() < 2) {
        return; // 2秒内保存过，不再保存
    }

    lastSaveTime = now; // 更新保存时间

    // ---- 2. 检查并创建目录 ----
    std::string dirName = "have_obj_data";
    if (!std::filesystem::exists(dirName)) {
        std::filesystem::create_directories(dirName);
        // std::cout << "Created directory: " << dirName << std::endl;
    }

    // ---- 3. 生成文件名 年_月_日_时_分_秒.pcd ----
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tmTime;
#ifdef _WIN32
    localtime_s(&tmTime, &t);
#else
    localtime_r(&t, &tmTime);
#endif

    char filename[256];
    snprintf(filename, sizeof(filename),
             "%s/%04d_%02d_%02d_%02d_%02d_%02d.pcd",
             dirName.c_str(),
             tmTime.tm_year + 1900,
             tmTime.tm_mon + 1,
             tmTime.tm_mday,
             tmTime.tm_hour,
             tmTime.tm_min,
             tmTime.tm_sec
    );

    std::string outputFilename(filename);

    // ---- 4. 保存为 pcd 二进制格式 ----
    if (!cloudROI || cloudROI->empty()) {
        std::cerr << "saveCloudAsPCD: cloudROI is empty, skip save." << std::endl;
        return;
    }

    if (pcl::io::savePCDFileBinary(outputFilename, *cloudROI) == -1) {
        std::cerr << "Failed to save PCD file: " << outputFilename << std::endl;
    } else {
        // std::cout << "Saved PCD: " << outputFilename
        //           << "  (" << cloudROI->size() << " points)" << std::endl;
    }
}


/*
主线程在这里，主要从这里开始处理点云的过滤和聚类。
*/
void LidarDetection::SubLidar()
{
    bool result     = mdc::visual::Connect();
    pc1PointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("lidar_det_ground_cloud"));// 点云的话题
    pc2PointXYZIRPub = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("lidar_det_nogrou_cloud"));// 点云的话题
    objPublisher    = mdc::visual::Publisher::Advertise<mdc::visual::MarkerArray>(ara::core::String("lidar_obj"));// 聚类框的话题
    guolv_path_Pub  = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("guolv_path_cloud"));// 点云的话题
    // road_path_Pub   = mdc::visual::Publisher::Advertise<mdc::visual::PointCloud2>(ara::core::String("road_path_cloud"));// 点云的话题
        // 开启单独线程 getDirect() 去获取行驶方向
    // pool.push_back(std::thread(&LidarDetection::getDirect, this));
    TRACKER tracker;
        // ✅ 初始化 GroundFilter
    ground_filter_.setGroundThickness(ground_roi_z);            // 离地面过滤高度
    // ground_filter_.enableRailDetection(1.5f, Eigen::Vector3f(1, 0, 0));
        //加载 lidar2lidar 外参
    PointProcessing::loadTransformFromYaml("Config.yaml");
    while (!node.IsStop()) 
    {
    	// 获取当前行驶方向的lidarID
        const uint32_t lidarID = getLidarID(this->model);
        if(lidarID == 0) // 表示不进行检测
        {
                auto out = std::make_shared<Haf3dDetectionOutArray<float>>();
                out->frameID        = std::to_string(0);
                out->seq            = 0;
                if (node.SendObject(out) != HAF_SUCCESS)
                {
                    continue;
                }
            continue;
        }
        // int lidar_ID_test = 6;
    	// 只拿对应方向的lidar数据A4:4  B2:6
		std::shared_ptr<LidarFrame<PointXYZIRT> > lidarData;
		if (node.GetLidar(lidarData, lidarID) != HAF_SUCCESS)
		// if (node.GetLidar(lidarData, 4) != HAF_SUCCESS) // A4
		// if (node.GetLidar(lidarData, 6) != HAF_SUCCESS) // B2
		{
            // std::cout<<" ----------------get lidardata Failed------------------ "<<std::endl;
			Stop();
			return;
		}
		if (lidarData == nullptr)
		{
            // std::cout<<" ----------------lidardata if nullptr------------------ " <<std::endl;
			continue;
		}
		// 先判断传感器是否出故障，如果出故障则直接上报，跳过以下步骤。
		if(lidarData->pointCloud.size() == 0)
		{
			auto out = std::make_shared<Haf3dDetectionOutArray<float>>();
			out->frameID        = std::to_string(this->model);// 传感器id-----------------------
			out->seq            = 1;// 1表示异常 -------------------------------
			if (node.SendObject(out) != HAF_SUCCESS)
			{
				HAF_LOG_ERROR << "Send Object Failed!";
				Stop();
				return;
			}
			continue;
		}
		// ------------------------------------------- 获取原始数据 ---------------------------------------------- //
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		PointT p;
		for (size_t i = 0; i < lidarData->pointCloud.size(); i++)
		{
			p.x = lidarData->pointCloud[i].x;
			p.y = lidarData->pointCloud[i].y;
			p.z = lidarData->pointCloud[i].z;
			cloud->push_back(p);
		}

        // ------------------------------------------- 提取感兴趣区域 ---------------------------------------------- //
        // std::cout << "***********************  debug  *********************  : " <<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudROI(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noGround(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr Ground(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i = 0; i < cloud->points.size();i++)
        {
            if     (std::abs(cloud->points[i].y) >= 10) // 左右
            {
                continue;
            }
            else if(std::abs(cloud->points[i].x) >= 70) // 前后
            {
                continue;
            }
            else if(std::abs(cloud->points[i].z) >= 3) // 上下
            {
                continue;
            }
            else if (road_point_list->size() > 2)
            {
                // 确定点所属的轨迹点范围
                for (size_t j = 0; j+2 < road_point_list->size(); j+=2) {
                    if ((cloud->points[i].x >= road_point_list->points[j].x) && (cloud->points[i].x < road_point_list->points[j + 2].x)) {
                        // 获取对应轨迹点的 y,z 值
                        float trajectoryY = road_point_list->points[j].y;
                        // float trajectoryZ = road_point_list->points[j].z - road_point_list->points[0].z - 0.4f;
                        // float trajectoryZ = ground_roi_z;
                        // 检查 y,z 值范围
                        if (cloud->points[i].y >= trajectoryY - 1.2 && cloud->points[i].y <= trajectoryY + 1.2) {
                            cloudROI->points.push_back(cloud->points[i]);
                            // if((cloud->points[i].z > trajectoryZ)){
                            //     noGround->points.push_back(cloud->points[i]);
                            // }else{
                            //     Ground->points.push_back(cloud->points[i]);
                            // }
                        }
                        break;
                    }
                }
            }
        }

        // 检测到目标物后，保存点云
        if (have_obj > 0) saveCloudAsPCD(cloudROI);

        //地面过滤
        if(cloudROI->size()<10) continue;
        ground_filter_.segment(cloudROI, Ground, noGround);

        // std::cout << "*********  debug  *********  :  cloudROI after"<<std::endl;
        // std::cout << "************   IOU_Cloud   **********  : " << road_point_list->size() <<std::endl;
        // ------------------------------------------- 对非地面点进行分割 ---------------------------------------------- //
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filt(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i = 0; i < noGround->points.size();i++)
        {
            if(noGround->points[i].z <= (-0.5 + 0.25))
            {
                noFilt->points.push_back(noGround->points[i]);
            }
            else
            {
                filt->points.push_back(noGround->points[i]);
            }
        }
        // //点云滤波处理--  针对  上层空间的点云
        // pcl::VoxelGrid<pcl::PointXYZ> voxel;
        // voxel.setInputCloud(filt);
        // voxel.setLeafSize(0.15,0.15,0.1);
        // voxel.filter(*filt);//获取滤波后的点云：cloudFiltered
        // ------------------------------------------- 对  近地面点 noFilt 进行深度分割 ---------------------------------------------- //
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt_0_10 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt_10_30(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt_30_x (new pcl::PointCloud<pcl::PointXYZ>);
        for(int i = 0; i < noFilt->points.size();i++)
        {
            if(noFilt->points[i].x <= 10)
            {
                noFilt_0_10->points.push_back(noFilt->points[i]);
            }
            else if((10 < noFilt->points[i].x) && (noFilt->points[i].x <= 30))
            {
                noFilt_10_30->points.push_back(noFilt->points[i]);
            }
            else
            {
                noFilt_30_x->points.push_back(noFilt->points[i]);
            }
        }
        // ------------------------------------------- 近处(30m以内)杂草去除 ---------------------------------------------- //
        // 1. 0~10m
        WEEDREMOVAL weedRemoval;
        weedRemoval.ang_res_h = 0.2;
        weedRemoval.ang_res_v = 0.2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt_0_10_weedRemoval(new pcl::PointCloud<pcl::PointXYZ>);
        weedRemoval.weedRemoval(noFilt_0_10,noFilt_0_10_weedRemoval);
        weedRemoval.ang_res_h = 0.1;
        weedRemoval.ang_res_v = 0.1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr noFilt_10_30_weedRemoval(new pcl::PointCloud<pcl::PointXYZ>);
        weedRemoval.weedRemoval(noFilt_10_30,noFilt_10_30_weedRemoval);

        // ------------------------------------------- 对非地面 上层点 filt进行深度分割 ---------------------------------------------- //
        pcl::PointCloud<pcl::PointXYZ>::Ptr filt_0_30(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filt_30_x(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i = 0; i < filt->points.size();i++)
        {
            if(filt->points[i].x <= 30)
            {
                filt_0_30->points.push_back(filt->points[i]);
            }
            else
            {
                filt_30_x->points.push_back(filt->points[i]);
            }
        }
        // //点云滤波处理--  针对  近距离上层空间的点云
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(filt_0_30);
        voxel.setLeafSize(0.15,0.15,0.1);
        voxel.filter(*filt_0_30);//获取滤波后的点云：cloudFiltered
        // ------------------------------------------- 点云拼接 ---------------------------------------------- //
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0_30(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_30_x(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_0_30 = *filt_0_30 + *noFilt_0_10_weedRemoval + *noFilt_10_30_weedRemoval;
        *cloud_30_x = *filt_30_x + *noFilt_30_x;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud_ = *cloud_0_30 + *cloud_30_x;
        // ------------------------------------------- 目标检测 ---------------------------------------------- //
        KDTREEDBSCAN dbscanCluster_0_30, dbscanCluster_30_x;
        // dbscanCluster_0_30.setParam(.1, .15, 3, 10, 1000);// 参数1:xGate_  参数2:yGate_   参数3:minPoints_   参数4: 
        dbscanCluster_0_30.setParam(.1, .15, 3, 20, 1000);
        // dbscanCluster_30_x.setParam(.15, .15, 3, 10, 1000);
        dbscanCluster_30_x.setParam(.15f, .25f, 3, 8, 1000);       // 60m 处蹲下的人,身上大概均匀分布点,z方向 0.26m 距离,y方向 0.18m 距离.x方向 很小.
        dbscanCluster_0_30.clustering(cloud_0_30);
        dbscanCluster_30_x.clustering(cloud_30_x);
		dbscanCluster_0_30.BoundingBox();
		dbscanCluster_30_x.BoundingBox();
        // ------------------------------------------- 目标跟踪 ---------------------------------------------- //
        std::vector<BoundingBox> lidarDets;
		for(int i = 0; i < dbscanCluster_0_30.obj.size(); i++)
		{
		    BoundingBox oneBox;
            oneBox.center.x  = dbscanCluster_0_30.obj[i].center.x;
            oneBox.center.y  = dbscanCluster_0_30.obj[i].center.y;
            oneBox.center.z  = dbscanCluster_0_30.obj[i].center.z;
            oneBox.l         = dbscanCluster_0_30.obj[i].length;
            oneBox.w         = dbscanCluster_0_30.obj[i].width;
            oneBox.h         = dbscanCluster_0_30.obj[i].height;
            oneBox.yaw       = dbscanCluster_0_30.obj[i].yaw;
            oneBox.realType  = "1";// 默认给0
            lidarDets.push_back(oneBox);
        }
		for(int i = 0; i < dbscanCluster_30_x.obj.size(); i++)
		{
		    BoundingBox oneBox;
            oneBox.center.x  = dbscanCluster_30_x.obj[i].center.x;
            oneBox.center.y  = dbscanCluster_30_x.obj[i].center.y;
            oneBox.center.z  = dbscanCluster_30_x.obj[i].center.z;
            oneBox.l         = dbscanCluster_30_x.obj[i].length;
            oneBox.w         = dbscanCluster_30_x.obj[i].width;
            oneBox.h         = dbscanCluster_30_x.obj[i].height;
            oneBox.yaw       = dbscanCluster_30_x.obj[i].yaw;
            oneBox.realType  = "1";// 默认给0
            lidarDets.push_back(oneBox);
        }
        std::vector<BoundingBox> track_result;
        float timestamp_1 = -1.;// 没用到
        tracker.track(lidarDets, track_result,timestamp_1);
        // -------------------------------------------  聚类结果可视化   ---------------------------------------------- //
        // 可视化
        if(this->mvizVision)
        {
            //往Mviz发送滤波后的ROI区域
            mdc::visual::PointCloud<mdc::visual::PointXYZRGB> data;// 发往mviz的数据流
            data.header.frameId = "map";// ---------点云坐标
            data.isDense = false;
            const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
            uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
            uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000UL;
            data.header.stamp = mdc::visual::Times { sec, nsec };
            for (size_t i = 0; i < noGround->points.size(); i++)
            {
                PointT pt;
                pt.x = noGround->points[i].x;
                pt.y = noGround->points[i].y;
                pt.z = noGround->points[i].z;
                mdc::visual::PointXYZRGB ptMviz(pt.x, pt.y, pt.z, 255, 10, 10);// x y z b g r
                data.points.push_back(ptMviz);
            }
            pc2PointXYZIRPub.Publish(data);
            /*************************************************/                
            mdc::visual::PointCloud<mdc::visual::PointXYZRGB> data2;// 发往mviz的数据流
            data2.header.frameId = "map";// ---------点云坐标
            data2.isDense = false;
            data2.header.stamp = mdc::visual::Times { sec, nsec };
            for (size_t i = 0; i < Ground->size(); i++)
            {
                PointT pt;
                pt.x = Ground->points[i].x;
                pt.y = Ground->points[i].y;
                pt.z = Ground->points[i].z;
                mdc::visual::PointXYZRGB ptMviz(pt.x, pt.y, pt.z, 200, 200, 200);// x y z b g r
                data2.points.push_back(ptMviz);
            }
            pc1PointXYZIRPub.Publish(data2);               
            /*************************************************/                
            mdc::visual::PointCloud<mdc::visual::PointXYZRGB> road;// 发往mviz的数据流
            road.header.frameId = "map";// ---------点云坐标
            road.isDense = false;
            road.header.stamp = mdc::visual::Times { sec, nsec };
            for (size_t i = 0; i < road_point_list->size(); i++)
            {
                PointT pt;
                pt.x = road_point_list->points[i].x;
                pt.y = road_point_list->points[i].y;
                pt.z = road_point_list->points[i].z;
                mdc::visual::PointXYZRGB ptMviz(pt.x, pt.y, pt.z, 255, 255, 255);// x y z b g r
                road.points.push_back(ptMviz);
            }
            guolv_path_Pub.Publish(road);
            /*************************************************/
            // mdc::visual::PointCloud<mdc::visual::PointXYZRGB> all_road;// 发往mviz的数据流
            // all_road.header.frameId = "map";// ---------点云坐标
            // all_road.isDense = false;
            // all_road.header.stamp = mdc::visual::Times { sec, nsec };
            // for (size_t i = 0; i < point_list->size(); i++)
            // {
            // 	PointT pt;
            // 	pt.x = point_list->points[i].x;
            // 	pt.y = point_list->points[i].y;
            // 	pt.z = 0;
            // 	mdc::visual::PointXYZRGB ptMviz(pt.x, pt.y, pt.z, 255, 255, 255);// x y z b g r
            // 	all_road.points.push_back(ptMviz);
            // }
            // road_path_Pub.Publish(all_road);
            /*************************************************/
            //往Mviz发送聚类结果
            // 坐标系名称：map   话题：/lidar_obj
            // objPublisher.Publish(vizMarkerArray);
        }

		mdc::visual::MarkerArray vizMarkerArray;// 聚类物队列
		float minDis = 999.0;
		int label = 0;
		if(track_result.size() == 0)
		{
			auto out = std::make_shared<Haf3dDetectionOutArray<float>>();
			out->frameID        = std::to_string(this->model);
			out->seq            = 0;
			node.SendObject(out) ;

		    continue;
		}
		for(int i = 0; i < track_result.size(); i++)
		{
			mdc::visual::Marker oneObj;
			oneObj.lifetime = Times(0,100000000);
			//设置物体的坐标系
			oneObj.header.frameId = "map";// 聚类框的坐标系
			//设置物体形状
			oneObj.type = mdc::visual::MarkerType::CUBE;
			//设置物体的ID
			oneObj.id   = i;
            oneObj.ns = "lidar_det_obj";  // 设置命名空间
            oneObj.lifetime.sec  = 0;
            oneObj.lifetime.nsec = 100000000;  // 0.1 秒 = 1e8 纳秒
			//设置物体的大小
			oneObj.scale.x = track_result[i].l;//长
			oneObj.scale.y = track_result[i].w;//宽
			oneObj.scale.z = track_result[i].h;//高
			//物体的颜色
			oneObj.color.r = 1;//红色
			oneObj.color.g = 0;
			oneObj.color.b = 0;
			//设置透明程度  0：透明    1：不透明
			oneObj.color.a = 0.5;
			//物体的坐标(物体中心为质点)
			oneObj.pose.position.x =  track_result[i].center.x;//3DBox的质心坐标xyz
			oneObj.pose.position.y =  track_result[i].center.y;
			oneObj.pose.position.z =  track_result[i].center.z;

			float tempDis = std::sqrt(  oneObj.pose.position.x * oneObj.pose.position.x
									  + oneObj.pose.position.y * oneObj.pose.position.y
									  + oneObj.pose.position.z * oneObj.pose.position.z);
			if(tempDis <= minDis)
			{
				minDis = tempDis;
				label = i;
			}
			//物体的坐标(四元素)
			QuaternionInwinic q = ToQuaternionInwinic( track_result[i].yaw, 0.0, 0.0);
			oneObj.pose.orientation.x = q.x;
			oneObj.pose.orientation.y = q.y;
			oneObj.pose.orientation.z = q.z;
			oneObj.pose.orientation.w = q.w;
			//压入一个检测目标
			vizMarkerArray.markers.emplace_back(oneObj);
		}
		// 最近的障碍物为红色，其他的为绿色
		// vizMarkerArray.markers[label].color.r = 1;
		// vizMarkerArray.markers[label].color.g = 0;
		// vizMarkerArray.markers[label].color.b = 0;

		// 往下游节点发送数据
		auto out = std::make_shared<Haf3dDetectionOutArray<float>>();
		out->frameID        = std::to_string(this->model);// 传感器id-----------------------
		out->seq            = 0;// 0表示正常 -------------------------------
		// 最近障碍物放在最前边
		Haf3dDetectionOut<float32_t> out3d;
		out3d.cls = 1;// 类别
		out3d.rect.center.x = vizMarkerArray.markers[label].pose.position.x;// 坐标
		out3d.rect.center.y = vizMarkerArray.markers[label].pose.position.y;
		out3d.rect.center.z = vizMarkerArray.markers[label].pose.position.z;
		out3d.rect.size.x   = vizMarkerArray.markers[label].scale.x;// 尺寸
		out3d.rect.size.y   = vizMarkerArray.markers[label].scale.y;
		out3d.rect.size.z   = vizMarkerArray.markers[label].scale.z;
		out->detectionOut3d.push_back(out3d);
		// 再填入其他障碍物
		for(int i = 0; i < vizMarkerArray.markers.size(); i++)
		{
			if( i == label ) continue; // 该障碍物已经填入队列，所以跳过
			Haf3dDetectionOut<float32_t> out3d;
			out3d.cls = 1;
			out3d.rect.center.x = vizMarkerArray.markers[i].pose.position.x;// 坐标
			out3d.rect.center.y = vizMarkerArray.markers[i].pose.position.y;
			out3d.rect.center.z = vizMarkerArray.markers[i].pose.position.z;
			out3d.rect.size.x   = vizMarkerArray.markers[i].scale.x;// 尺寸
			out3d.rect.size.y   = vizMarkerArray.markers[i].scale.y;
			out3d.rect.size.z   = vizMarkerArray.markers[i].scale.z;
			out->detectionOut3d.push_back(out3d);
		}
		if (node.SendObject(out) != HAF_SUCCESS)
		{
			HAF_LOG_ERROR << "Send Object failed!";
			Stop();
			return;
		}
		// 可视化
		if(this->mvizVision)
		{
			objPublisher.Publish(vizMarkerArray);
		}
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

void LidarDetection::getDirect()
{
    while(!node.IsStop())
    {
        // // std::cout << "++++++++++++ getDirect +++++++++++++"<<std::endl;

        // this->model = receiver.receiveMessage()[0];
        // ID_road_start = receiver.receiveMessage()[1];
        // ID_road_last = receiver.receiveMessage()[2];
        // have_obj = receiver.receiveMessage()[3];

        // /////////////////////     手动修改配置参数     ///////////////////////
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
            // std::cout << "Received integers: " << this->model << " and " << ID_road_start << " and " << ID_road_last << std::endl;

            std::string road_path = getTrajectoryFileNameByIDs(ID_road_start, ID_road_last);
            // std::cout << "loadPoints--------------------" << road_path <<std::endl;
            point_list->clear();
            PointProcessing::loadPoints(road_path, point_list);
            std::sort(point_list->points.begin(), point_list->points.end(), [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                return a.x < b.x;  // 按x值升序排序
            });
            history_ID_road_start = ID_road_start;
            history_ID_road_last = ID_road_last;

            // std::cout << "point_list  size  :" << point_list->size() <<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}