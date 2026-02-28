/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  lidar目标跟踪demo源文件
 */
#include "lidar_tracker.h"
#include "core/core.h"
#include "object/object.h"

using namespace std;
using namespace Adsfi;


LidarObstacleTracking::~LidarObstacleTracking()
{
    for (auto &it : pool) {
        if (it.joinable()) {
            it.join();
        }
    }
}


void LidarObstacleTracking::Process()
{
    while (!node.IsStop())
    {
        std::shared_ptr<Haf3dDetectionOutArray<float32_t> > data;
        if (node.GetDetectionObstacles(data) != HAF_SUCCESS)
        {
            return;
        }
        auto out = std::make_shared<Haf3dDetectionOutArray<float32_t>>();
    	out->frameID      = data->frameID;//iter->objectID;//lidar的ID
    	out->seq         = data->seq;//iter->types;   // 激光雷达异常标志位：0正常,1异常
    	if((int)out->seq == 1)
    	{
    	    std::cout<<"ID:"<<out->frameID<<"  异常:"<<(int)out->seq<<std::endl;
    	}
    	else
    	{
    	    std::cout<<"ID:"<<out->frameID<<"  正常:"<<(int)out->seq<<std::endl;
    	}
        Haf3dDetectionOut<float32_t> out3d;
        list<Haf3dDetectionOut<float32_t>>::iterator iter;
        std::cout<<"障碍物个数 = "<<data->detectionOut3d.size()<<std::endl;
        for (iter = data->detectionOut3d.begin(); iter != data->detectionOut3d.end(); iter++)
        {
        	out3d.cls           = iter->cls;     // 类别
        	std::cout<<" cls = "<<(int)out3d.cls;
        	out3d.rect.center.x = iter->rect.center.x;// 坐标
        	std::cout<<" x = "<<out3d.rect.center.x;
			out3d.rect.center.y = iter->rect.center.y;
			std::cout<<" y = "<<out3d.rect.center.y<<std::endl;
			out3d.rect.center.z = iter->rect.center.z;
	        out3d.rect.size.x   = iter->rect.size.x;// 长宽高
	        out3d.rect.size.y   = iter->rect.size.y;
	        out3d.rect.size.z   = iter->rect.size.z;
	        out->detectionOut3d.push_back(out3d);
        }
        std::cout<<"-------------"<<std::endl;
        if (node.SendResult(out) != HAF_SUCCESS)
        {
            HAF_LOG_ERROR << "Send tracking results failed";
            Stop();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return;
}


void LidarObstacleTracking::PrintDetectionObstacles()
{
    while (!node.IsStop()) {
        std::shared_ptr<Haf3dDetectionOutArray<float32_t> > data;
        if (node.GetDetectionObstacles(data) != HAF_SUCCESS) {
            return;
        }
        HAF_LOG_INFO << "Object array received.";
        frameID = data->frameID;
        seq = data->seq;
        HAF_LOG_INFO << "frameID: " << frameID;
        HAF_LOG_INFO << "seq: " << seq;
        list<Haf3dDetectionOut<float32_t>>::iterator iter;
        for (iter = data->detectionOut3d.begin(); iter != data->detectionOut3d.end(); iter++) {
            HAF_LOG_INFO << "objectID: " << iter->objectID;
            HAF_LOG_INFO << "Rect center is (" << iter->rect.center.x << ", " << iter->rect.center.y << ", " <<
                iter->rect.center.z << ")";
            HAF_LOG_INFO << "Rect size is (" << iter->rect.size.x << ", " << iter->rect.size.y << ", " <<
                iter->rect.size.z << ")";
        }
        HAF_LOG_INFO << "time_sec: " << data->timestamp.sec << ", time_nsec: " << data->timestamp.nsec;
    }
    return;
}


void LidarObstacleTracking::PrintPointCloud(const uint32_t instanceId)
{
    while (!node.IsStop()) {
        std::shared_ptr<LidarFrame<PointXYZIRT> > lidarData;
        if (node.GetPointCloud(lidarData, instanceId) != HAF_SUCCESS) {
            Stop();
            return;
        }
        if (lidarData == nullptr) {
            HAF_LOG_INFO << "Lidar data is null.";
            continue;
        }
        HAF_LOG_INFO << "LidarObstacleTracking Rcv point cloud, points size : " << lidarData->pointCloud.size() <<
            ", instance id : " << instanceId;
        HAF_LOG_INFO << "Lidar time sec: " << lidarData->timestamp.sec;
        HAF_LOG_INFO << "Lidar time nsec: " << lidarData->timestamp.nsec;
    }
    return;
}


void LidarObstacleTracking::PrintLocation()
{
    while (!node.IsStop()) {
        std::shared_ptr<HafLocation> location;
        if (node.GetLocation(location) != HAF_SUCCESS) {
            Stop();
            return;
        }
        HAF_LOG_INFO << "Location xyz: (" << location->pose.pose.position.x << ", " << location->pose.pose.position.y <<
            ", " << location->pose.pose.position.z << ")";
    }
    return;
}


void LidarObstacleTracking::SendResult()
{
    while (!node.IsStop()) {
        HAF_LOG_INFO << "Sending out tracking results.";
        auto out = std::make_shared<Haf3dDetectionOutArray<float32_t>>();
        Haf3dDetectionOut<float32_t> out3d;
        out3d.objectID = 1;
        out3d.cls = 0;
        out3d.confidence = 0;
        out3d.existenceProbability = 1.0;
        out3d.rect.center.x = 1.0;
        out3d.rect.center.y = 1.0;
        out3d.rect.center.x = 1.0;
        out3d.rect.size.x = 1.0;
        out3d.rect.size.y = 1.0;
        out3d.rect.size.z = 0.0;
        out->detectionOut3d.push_back(out3d);
        out->count = 1;
        out->frameID = frameID;
        out->seq = 0;
        out->timestamp.sec = 1;
        out->timestamp.nsec = 1;

        if (node.SendResult(out) != HAF_SUCCESS) {
            HAF_LOG_ERROR << "Send tracking results failed";
            Stop();
            return;
        }
        const uint32_t sleepDuration = 100U;
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepDuration));
    }
    return;
}



