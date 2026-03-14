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

void Location::Process()
{
    auto curLidarSlam = node.GetLidarSlam(1);
    // Mock output data
    auto out = std::make_shared<HafLocation>();

	// --- 静态文件句柄，只打开一次 ---
    static std::ofstream ofs("location_points.txt", std::ios::app);
	if (!ofs.is_open())
    {
        std::cerr << "无法打开文件 location_points.txt，用于保存定位结果！" << std::endl;
    }

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
		// std::cout<<"组合导航与A3激光雷达状态    = "<<out->header.seq<<std::endl;
		// std::cout<<"rfid的ID                   = "<<out->pose.pose.orientation.x<<std::endl;
		// std::cout<<"rfid的强度值               = "<<out->pose.pose.orientation.y<<std::endl;
		// std::cout<<"x坐标                      = "<<out->pose.pose.position.x<<std::endl;
		// std::cout<<"y坐标                      = "<<out->pose.pose.position.y<<std::endl;
		// std::cout<<"z坐标                      = "<<out->pose.pose.position.z<<std::endl;
		// std::cout<<"yaw                        = "<<out->pose.pose.orientation.z<<std::endl;

		// --- 写入坐标到文件 ---
		// std::cout<<"-----------save_file_flag : " << save_file << std::endl;
        if (ofs.is_open() && save_file)
        {
            ofs << out->pose.pose.position.x << " "
                << out->pose.pose.position.y << " "
                << out->pose.pose.position.z << std::endl;
            ofs.flush();  // 立即写入磁盘
        }

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
	}

    return;
}
