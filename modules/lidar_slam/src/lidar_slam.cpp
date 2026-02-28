#include "lidar_slam.h"

#include <cmath> // 包含 atan2 和 M_PI
#include <iostream>
#include <thread>
#include <chrono>
#include <pcl/common/pca.h>

using namespace std;
using namespace Adsfi;

// 用于可视化的配准后的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr currentScanPC(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr currentBottomPC(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr mapBottomPC(new pcl::PointCloud<pcl::PointXYZ>);
// 用于可视化的传感器轨迹
mdc::visual::PointCloud<mdc::visual::PointXYZRGB> lidarHistoryTrajectory;
mdc::visual::PointCloud<mdc::visual::PointXYZRGB> gnssHistoryTrajectory;
mdc::visual::PointCloud<mdc::visual::PointXYZRGB> imuHistoryTrajectory;
float historyYaw;
// 车辆状态
vehicleState egoState;
float score = 9999;
int counterLidar = -1;
// 初始化卡尔曼滤波器
EKF ekfFilter;

//旋转矩阵转欧拉角
Eigen::Vector3f rotationMatrixToEulerAnglesZYX(const Eigen::Matrix3f& R) {
    float m10 = R(1, 0);
    float m11 = R(1, 1);
    float m12 = R(1, 2);
    float m21 = R(2, 1);
    float m22 = R(2, 2);
    float pitch = -std::asin(m12);
    if (std::abs(m12) < 1.0) {
        float roll = std::atan2(m21, m22);
        float yaw = std::atan2(m10, m11);
        return Eigen::Vector3f(yaw, pitch, roll);
    }
    else
    {
        return Eigen::Vector3f(0, pitch, 0);
    }
}

Adsfi::HafStatus LidarSlam::Init()
{
	// 创建文件文本
	this->file_write.open(filePath);
	this->xyzyawFile.open("xyzyaw.txt");
	// ndt 参数
	this->ndt_omp = boost::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
	this->ndt_omp->setResolution(this->ndtGridResolution);//设置NDT网格结构的分辨率（必须大于0.1） 1.5
	this->ndt_omp->setStepSize(this->step);//为More-Thuente线搜索设置最大步长，太小容易陷入局部最优解  0.6
	this->ndt_omp->setMaximumIterations (this->iterations);//设置匹配迭代的最大次数
	this->ndt_omp->setTransformationEpsilon (this->precision);//设置求解精度（迭代增量的大小）   0.01
	this->ndt_omp->setNumThreads(this->threadNum);//设置线程数
	this->ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

	// 导入地图
	importMap();
	std::cout<<" <----------------- ndt 算法参数 --------------------> "<<std::endl;
	std::cout<<" initX      "<<initX<<std::endl;
	std::cout<<" initY      "<<initY<<std::endl;
	std::cout<<" initZ      "<<initZ<<std::endl;
	std::cout<<" initThetaX "<<initThetaX<<std::endl;
	std::cout<<" initThetaY "<<initThetaY<<std::endl;
	std::cout<<" initThetaZ "<<initThetaZ<<std::endl;
	std::cout<<" voxelGridSize     "<<voxelGridSize<<std::endl;
	std::cout<<" ndtGridResolution "<<ndtGridResolution<<std::endl;
	std::cout<<" precision         "<<precision<<std::endl;
	std::cout<<" step              "<<step<<std::endl;
	std::cout<<" iterations        "<<iterations<<std::endl;
	std::cout<<" threadNum         "<<threadNum<<std::endl;
	std::cout<<" <----------------- 工作模式 --------------------> "<<std::endl;
	if(this->workingMode == 0)
	{
		std::cout<<"标定模式"<<std::endl;
		std::cout<<"待标定文件路径:"<<filePath<<std::endl;
	}
	else
	{
		std::cout<<"普通模式"<<std::endl;
	}

	this->zAngle     = this->zAngle     * (3.1415926536/180.0);

	// 最初考虑的是每个工位点对应经纬度信息，但现场rtk信号不好，所以方法1只用于保留，并未使用，而是使用方法2
	// 方法1:获取每个工位点对应的坐标信息（上线点格式为：lat-lon-heading(大地坐标系)）
	rfid.getOnlinePoint(this->rfidPath);
	// 方法2: 获取每个工位点对应的坐标信息（上线点格式为：x-y-yaw）
	getOnlineX_Y_Yaw(this->rfidPath);
	// 与rfid标签机（型号RF650R）建立udp通讯
	rfid.connectRF650R();
	pool.push_back(std::thread(&LidarSlam::threadGetRfid, this));

    // ==================== ABC重定位功能 - 初始化开始 ====================
    initRelocSystem();
    // ==================== ABC重定位功能 - 初始化结束 ====================

	return node.Init(); // 初始化LidarSlam框架对象
};

Eigen::Vector3d LidarSlam::axis_aligned_variances(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    if (cloud->empty()) return Eigen::Vector3d::Zero();

    // 均值
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& p : cloud->points) mean += Eigen::Vector3d(p.x, p.y, p.z);
    mean /= double(cloud->size());

    // 方差（无偏或有偏按需，这里用有偏）
    double sx=0, sy=0, sz=0;
    for (const auto& p : cloud->points) {
        sx += (p.x - mean.x()) * (p.x - mean.x());
        sy += (p.y - mean.y()) * (p.y - mean.y());
        sz += (p.z - mean.z()) * (p.z - mean.z());
    }
    const double n = double(cloud->size());
    return { sx / n, sy / n, sz / n };  // 分别就是 x/y/z 轴方向的方差
}

void LidarSlam::Process()
{
    auto curLidar = node.GetLidar(1);
    if (curLidar.empty() || curLidar.front()->pointCloud.size()  == 0)
	{
    	std::cout << "当前激光雷达未上传数据，属于断流状态，发送故障停车信号" << std::endl;
    	return;
	}

//	// 获取当前的 GNSS 状态
//	auto curGnss = node.GetGnss(1); // 该函数为阻塞式
//	const bool lidarError = curLidar.front()->pointCloud.empty();
//	const bool insError   = (curGnss.front()->header.seq == 1);
//
//	// 判断激光雷达是否异常
//	out->header.seq = lidarError ? (insError ? 11:1) : (insError ? 10:0);	// 第1个1代表激光雷达异常，第2个1代表ins异常。

	if (curLidar.front()->seq == counterLidar)
	{
		sendLocation();
		return;
	}
	// 更新校验码
	counterLidar = curLidar.front()->seq;

	// 数据结构转换
	pcl::PointXYZ p;
	currentScanPC->clear();
	currentBottomPC->clear();
	for (size_t i = 0; i < curLidar.front()->pointCloud.size(); i++)
	{
		p.x = curLidar.front()->pointCloud[i].x;
		p.y = curLidar.front()->pointCloud[i].y;
		p.z = curLidar.front()->pointCloud[i].z;
//		if(p.z < -2.2){
//			currentBottomPC->push_back(p);
//			continue;
//		}
		currentBottomPC->push_back(p);
		currentScanPC->push_back(p);
	}

	// 初始化
	if(this->initLocation)
	{
		calInitPosture(currentScanPC);
		reloc_state_ = RelocState::IDLE;
	}
	// 车辆状态已经进行了重定位初始化
	else
	{
		// ==================== ABC重定位功能 - 状态检查开始 ====================
	    double min_distance;
	    int nearest_index = findNearestRelocPoint(min_distance);
	    std::cout << "reloc_state_ = " << static_cast<int>(reloc_state_) << std::endl;
	    switch (reloc_state_) {
	        case RelocState::IDLE:
			{
				alignedPC = align(currentScanPC, score);
				std::cout << "aligned score = " << score << std::endl;
				if (lossLocNum > 5)
				{
					ekfFilter.first_init_ = false;
					ekfFilter.second_init_ = false;
					egoState.init = false;
					this->initLocation = true;
					break;
				}
				float distance_between_2_frames =std::sqrt(std::pow(egoState.x - this->lastLocation(0, 3), 2) + std::pow(egoState.y - this->lastLocation(1, 3), 2) + std::pow(egoState.z - this->lastLocation(2, 3), 2) );
				if(score > 500.0 || distance_between_2_frames > 10.0)// 如果被完全遮挡
				{
					std::cout<<"距离跳变，发送故障停车信号" << std::endl;
					lossLocNum ++;
					break;
				}
				lossLocNum = (lossLocNum > 0) ? lossLocNum - 1 : 0;

				// (更新lidar定位)
				egoState.x                      = this->lastLocation(0, 3);
				egoState.y                      = this->lastLocation(1, 3);
				egoState.z                      = this->lastLocation(2, 3);
				Eigen::Matrix3f R   = this->lastLocation.block<3, 3>(0, 0);
				Eigen::Vector3f eulerAngles = rotationMatrixToEulerAnglesZYX(R);
				egoState.yaw = eulerAngles(0);
				historyYaw = egoState.yaw;
				// 对轨迹进行滤波
				ekfFilter.run(egoState);

				// 状态1: 空闲状态，检查是否接近重定位点
				if (min_distance < reloc_distance_threshold_ && trainDirection == 0) {
					reloc_state_ = RelocState::WAITING_FOR_TRIGGER;
					current_reloc_point_index_ = nearest_index;
					std::cout << "[重定位] 接近重定位点 " << reloc_points_[nearest_index].name
							  << " (距离: " << min_distance << " 米)" << std::endl;
				}
				break;
			}

	        case RelocState::WAITING_FOR_TRIGGER:
	            // 状态3: 等待触发，持续发布冻结位置
	            // 检查是否收到触发信号
				{
					std::lock_guard<std::mutex> state_lock(reloc_state_mutex_);
					std::cout << "WAITING_FOR_TRIGGER, 当前状态:" << reloc_trigger_received_ << std::endl;
					if (trainDirection != 0) {
						reloc_state_ = RelocState::RELOCATING;
						std::cout << "[重定位] 开始执行重定位，点 "
								  << reloc_points_[current_reloc_point_index_].name << "..." << std::endl;
					}
					if(trainDirection != 0 && reloc_state_ == RelocState::WAITING_FOR_TRIGGER){
						std::cout << "速度已有但转盘未解锁，发送故障停车信号" << std::endl;
					}
				}
	            break;

	        case RelocState::RELOCATING:
	        {
	            // 状态4: 重定位中
	        	ekfFilter.first_init_ = false;
	        	ekfFilter.second_init_ = false;
	        	reloc_state_ = RelocState::LEAVING;

	            std::cout << "[重定位] 重定位完成，点 "
	                      << reloc_points_[current_reloc_point_index_].name
	                      << "，开始离开..." << std::endl;
	            break;
	        }

	        case RelocState::LEAVING:
	        {
				// 状态5: 离开中，检查是否已离开重定位区域
				float score;
				alignedPC = align(currentScanPC, score);
				std::cout << "aligned score = " << score << std::endl;
				if (lossLocNum > 5)
				{
					ekfFilter.first_init_ = false;
					ekfFilter.second_init_ = false;
					egoState.init = false;
					this->initLocation = true;
					break;
				}
				if(score > 500.0)// 如果被完全遮挡
				{
					lossLocNum ++;
					break;
				}
				lossLocNum = (lossLocNum > 0) ? lossLocNum - 1 : 0;
				// (更新lidar定位)
				egoState.x                      = this->lastLocation(0, 3);
				egoState.y                      = this->lastLocation(1, 3);
				egoState.z                      = this->lastLocation(2, 3);
				Eigen::Matrix3f R   = this->lastLocation.block<3, 3>(0, 0);
				Eigen::Vector3f eulerAngles = rotationMatrixToEulerAnglesZYX(R);
				egoState.yaw = eulerAngles(0);
				historyYaw = egoState.yaw;
				// 对轨迹进行滤波
				ekfFilter.run(egoState);

				if (min_distance > leave_distance_threshold_) {
					reloc_state_ = RelocState::IDLE;
					current_reloc_point_index_ = -1;
					std::cout << "[重定位] 已离开重定位区域，返回IDLE状态" << std::endl;
				}
				break;
	        }
	    }
		// ==================== ABC重定位功能 - 状态检查结束 ====================

	}
	sendLocation();
	return;
}

bool LidarSlam::calInitPosture(const pcl::PointCloud<pcl::PointXYZ>::Ptr& currentScanPC)
{
	lossLocNum = 0;
	std::cout<<"AN算法初始化模式"<<std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	bool isInitSuccess = an.get_init_pose(currentScanPC,
					 scan_cloud_aligned,
					 this->lastLocation);
	if(!isInitSuccess)	return isInitSuccess;

	// 标记车辆已被初始化的状态
	this->initLocation = false;
	// 初始化车辆状态（x、y、z、yaw） 与 运动模型控制量(v、yawRate)
	egoState.x                  = this->lastLocation(0, 3);
	egoState.y                  = this->lastLocation(1, 3);
	egoState.z                  = this->lastLocation(2, 3);

	Eigen::Matrix3f R   = this->lastLocation.block<3, 3>(0, 0);
	Eigen::Vector3f eulerAngles = rotationMatrixToEulerAnglesZYX(R);
	egoState.yaw                = eulerAngles(0);
	historyYaw = eulerAngles(0);
	return isInitSuccess;
}

void LidarSlam::sendLocation()
{
	// 往下游节点发送定位结果
	timeval tv;
	gettimeofday(&tv, nullptr);
	const uint32_t usScaler      = 1000U;
	out->header.timestamp.sec    = tv.tv_sec;
	out->header.timestamp.nsec   = tv.tv_usec * usScaler;
	// 1端坐标信息
	out->pose.pose.position.x    = (egoState.init) ? ekfFilter.x_post_(0,0) : egoState.x;//
	out->pose.pose.position.y    = (egoState.init) ? ekfFilter.x_post_(1,0) : egoState.y;//
	out->pose.pose.position.z    = this->lastLocation(2, 3);
	this->xyzyawFile <<	out->pose.pose.position.x << ", " << out->pose.pose.position.y << ", " << out->pose.pose.position.z << ", " << score << std::endl;
	// 1端航向角
	out->pose.pose.orientation.z = historyYaw; //
	// rfid的ID值和强度值在子线程进行获取，不影响定位算法运行速度
    // rfid 信息
	out->pose.pose.orientation.x = (float)this->id;// rfid的ID号
	out->pose.pose.orientation.y = (float)this->intensity;// rfid的强度值
	// 传感器正常
	if (!node.IsStop())
	{
		if (node.SendLidarSlam(out) != HAF_SUCCESS)
		{
			std::cout << "SendLidarSlam error" << std::endl;
		}
	}

	if(isVision)
	{
		GPSDATA gpsData;
		egoState.x = out->pose.pose.position.x;
		egoState.y = out->pose.pose.position.y;
		// 可视化匹配结果
		mvizView(alignedPC, egoState, gpsData);
	}
}

void LidarSlam::mvizView(pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPC, vehicleState egoState, GPSDATA gpsData)
{
/*********************************************************************************
*      发布 配准后的 点云数据 topic: alignedCloud    frame(坐标系):map         *
**********************************************************************************/
    bool isPublish;
    mdc::visual::PointCloud<mdc::visual::PointXYZRGB> data;
    data.header.frameId = "map";// ---------点云坐标系
    const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
    uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000UL;
    data.isDense = false;
    data.header.stamp = mdc::visual::Times { sec, nsec };
    for (size_t i = 0; i < alignedPC->points.size(); i++)
    {
        mdc::visual::PointXYZRGB ptMviz(alignedPC->points[i].x,//x
                                        alignedPC->points[i].y,//y
                                        alignedPC->points[i].z,//z
                                        0, 0, 255);//  b g r
        data.points.push_back(ptMviz);
    }
    isPublish = this->pcPointXYZIRPub.Publish(data);

/*********************************************************************************
*           发布 实时定位Posture  topic: slamPosture    frame(坐标系):map                 *
**********************************************************************************/
    mdc::visual::PoseStamped poseTimestamp;
    poseTimestamp.header.frameId = "map";// ---------轨迹坐标系
    poseTimestamp.header.stamp = mdc::visual::Times { sec, nsec };
    poseTimestamp.pose.position = mdc::visual::Point(this->lastLocation(0, 3), this->lastLocation(1, 3), 0);
//    poseTimestamp.pose.position = mdc::visual::Point(an.anchor_boxes[13].center_x, an.anchor_boxes[13].center_y, -2.32445);

	Eigen::AngleAxisd rollAngle(0,   Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(historyYaw,     Eigen::Vector3d::UnitZ());
//	Eigen::AngleAxisd yawAngle(an.anchor_boxes[13].point_count,     Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
	poseTimestamp.pose.orientation = mdc::visual::Quaternion(q.x(), q.y(), q.z(), q.w());
	isPublish = this->slamPosturePub.Publish(poseTimestamp);

/*********************************************************************************
*           发布 实时车尾Posture  topic: tailPosture    frame(坐标系):map                 *
**********************************************************************************/
	mdc::visual::PoseStamped tailposeTimestamp;
	tailposeTimestamp.header.frameId = "map";// ---------轨迹坐标系
	tailposeTimestamp.header.stamp = mdc::visual::Times { sec, nsec };
	tailposeTimestamp.pose.position = mdc::visual::Point(this->lastLocation(0, 3), this->lastLocation(1, 3), 0);
	Eigen::Vector3d offset_body(-10.755, 0.0, 0.0);   // 车体坐标系下车尾相对车头
	Eigen::Matrix3d R = q.toRotationMatrix();    // 车头旋转矩阵（map <- body）

	Eigen::Vector3d head_pos(this->lastLocation(0, 3), this->lastLocation(1, 3), 0);
	Eigen::Vector3d tail_pos = head_pos + R * offset_body;

	tailposeTimestamp.pose.position = mdc::visual::Point(
	    tail_pos.x(), tail_pos.y(), tail_pos.z()
	);
	tailposeTimestamp.pose.orientation = mdc::visual::Quaternion(q.x(), q.y(), q.z(), q.w());
	isPublish = this->tailPosturePub.Publish(tailposeTimestamp);

/*********************************************************************************
*           发布 历史轨迹  topic: lidarTrajectory    frame(坐标系):map                 *
**********************************************************************************/
	lidarHistoryTrajectory.header.frameId = "map";// ---------轨迹坐标系
	lidarHistoryTrajectory.isDense = false;
	lidarHistoryTrajectory.header.stamp = mdc::visual::Times { sec, nsec };
	lidarHistoryTrajectory.points.clear();
	for (size_t i = 0; i < currentBottomPC->points.size(); i++)
	{
		mdc::visual::PointXYZRGB ptMviz(currentBottomPC->points[i].x,//x
										currentBottomPC->points[i].y,//y
										currentBottomPC->points[i].z,//z
														0, 255, 255);//  b g r
		lidarHistoryTrajectory.points.push_back(ptMviz);
	}
	isPublish = this->lidarTrajectPub.Publish(lidarHistoryTrajectory);

/*********************************************************************************
*           发布 GNSS轨迹  topic: gnssTrajectory    frame(坐标系):map             *
**********************************************************************************/
    gnssHistoryTrajectory.header.frameId = "map";// ---------轨迹坐标系
    gnssHistoryTrajectory.isDense = false;
    gnssHistoryTrajectory.header.stamp = mdc::visual::Times { sec, nsec };
    gnssHistoryTrajectory.points.clear();
	for (int i = 0; i < an.anchor_boxes.size(); ++i)
	{
		if(!an.anchor_boxes[i].is_valid)
			continue;

		mdc::visual::PointXYZRGB onePt3(an.anchor_boxes[i].center_x,//x
										an.anchor_boxes[i].center_y,//y
									        0,//z
									        0, 255, 0);//  b g r
		gnssHistoryTrajectory.points.push_back(onePt3);
	}
	isPublish = this->gnssTrajectPub.Publish(gnssHistoryTrajectory);
    
/*********************************************************************************
*            发布 点云地图  topic: mapCloud    frame(坐标系):map                  *
**********************************************************************************/
	if(this->sum == 10)
	{
	    mdc::visual::PointCloud<mdc::visual::PointXYZRGB> data2;
	    data2.header.frameId = "map";// ---------点云坐标系
	    data2.isDense = false;
	    data2.header.stamp = mdc::visual::Times { sec, nsec };
	    // map
	    for (size_t i = 0; i < this->mapCloud->points.size(); i++)
	    {
		    mdc::visual::PointXYZRGB ptMviz(this->mapCloud->points[i].x,//x
										    this->mapCloud->points[i].y,//y
										    this->mapCloud->points[i].z,//z
										    255, 255, 255);//  b g r
		    data2.points.push_back(ptMviz);
	    }
	    isPublish = this->mapPub.Publish(data2);
	    this->sum = 0;

	    /*********************************************************************************
	    *           发布 imu轨迹  topic: imuTrajectory    frame(坐标系):map               *
	    **********************************************************************************/
		imuHistoryTrajectory.header.frameId = "map";// ---------轨迹坐标系
		imuHistoryTrajectory.isDense = false;
		imuHistoryTrajectory.header.stamp = mdc::visual::Times { sec, nsec };

		imuHistoryTrajectory.points.clear();
		for (size_t i = 0; i < mapBottomPC->points.size(); i++)
		{
			mdc::visual::PointXYZRGB ptMviz(mapBottomPC->points[i].x,//x
											mapBottomPC->points[i].y,//y
											mapBottomPC->points[i].z,//z
											255, 0, 0);//  b g r
			imuHistoryTrajectory.points.push_back(ptMviz);
		}
		isPublish = this->imuTrajectPub.Publish(imuHistoryTrajectory);
	}
	this->sum++;
    return;
}

// 获取GPS状态函数
bool LidarSlam::getGpsStatus(uint16_t status)
{
    if( (status == 0x01) || (status == 0x02) || (status == 0x03) || (status == 0x04) )
    {
        return true;
    }
    else
    {
        return false;
    }
}
void LidarSlam::getOnlineX_Y_Yaw(std::string path)
{
	std::ifstream file_read(path);
	std::string line;
	while(file_read)
	{
		if(getline(file_read,line)) //从文件file_path中获取以'\n'为结尾的字符串,存入line中,然后文件指针自动下移至下一行
		{
			std::vector<std::string> str_vec_ptr = split(line);
			if(str_vec_ptr.empty()) continue;
			this->olPts.x.push_back(std::stod(str_vec_ptr[0]));
			this->olPts.y.push_back(std::stod(str_vec_ptr[1]));
			this->olPts.yaw.push_back(std::stod(str_vec_ptr[2]));
		}
	}
}
float LidarSlam::normalizeYaw(float yaw)
{
	float PI = 3.14159265358979323846f;
	yaw = std::fmod(yaw + PI, 2 * PI); // 将角度映射到 [0, 2*PI) 范围内
	if (yaw < 0)
	{
		yaw += 2 * PI; // 确保范围在 [0, 2*PI)
	}
	return yaw - PI; // 映射到 [-PI, PI]
}
void LidarSlam::threadGetRfid()
{
	while(!node.IsStop())
	{
		this->rfid.getID(this->id,this->intensity);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 1毫秒
	}
}
void LidarSlam::importMap()//导入全局地图
{
	this->mapCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    std::fstream input(this->mapPath.c_str(), ios::in | ios::binary);
    // 检查文件是否成功打开
    std::cout << "正在检查地图路径 ......";
    if (input.is_open())
    {
        std::cout << "[ 地图路径输入正确 ]\n" << std::endl;
    }
    else
    {
        std::cout << "[ 地图路径输入错误, 请确认地图文件地址是否正确 !!! ]\n" << std::endl;
        this->node.Stop();//关闭程序
        return;
    }
    std::cout << "正在导入全局地图 ......";
    for(int i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZ PointXYZ;
        input.read((char *)&PointXYZ.x, 3 * sizeof(float));
//        if(PointXYZ.z < -2.7){
//        	mapBottomPC->push_back(PointXYZ);
//        	continue;
//        }
        this->mapCloud->points.push_back(PointXYZ);
    }
    std::cout << "地图导入成功" << std::endl;

    //
    an.init(this->mapCloud);
    std::cout << "地图已经导入AN算法" << std::endl;

    //对地图进行下采样
    this->voxelgrid2.setInputCloud((this->mapCloud));//设置输入点云
    this->voxelgrid2.filter(*(this->mapCloud));      //设置输出点云
    this->ndt_omp->setInputTarget((this->mapCloud));//设置地图
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LidarSlam::align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud_ ,float & score) //返回转化后的点云
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *scan_cloud = *scan_cloud_;
    voxelgrid.setInputCloud(scan_cloud);//对当前帧进行下采样
    voxelgrid.filter(*scan_cloud);
    ndt_omp->setInputSource(scan_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    // 匹配算法数值初始化
    if(initLocation)
    {
        initLocation = false;
        Eigen::Translation3f init_translation(this->initX, this->initY, this->initZ);
        Eigen::AngleAxisf init_rotation_x(this->initThetaX, Eigen::Vector3f::UnitX());//roll
        Eigen::AngleAxisf init_rotation_y(this->initThetaY, Eigen::Vector3f::UnitY());//pitch
        Eigen::AngleAxisf init_rotation_z(this->initThetaZ, Eigen::Vector3f::UnitZ());//yaw
        Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
        ndt_omp->align(*aligned, init_guess);
        score  = ndt_omp->getFitnessScore ();
        this->lastLocation = ndt_omp->getFinalTransformation();//保存当前时刻的位姿
        return aligned;
    }
    else
    {
        ndt_omp->align(*aligned, this->lastLocation);//以上一时刻的位姿为初值
        score  = ndt_omp->getFitnessScore ();
        this->lastLocation = ndt_omp->getFinalTransformation();//保存当前时刻的位姿
        return aligned;
    }
}
std::vector<std::string> LidarSlam::split(const std::string &readData)
{
   std::vector<std::string> str_vec_ptr;
   std::string token;
   std::stringstream ss(readData);  //stringstream::str将流中数据转换成string字符串
	// 如果输入数据为空，则直接返回空的vector
	if (readData.empty()) {
		return str_vec_ptr;
	}
   while (getline(ss, token, ',')){//从字符串中ss读取字符，以逗号为结束符，将读到的数据存入token变量;直到读取完一整条nmea数据
		 str_vec_ptr.push_back(token);
   }
   return str_vec_ptr;
}

// ==================== ABC重定位功能实现 - 开始 ====================

/**
 * 初始化重定位系统
 * 功能：初始化ABC三个重定位点坐标、参数和UDP通信
 */
void LidarSlam::initRelocSystem()
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "初始化ABC重定位系统..." << std::endl;
    std::cout << "========================================" << std::endl;

    // 初始化状态变量
    reloc_state_ = RelocState::IDLE;
    current_reloc_point_index_ = -1;
    udp_socket_ = -1;
    udp_running_ = false;

    reloc_distance_threshold_ = 0.50;  // 20cm，触发重定位的距离阈值
    leave_distance_threshold_ = 0.80;   // 60cm，判断离开的距离阈值
    udp_server_port_ = 8888;            // UDP接收端口
    udp_client_ip_ = "192.168.30.42";   // UDP发送目标IP
    udp_client_port_ = 8889;            // UDP发送目标端口

    // 硬编码ABC三个重定位点坐标（单位：米）
    // 请根据实际情况修改这些坐标值
    reloc_points_.clear();
//    reloc_points_.emplace_back(300,300, "A");   // 点A坐标
//    reloc_points_.emplace_back(300,300, "B");   // 点B坐标
//    reloc_points_.emplace_back(300,300, "C");   // 点C坐标
    reloc_points_.emplace_back(24.368005,-1.228467, "A");   // 点A坐标
	reloc_points_.emplace_back(20.548017,11.650597, "B");   // 点B坐标
	reloc_points_.emplace_back(28.799568,19.896493, "C");   // 点C坐标

    std::cout << "重定位参数配置：" << std::endl;
    std::cout << "  触发距离阈值: " << reloc_distance_threshold_ << " 米" << std::endl;
    std::cout << "  离开距离阈值: " << leave_distance_threshold_ << " 米" << std::endl;
    std::cout << "  UDP接收端口: " << udp_server_port_ << std::endl;
    std::cout << "  UDP发送目标: " << udp_client_ip_ << ":" << udp_client_port_ << std::endl;
    std::cout << "\n重定位点坐标：" << std::endl;
    std::cout << "  点A: (" << reloc_points_[0].x << ", " << reloc_points_[0].y << ")" << std::endl;
    std::cout << "  点B: (" << reloc_points_[1].x << ", " << reloc_points_[1].y << ")" << std::endl;
    std::cout << "  点C: (" << reloc_points_[2].x << ", " << reloc_points_[2].y << ")" << std::endl;

    // 设置UDP通信
    setupUDP();

    std::cout << "ABC重定位系统初始化完成！" << std::endl;
    std::cout << "========================================\n" << std::endl;
}

/**
 * 设置UDP通信
 * 功能：创建UDP socket，绑定端口，启动接收线程
 */
void LidarSlam::setupUDP()
{
    // 创建UDP socket
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        std::cerr << "错误: 创建UDP socket失败！" << std::endl;
        return;
    }

    // 设置服务器地址（接收端）
    memset(&udp_server_addr_, 0, sizeof(udp_server_addr_));
    udp_server_addr_.sin_family = AF_INET;
    udp_server_addr_.sin_addr.s_addr = INADDR_ANY;
    udp_server_addr_.sin_port = htons(udp_server_port_);

    // 绑定socket
    if (::bind(udp_socket_, (struct sockaddr*)&udp_server_addr_, sizeof(udp_server_addr_)) < 0) {
        std::cerr << "错误: 绑定UDP端口 " << udp_server_port_ << " 失败！" << std::endl;
        close(udp_socket_);
        udp_socket_ = -1;
        return;
    }

    // 设置客户端地址（发送端）
    memset(&udp_client_addr_, 0, sizeof(udp_client_addr_));
    udp_client_addr_.sin_family = AF_INET;
    udp_client_addr_.sin_port = htons(udp_client_port_);
    inet_pton(AF_INET, udp_client_ip_.c_str(), &udp_client_addr_.sin_addr);

    // 设置非阻塞模式
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms超时
    setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 启动UDP接收线程
    udp_running_ = true;
    udp_receive_thread_ = std::thread(&LidarSlam::udpReceiveThread, this);

    std::cout << "UDP通信设置完成，监听端口: " << udp_server_port_ << std::endl;
}

/**
 * UDP接收线程
 * 功能：持续监听UDP端口，接收1字节触发信号
 */
void LidarSlam::udpReceiveThread(){
    uint8_t recvBuffer[1];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (udp_running_) {
        int recv_len = recvfrom(udp_socket_, recvBuffer, sizeof(recvBuffer), 0,
                               (struct sockaddr*)&sender_addr, &sender_len);

        if (recv_len > 0) {
            // 检查是否在等待触发状态
			std::lock_guard<std::mutex> lock(reloc_state_mutex_);
			trainDirection = recvBuffer[0];
			std::cout << "[UDP] 收到方向信号: " << static_cast<int>(trainDirection) << " ，当前转盘状态：" << ((trainDirection == 0) ? "锁定" : "解锁") << std::endl;
        }

        // 短暂休眠避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * 关闭UDP通信
 * 功能：停止UDP线程，关闭socket
 */
void LidarSlam::closeUDP()
{
    udp_running_ = false;
    if (udp_receive_thread_.joinable()) {
        udp_receive_thread_.join();
    }
    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    }
    std::cout << "[UDP] UDP通信已关闭" << std::endl;
}

/**
 * 计算两点之间的欧氏距离
 * 功能：计算x,y平面上两点的距离（norm）
 */
double LidarSlam::calculateDistance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * 查找最近的重定位点
 * 功能：遍历ABC三个点，找出距离当前位置最近的点
 * 返回：最近点的索引，min_distance返回最小距离值
 */
int LidarSlam::findNearestRelocPoint(double& min_distance)
{
    // 从lastLocation矩阵中获取当前位置
    double current_x = this->lastLocation(0, 3);
    double current_y = this->lastLocation(1, 3);

    int nearest_index = -1;
    min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < reloc_points_.size(); ++i) {
        double dist = calculateDistance(current_x, current_y,
                                       reloc_points_[i].x, reloc_points_[i].y);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_index = i;
        }
    }

    return nearest_index;
}


