/*
 * Description:  Define multisensor_fusion.cpp
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */
#include<string>
#include <sys/time.h>
#include "core/logger.h"
#include "object/object.h"
#include<yaml-cpp/yaml.h>
#include <cstring>
#include <vector>
#include "Logger.h"
#include "multisensor_fusion.h"
#include <memory>

using namespace Adsfi;
Logger &logger = Logger::getInstance();

/**
 * 把工位点文本, [x1,y1],[x2,y2]转成集合方便获取
 * @param workList
 * @return pairs
 */
std::vector <Pair> parseWorkList(const std::string &workList) {
    std::vector <Pair> pairs;
    std::string cleanedWorkList = workList;

    cleanedWorkList.erase(std::remove(cleanedWorkList.begin(), cleanedWorkList.end(), '['), cleanedWorkList.end());
    cleanedWorkList.erase(std::remove(cleanedWorkList.begin(), cleanedWorkList.end(), ']'), cleanedWorkList.end());

    std::stringstream ss(cleanedWorkList);
    std::string pairStr;

    // 逐个解析每对值
    while (std::getline(ss, pairStr, ',')) {
        Pair p;
        try {
			p.x = std::stof(pairStr);
			if (std::getline(ss, pairStr, ',')) {
				p.y = std::stof(pairStr);
				pairs.push_back(p);
			}
		} catch (const std::exception& e) {
			logger.log(Logger::ERROR, "解析工位点失败: " + std::string(e.what()));
		}
    }
    return pairs;
}

/**
 * 把16进制字符串转字节数组
 * @param hexStr
 * @return
 */
std::vector <uint8_t> hexStringToBytes(const std::string &hexStr) {
    std::vector <uint8_t> bytes;
    std::istringstream hexStream(hexStr);
    std::string byteStr;
    while (hexStream >> std::setw(2) >> std::hex >> byteStr) {
        bytes.push_back(static_cast<uint8_t>(std::stoi(byteStr, nullptr, 16)));
        if (hexStream.peek() == ' ') {
            hexStream.ignore(); // 忽略空格
        }
    }
    return bytes;
}

// ==================== MultisensorFusion的构造函数 ====================
MultisensorFusion::MultisensorFusion(std::string configFile) : node(configFile) {
    try {
    	int a = 0;
        YAML::Node config = YAML::LoadFile(configFile);
        // 获取工位点信息
        workList_ = config["workList"].as<std::string>();
        workPairs_ = parseWorkList(workList_);

        // 1.往车控发送数据
        int destPort_ZC = config["ClientSock_ZC"]["destPort"].as<int>();
        std::string destIP_ZC = config["ClientSock_ZC"]["destIP"].as<std::string>();
//        std::cout << "destIP_ZC = " << destIP_ZC << ", destPort_ZC = " << destPort_ZC << std::endl;
        // -------------- 只是开启一个子线程去设置通讯 车控端的ip 和 车控端的端口
        pool.push_back(std::thread(&udpCommHelp::initClientSock_ZC, & this->udpHelp, destPort_ZC, destIP_ZC));

        // 2.获取车控发过来的数据
        int serverPort_ZC = config["serverSock_ZC"]["destPort"].as<int>();
        std::string serverDestIP_ZC = config["serverSock_ZC"]["destIP"].as<std::string>();
        // -------------- 只是开启一个子线程去设置 本机通讯ip(没用到) 和 本机端口
        pool.push_back(std::thread(&udpCommHelp::initServerZc, &this->udpHelp, serverDestIP_ZC, serverPort_ZC));
        // 3.获取感知结果，获取定位结果，然后把结构化数据发送到车控
        pool.push_back(std::thread(&MultisensorFusion::getFusionSend, this));

    } catch (const YAML::Exception &yamlEx) {
        logger.logException(std::runtime_error(yamlEx.what()));
    } catch (const std::exception &e) {
        logger.logException(std::runtime_error(e.what()));
    } catch (...) {
        logger.log(Logger::ERROR, "初始化位置异常");
    }

    // 初始化状态变量
	udp_socket_ = -1;
	udp_running_ = false;
	udp_server_port_ = 8889;            // UDP接收端口
	udp_client_ip_ = "192.168.30.42";   // UDP发送目标IP
	udp_client_port_ = 8888;            // UDP发送目标端口

//	std::cout << "  UDP接收端口: " << udp_server_port_ << std::endl;
//	std::cout << "  UDP发送目标: " << udp_client_ip_ << ":" << udp_client_port_ << std::endl;

	// 设置UDP通信
	setupUDP();

    // 打开日志文件
    sendFile.open("sendData.txt");
    slamFile.open("slamData.txt");
    recvFile.open("recvData.txt");
    if (!sendFile.is_open() || !slamFile.is_open() || !recvFile.is_open()) {
		logger.log(Logger::ERROR, "无法打开日志文件");
	}

//    std::cout << "MultisensorFusion 构造函数执行完毕..." << std::endl;
}

MultisensorFusion::~MultisensorFusion() {
//	std::cout << "开始析构" << std::endl;
    isThreadRunning = false;
    udp_running_ = false;

    // 关闭socket以中断阻塞的recvfrom，关闭UDP线程
	if (udp_socket_ >= 0) {
		shutdown(udp_socket_, SHUT_RDWR);
		close(udp_socket_);
		udp_socket_ = -1;
	}

	if (udp_receive_thread_.joinable()) {
		udp_receive_thread_.join();
	}

	// 关闭感知线程、定位线程、中车数据接收线程
    for (auto &it: pool) {
        if (it.joinable()) {
            it.join();
        }
    }

    // 关闭数据记录句柄
    if(sendFile.is_open()){
    	sendFile.close();
    }
    if(slamFile.is_open()){
    	slamFile.close();
	}
    if(recvFile.is_open()){
		recvFile.close();
	}
//    std::cout << "析构完成" << std::endl;
}

/**
 * 设置UDP通信
 * 功能：创建UDP socket，绑定端口，启动接收线程
 */
void MultisensorFusion::setupUDP()
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
    udp_receive_thread_ = std::thread(&MultisensorFusion::udpReceiveTunableSignalThread, this);

//    std::cout << "UDP通信设置完成，监听端口: " << udp_server_port_ << std::endl;
}

void MultisensorFusion::udpReceiveTunableSignalThread()
{
    uint8_t buffer[1];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);

    while (udp_running_) {
        int recv_len = recvfrom(udp_socket_, buffer, 1, 0,
                               (struct sockaddr*)&sender_addr, &sender_len);

        if (recv_len > 0) {
//            std::cout << "[UDP] slam的转盘确认信号: 0x" << std::hex << (int)buffer[0] << std::dec << std::endl;
            // 线程安全更新
			updateTunableStatusConfirm(buffer[0]);
        }

        // 短暂休眠避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

/**
 * 发送UDP完成信号
 * 功能：重定位完成后向外部系统发送0xFF完成标志
 */
void MultisensorFusion::sendUDPTunSpeed(uint8_t diRection)
{
    if (udp_socket_ < 0) {
        std::cerr << "[UDP] 警告: UDP socket未初始化，无法发送完成信号" << std::endl;
        return;
    }

    uint8_t sendBuff[1];
    sendBuff[0] = diRection;
    int sent_len = sendto(udp_socket_, sendBuff, sizeof(sendBuff), 0,
                         (struct sockaddr*)&udp_client_addr_,
                         sizeof(udp_client_addr_));

//    if (sent_len < 0) {
//        std::cerr << "[UDP] 错误: 发送完成信号失败！" << std::endl;
//    } else {
//        std::cout << "[UDP] 成功发送方向信号 tbSignal " << static_cast<int>(diRection) << " 到 "
//                  << udp_client_ip_ << ":" << udp_client_port_ << std::endl;
//    }
}

/**
 * 独立线程，去获取感知数据和定位数据，同时给车控发送结构数据
 */
void MultisensorFusion::getFusionSend() {
    //这里延迟,是为了等待node节点初始化完成,因为是异步的
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (isThreadRunning) {
        try {
            getPreFusion();		// 获取感知结果
            RecvLocation();		// 获取定位结果
			Fusions fusionsCopy = getFusionsCopy();		// 获取数据副本用于发送

            // 该if...else if是为了避免自动模式时，slam.direction的值为1、2，所以发送给车控之前要转换成 170 （朝2端前进）和 85 （朝1端前进）
			uint8_t direction = fusionsCopy.slam.direction;
			if (direction == FusionConstants::DIRECTION_TO_END2) {
				fusionsCopy.slam.direction = FusionConstants::DIRECTION_MANUAL_END2;
			} else if (direction == FusionConstants::DIRECTION_TO_END1) {
				fusionsCopy.slam.direction = FusionConstants::DIRECTION_MANUAL_END1;
			}
			// 写入日志文件
			std::string hexData = fusionsCopy.toHexString();
			sendFile << hexData << std::endl;
			slamFile << static_cast<int>(fusionsCopy.slam.hb) << ", "
					 << fusionsCopy.slam.headX << ", "
					 << fusionsCopy.slam.headY << ", "
					 << fusionsCopy.slam.headZ << ", "
					 << fusionsCopy.slam.headT << std::endl;
			// 发送数据到车控
			udpHelp.sendMsg(hexStringToBytes(hexData));

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        catch (std::exception &e) {
            std::cerr << "发中车异常: " << e.what() << std::endl;
        }
    }
}

/**
 * 进程管理
 */
void MultisensorFusion::Process() {
    // 开启线程去获取车控端的结构化数据
    pool.push_back(std::thread(&MultisensorFusion::getZcData, this));
//    // 开启线程，在内部广播车辆的行驶方向
    pool.push_back(std::thread(&MultisensorFusion::mdcBroadcastData, this));
}

//---------------------------------------Process函数去调用一下两个函数
/**
 * 定时接收来自车控的数据
 */
void MultisensorFusion::getZcData() {
	while (isThreadRunning) {
		unsigned char buf[3080];
		memset(buf, 0, sizeof(buf));
		// -------接收来自车控的结构化数据数据
		int recv_len = udpHelp.recvData_clientSock_zc(buf, sizeof(buf));
		if (recv_len > 0)
		{
			std::vector<unsigned char> buf_vec(buf, buf + recv_len);
			// 写入接收日志
			for (int i = 0; i < recv_len; i++){
				recvFile << static_cast<int>(buf_vec[i]) << ", ";
			}
			recvFile << std::endl;

			// ===== 关键修改：一次性完成所有ZcData更新 =====
			{
				std::lock_guard<std::mutex> lock(fusions_mutex_);
				ZcData& zc = fusions_.zcData;

				// 1. 解析原始数据
				zc.setData(buf_vec);

				// 2. 立即筛选目标工位点（在同一个锁内）
				int number = static_cast<int>(zc.Station_Num);
				if (number > 0){
					for (int i = 1; i <= number; i++) {
						uint8_t status = zc.All_Station[i-1].Station_Status;
						if ((status & (0x01 << 4)) == 16) {
							continue;
						}
						if ((status & (0x01 << 2)) == 4 || (status & (0x01 << 5)) == 0) {
							zc.Station = zc.All_Station[i-1].Station_ID;
							break;
						}
					}
				}

				if (zc.Station != FusionConstants::INVALID_STATION && (zc.Station - 1) < workPairs_.size()) {
					fusions_.targetPair = workPairs_[zc.Station - 1];
				}

				// 发送UDP
				sendUDPTunSpeed(zc.Direction);
			}
			// ===== 锁已释放 =====
		}
		//休眠 20 毫秒
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

/**
 * 发送组播数据，用于在 MDC 内部广播方向
 */
void MultisensorFusion::mdcBroadcastData() {
    // 配置组播地址、端口和接口名称
    std::string multicastAddress = "239.0.0.1";  // 替换为需要的组播地址
    uint16_t port = 8848;  // 替换为需要的端口
    std::string interfaceName = "eth0.12";  // 替换为实际的网卡名称
    UdpMulticastSender sender(multicastAddress, port, interfaceName);
    while (isThreadRunning) {
    	// 获取当前方向
		int currentDir = getDirection();

		// 方向转换：170->2, 85->1
		if (currentDir == FusionConstants::DIRECTION_MANUAL_END2) {
			currentDir = FusionConstants::DIRECTION_TO_END2;
		} else if (currentDir == FusionConstants::DIRECTION_MANUAL_END1) {
			currentDir = FusionConstants::DIRECTION_TO_END1;
		}

        uint8_t preDirection = static_cast<uint8_t>(currentDir & 0xFF);
        // 获取ZcData副本
		ZcData zcCopy = getZcDataCopy();
		uint8_t obstacleStatus = getObstacleStatus();

        if(zcCopy.Station_Num > 0){
        	sender.sendMessage(preDirection, zcCopy.All_Station.front().Station_ID, zcCopy.All_Station[zcCopy.Station_Num-1].Station_ID, obstacleStatus);
        }
        else {
        	sender.sendMessage(0, 0, 0, 0);
		}
        // 等待 100 毫秒
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

//---------------------------------------构造函数去调用一下两个函数 getPreFusion（），RecvLocation（）
/*
 * 获取感知数据
 */
void MultisensorFusion::getPreFusion() {
    HafFusionOutArray <float32_t> data;
    // 从节点获取融合输出对象数据
    auto fusionOutObjectsDataPtrVec = node.GetNFusionOutObjectData(1);
    if (!fusionOutObjectsDataPtrVec.empty()) {
        // 提取数据到data
        data = *fusionOutObjectsDataPtrVec.front();
    }

    // 提取数据帧ID 和 异常标志
    std::string idStr = data.frameID;
    uint32_t isEx = data.seq;
    // 更新传感器故障状态
	updateSensorFault([&idStr, isEx](SensorFault& sf) {
		if (idStr == "1") {
			sf.a4 = (isEx == 1);
		} else if (idStr == "2") {
			sf.b2 = (isEx == 1);
		}
	});

	// 更新障碍物数据
	if (data.fusionOut.size() <= 0) {
		updateObstacle([](Obstacle& obs) {
			obs.reset();
		});
	} else {
		for (const auto &iter : data.fusionOut) {
			updateObstacle([&iter](Obstacle& obs) {
				obs.Obstacle_Type = iter.cls;
				float32_t x = iter.rect.center.x;
				float32_t y = iter.rect.center.y;
				float32_t z = iter.rect.center.z;
				float32_t distance = std::sqrt(x * x + y * y + z * z);
				obs.Obstacle_Distance = static_cast<int>(distance * 100);
			});
		}
	}
}

/**
 * 接收来自定位的数据
 */
void MultisensorFusion::RecvLocation() {
    try {
        // 获取定位数据
        HafLocation data;
        auto curLocationVec = node.GetNLocationData(1);

        if(curLocationVec.empty() || curLocationVec.front() == nullptr){
        	HAF_LOG_ERROR << "Failed to Get First Location Data";
        	return;
        }
        data = *curLocationVec.front();

        // 提取定位数据
		uint32_t seq = data.header.seq;
		float headX = data.pose.pose.position.x;
		float headY = data.pose.pose.position.y;
		float headZ = data.pose.pose.position.z;
		float headT = data.pose.pose.orientation.z;
		float64_t rfidId = data.pose.pose.orientation.x;
		float64_t rfidRssi = data.pose.pose.orientation.y;

		// 计算车尾位置
		Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yawAngle(headT, Eigen::Vector3d::UnitZ());
		Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
		Eigen::Matrix3d R = q.toRotationMatrix();
		Eigen::Vector3d offset_body(-FusionConstants::TRAIN_LENGTH, 0.0, 0.0);
		Eigen::Vector3d head_pos(headX, headY, headZ);
		Eigen::Vector3d tail_pos = head_pos + R * offset_body;

		float64_t tail_x = tail_pos(0);
		float64_t tail_y = tail_pos(1);
		float64_t tail_z = tail_pos(2);
		float64_t tail_t = getAnotherYaw(headT);

		// 获取当前状态（需要加锁读取）
		ZcData zcCopy = getZcDataCopy();
		Pair targetPairCopy = getTargetPair();
		int zcDataStation = static_cast<int>(zcCopy.Station);

		float dis_1toStation = 0.0f;
		float dotProduct = 0.0f;
		int newDirection = 0;
		int newHistoryStation = 0;
		int newHistoryDirection = 0;
		int newHistorySymbol = 0;

		// 读取当前历史状态
		{
			std::lock_guard<std::mutex> lock(fusions_mutex_);
			newHistoryStation = fusions_.historyStation;
			newHistoryDirection = fusions_.historyDirection;
			newHistorySymbol = fusions_.historySymbol;
		}
		if (zcDataStation != FusionConstants::INVALID_STATION) {
			// 获取工位点坐标
			float stationX = targetPairCopy.x;
			float stationY = targetPairCopy.y;
			// 计算1端到目标工位点的距离
			dis_1toStation = std::sqrt((headX - stationX) * (headX - stationX) +
									   (headY - stationY) * (headY - stationY));

			// 1. 构建从1端(车头)指向2端(车尾)的矢量
			float vec1to2_X = tail_x - headX;
			float vec1to2_Y = tail_y - headY;
			// 2. 构建从1端(车头)指向工位点的矢量
			float vec1toStation_X = stationX - headX;
			float vec1toStation_Y = stationY - headY;
			dotProduct = vec1toStation_X * vec1to2_X + vec1toStation_Y * vec1to2_Y;


			// 更新了目标工位点，才会重新计算朝1端走还是朝2端走
			if (newHistoryStation != zcDataStation) {
				// 1. 更新历史工位点
				newHistoryStation = zcDataStation;
				// 2. 点积为负,朝1端方向行驶，否则朝2端
				newHistoryDirection = (dotProduct < 0) ?
									  FusionConstants::DIRECTION_TO_END1 :
									  FusionConstants::DIRECTION_TO_END2;
			}

			newDirection = newHistoryDirection;
		} else {
			newHistoryStation = zcDataStation;
			dis_1toStation = 0.0;
			dotProduct = 0.0;
			newHistoryDirection = 0;
			newDirection = 0;
		}

		// 线程安全更新所有Slam和状态数据
		{
			std::lock_guard<std::mutex> lock(fusions_mutex_);

			// 更新状态
			fusions_.historyStation = newHistoryStation;
			fusions_.historyDirection = newHistoryDirection;
			fusions_.historySymbol = newHistorySymbol;
			fusions_.direction = newDirection;

			// 更新传感器故障
			fusions_.sensorFault.gps = seq;

			// 更新Slam数据
			Slam& slam = fusions_.slam;
			slam.station_distance = static_cast<int>(dis_1toStation * 100);
			slam.direction = newDirection;
			slam.station_info = zcDataStation;
			slam.hb = fusions_.incrementHeartbeat();
			slam.version = 0;
			slam.rfid_tag_id = static_cast<int>(rfidId);
			slam.rfid_rssi = (rfidRssi >= 1) ? 1 : 0;

			slam.headX = static_cast<int>(headX * 100) + FusionConstants::POSITION_OFFSET;
			slam.headY = static_cast<int>(headY * 100) + FusionConstants::POSITION_OFFSET;
			slam.headZ = static_cast<int>(headZ * 100) + FusionConstants::POSITION_OFFSET;
			slam.headT = static_cast<int>(headT * 100) + FusionConstants::ANGLE_OFFSET;
			slam.tailX = static_cast<int>(tail_x * 100) + FusionConstants::POSITION_OFFSET;
			slam.tailY = static_cast<int>(tail_y * 100) + FusionConstants::POSITION_OFFSET;
			slam.tailZ = static_cast<int>(tail_z * 100) + FusionConstants::POSITION_OFFSET;
			slam.tailT = static_cast<int>(tail_t * 100) + FusionConstants::ANGLE_OFFSET;
		}
		// 写入日志（在锁外执行）
//		std::cout << "目标工位点：" << zcDataStation
//				  << ", 方向：" << newDirection
//				  << ", 距离：" << static_cast<int>(dis_1toStation * 100) << std::endl;
		slamFile << zcDataStation << ", " << newDirection << ", "
				 << static_cast<int>(dis_1toStation * 100) << ", "
				 << dis_1toStation << ", " << dotProduct << ", "
				 << newHistoryDirection << ", " << newHistorySymbol << ", "
				 << targetPairCopy.x << ", " << targetPairCopy.y << ", "
				 << headX << ", " << headY << ", " << headT << ", "
				 ;
    }
    catch (const std::exception &e) {
        logger.logException(std::runtime_error(e.what()));
    }
    catch (...) {
        logger.log(Logger::ERROR, "getLocation捕获到未知异常");
    }
}








