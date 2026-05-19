/* *
 * Description:  Define multisensor_fusion.h
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * */

#ifndef SAMPLE_MULTISENSOR_FUSION_H
#define SAMPLE_MULTISENSOR_FUSION_H

#include "adsf/planning_base.h"
#include "core/core.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <sstream>
#include <thread>
#include "udpCommHelp.h"
#include <ctime>
#include <sys/time.h>

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

// ==================== 常量定义 ====================
namespace FusionConstants {
    constexpr int POSITION_OFFSET = 100000;      // 位置偏移量
    constexpr int ANGLE_OFFSET = 315;            // 角度偏移量
    constexpr double TRAIN_LENGTH = 10.755;      // 车身长度(米)
    constexpr int INVALID_STATION = 99;          // 无效工位点标识
    constexpr int DIRECTION_TO_END1 = 1;         // 朝1端行驶
    constexpr int DIRECTION_TO_END2 = 2;         // 朝2端行驶
    constexpr int DIRECTION_MANUAL_END1 = 85;    // 手动模式朝1端
    constexpr int DIRECTION_MANUAL_END2 = 170;   // 手动模式朝2端
    constexpr double VELOCITY_OFFSET = 30000.0;  // 速度偏移量
}

// ==================== 工位点坐标结构 ====================
struct Pair {
    float_t x;
    float_t y;
    void setX(const float_t &x1) {
        x = x1;
    }
    void setY(const float_t &y1) {
        y = y1;
    }
};

// ==================== 工位点状态结构 ====================
struct StationStruct {
    uint8_t Station_ID = 0;     // 工位点编号
    uint8_t Station_Status = 0; // 工位点状态
};

/**
 * 雷达状态信息实体
*/
struct LidarStatusMessage {
    uint32_t messageId;               // 消息ID
    uint16_t messageType;             // 消息类型
    uint16_t length;                  // 消息长度
    uint8_t messageLidar_type;//雷达位置
    uint8_t ethPhyErrorState;         // 以太网收发器模块故障状态
    uint8_t socCtrlErrorState;        // SOC控制器模块故障状态
    uint8_t receiveModuleErrorState;  // 接收模块故障状态
    uint8_t sendModuleErrorState;     // 发射模块故障状态
    uint8_t electricMotorErrorState;  // 电机模块故障状态
    uint8_t highTemperatureErrorState;//温度过温告警状态0：温度无故障 1：温度一般故障 2：温度严重故障
    uint8_t lowTemperatureErrorState;//电压过高告警状态 0：电压无告警1：电压一般告警2：电压严重告警
    uint8_t highVoltageErrorState;//电压过高告警状态 0：电压无告警1：电压一般告警2：电压严重告警
    uint8_t lowVoltageErrorState;//电压过低告警状态0：电压无告警1：电压一般告警2：电压严重告警
};

// ==================== SLAM定位数据 ====================
class Slam {
public:
	uint8_t hb = 0;                    // 心跳
	uint8_t version = 0;               // 版本号
	uint64_t headX = 0;                // 车头x
	uint64_t headY = 0;                // 车头y
	uint64_t headZ = 0;                // 车头z
	uint64_t headT = 0;                // 车头航向角
	uint64_t tailX = 0;                // 车尾x
	uint64_t tailY = 0;                // 车尾y
	uint64_t tailZ = 0;                // 车尾z
	uint64_t tailT = 0;                // 车尾航向角
	uint8_t rfid_rssi = 0;             // RFID强度值
	uint8_t direction = 0;             // 方向
	uint16_t station_distance = 0;     // 距工位点距离
	uint8_t station_info = 0;          // 工位点信息
	uint8_t rfid_tag_id = 0;           // RFID的id
	uint8_t tunable_status_confirm = 0;// 转盘确认信息

    // 生成16进制字符串的方法
    std::string toHexString() const {
        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');

        // 心跳
        ss << std::setw(2) << static_cast<int>(hb) << " ";
        //版本号
        ss << std::setw(2) << static_cast<int>(version) << " ";
        // 车头x
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((headX >> (i * 8)) & 0xFF) << " ";
        }
        // 车头y
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((headY >> (i * 8)) & 0xFF) << " ";
        }
        // 车头z
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((headZ >> (i * 8)) & 0xFF) << " ";
        }
        // 车头t
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((headT >> (i * 8)) & 0xFF) << " ";
        }
        // 车尾x
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((tailX >> (i * 8)) & 0xFF) << " ";
        }
        // 车尾y
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((tailY >> (i * 8)) & 0xFF) << " ";
        }
        // 车尾z
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((tailZ >> (i * 8)) & 0xFF) << " ";
        }
        // 车尾t
        for (int i = 7; i >= 0; --i) {
            ss << std::setw(2) << ((tailT >> (i * 8)) & 0xFF) << " ";
        }
        // RFID强度值
        ss << std::setw(2) << static_cast<int>(rfid_rssi) << " ";
        // 方向
        ss << std::setw(2) << static_cast<int>(direction) << " ";
        // 距工位点距离
        ss << std::setw(2) << ((station_distance >> 8) & 0xFF) << " "
           << std::setw(2) << (station_distance & 0xFF) << " ";
        // 工位点信息
        ss << std::setw(2) << static_cast<int>(station_info) << " ";
        // RFID的强度
        ss << std::setw(2) << static_cast<int>(rfid_tag_id) << " ";
        // 转盘确认信息
		ss << std::setw(2) << static_cast<int>(tunable_status_confirm) << " ";

        std::string hexString = ss.str();
        std::vector <uint8_t> bytes;

        // 将16进制字符串转换成字节数组
        std::istringstream iss(hexString);
        std::string byteStr;
        while (iss >> byteStr) {
            bytes.push_back(std::stoul(byteStr, nullptr, 16));
        }
        // 补足00
        while (bytes.size() < 100) {
            bytes.push_back(0x00);
        }
        // 将字节数组转换回16进制字符串
        std::stringstream result;
        for (const auto &byte: bytes) {
            result << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << static_cast<int>(byte) << " ";
        }
        return result.str();
    }
};

// ==================== 感知障碍物数据 ====================
class Obstacle {
public:
    uint8_t Obstacle_Type = 0;			// 障碍物类型
	uint32_t Obstacle_Distance = 0;		// 距障碍物距离
	uint32_t confidence = 0;			// 目标种类置信度(1到10)
	uint32_t rect_center_x = 0;			// 目标物在图像中的中心坐标x(像素)
	uint32_t rect_center_y = 0;			// 目标物在图像中的中心坐标y(像素)
	uint32_t rect_size_x = 0;			// 目标物在图像中的宽度(像素)
	uint32_t rect_size_y = 0;			// 目标物在图像中的高度(像素)
	uint32_t abs_rect_size_x = 30000;	// 目标物在毫米波雷达坐标系下x方向位置(米)
	uint32_t abs_rect_size_y = 30000;	// 目标物在毫米波雷达坐标系下y方向位置(米)
	uint32_t velocity_x = 30000;		// 目标物在毫米波雷达坐标系下x方向速度(米/秒)
	uint32_t velocity_y = 30000;		// 目标物在毫米波雷达坐标系下y方向速度(米/秒)

	void reset() {
		Obstacle_Type = 0;
		Obstacle_Distance = 0;
		confidence = 0;
		rect_center_x = 0;
		rect_center_y = 0;
		rect_size_x = 0;
		rect_size_y = 0;
		abs_rect_size_x = 30000;
		abs_rect_size_y = 30000;
		velocity_x = 30000;
		velocity_y = 30000;
	}

    // 将所有成员变量转换为16进制字符串的方法
    std::string toHexString() const {
        std::stringstream ss;
        // 转换 uint8_t 类型的变量
        ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(Obstacle_Type) << " ";
        // 转换 uint32_t 类型的变量
        for (auto value: {Obstacle_Distance, confidence, rect_center_x, rect_center_y, rect_size_x, rect_size_y,
                          abs_rect_size_x, abs_rect_size_y, velocity_x, velocity_y}) {
            for (int i = 3; i >= 0; --i) {
                uint8_t byte = (value >> (i * 8)) & 0xFF;
                ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
            }
        }

        // 获取当前字符串
        std::string hexString = ss.str();
        // 计算当前字符串的字节数
        size_t currentBytes = hexString.length() / 3; // 每个字节占3个字符（两个16进制字符加一个空格）
        // 补充00字节，直到达到80个字节
        while (currentBytes < 80) {
            hexString += "00 ";
            currentBytes++;
        }
        return hexString;
    }
};

// ==================== 传感器故障信息 ====================
class SensorFault {
public:

    uint8_t level = 0;					// 告警等级
    bool b1 = false;					// b1雷达故障
    bool b2 = false;					// b2雷达故障
    bool a3 = false;					// a3雷达故障
    bool a4 = false;					// a4雷达故障
    bool gps = false;					// 组合惯导
    bool milli_wave_radar1 = false;		// 毫米波雷达1
    bool milli_wave_radar2 = false;		// 毫米波雷达2
    bool camera1 = false;				// 相机1
    bool camera2 = false;				// 相机2
    bool camera3 = false;				// 相机3
    bool camera4 = false;				// 相机4

    // 压缩成 16 进制字符串
    std::string toHexString() const {
        uint16_t data = 0;
        // 压缩成 16 bits
        data |= (b1 & 0x1) << 0;
        data |= (b2 & 0x1) << 1;
        data |= (a3 & 0x1) << 2;
        data |= (a4 & 0x1) << 3;
        data |= (gps & 0x1) << 4;
        data |= (milli_wave_radar1 & 0x1) << 5;
        data |= (milli_wave_radar2 & 0x1) << 6;
        data |= (camera1 & 0x1) << 7;
        data |= (camera2 & 0x1) << 8;
        data |= (camera3 & 0x1) << 9;
        data |= (camera4 & 0x1) << 10;

        std::vector <uint8_t> result(3);
        result[0] = level;                // level 单独占用一个字节
        result[1] = data & 0xFF;          // 低 8 bits (之前是 result[2])
        result[2] = (data >> 8) & 0xFF;   // 高 8 bits (之前是 result[1])

        // 将每个字节转换为 16 进制字符串
        std::stringstream ss;
        for (auto byte: result) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int) byte << " ";
        }
        return ss.str();
    }
};

// ==================== 雷达具体故障信息 ====================
class RadarFault {
public:
    bool ethernet = false;			//以太网收发器模块故障
	bool soc = false;				//soc控制器故障
	bool receiver = false;			//接收模块故障
	bool transmitter = false;		//发射模块故障
	bool motor = false;				//电机模块故障
	uint8_t temperature = 0;		//温度过温异常
	uint8_t low_temperature = 0;	//温度低温异常
	uint8_t voltage = 0;			//电压过高异常
	uint8_t low_voltage = 0;		//电压过低异常

    // 压缩成 16 进制字符串
    std::string toHexString() const {
        uint32_t data = 0;
        // 压缩bool变量，5个bool变量压缩到前5位
        data |= (ethernet & 0x1) << 4;
        data |= (soc & 0x1) << 3;
        data |= (receiver & 0x1) << 2;
        data |= (transmitter & 0x1) << 1;
        data |= (motor & 0x1);

        // 5个bool变量占1字节，第2字节开始是温度、电压等数值
        std::vector <uint8_t> result(5);
        result[0] = data & 0xFF;            // 前 8 bits，5个bool
        result[1] = temperature;            // temperature
        result[2] = low_temperature;        // low_temperature
        result[3] = voltage;                // voltage
        result[4] = low_voltage;            // low_voltage

        // 将每个字节转换为 16 进制字符串
        std::stringstream ss;
        for (auto byte: result) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int) byte << " ";
        }
        return ss.str();
    }
};


// vcu_to_mdc 的车控数据
class ZcData {
public:
	uint8_t HB = 0;										// VCU心跳
	uint8_t RED_Alarm = 0;								// 机车红色报警标志
	uint8_t Yellow_Alarm = 0;							// 机车黄色报警标志
	uint8_t Coup_Status = 0;							// 机车挂钩状态
	uint8_t Station = FusionConstants::INVALID_STATION;	// 工位点目标信息
	uint8_t Direction = 0;								// 方向
	uint16_t LocoSpd = 0;								// 机车速度
	uint32_t LocoTime = 0;								// 机车时间
	uint8_t Run_Mode = 0;								// 机车模式
	uint8_t Station_Num = 0;							// 工位点数量
	uint8_t TurntableSignal = 0;						// 转盘信号
	std::vector<StationStruct> All_Station;				// 工位点ID与状态

	ZcData() {
		All_Station.resize(20);
	}

    void setData(const std::vector<unsigned char> &data)
    {
        if(data.size() < 55)	return;

		this->HB = data[0];
		this->RED_Alarm = data[1];
		this->Yellow_Alarm = data[2];
		this->Coup_Status = data[3];
		this->Station = FusionConstants::INVALID_STATION;
		this->Direction = data[5];
		this->LocoSpd = (data[6] << 8) | data[7];
		this->LocoTime = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
		this->Run_Mode = data[12];
		this->Station_Num = data[13];
		for(int i = 1; i <= this->Station_Num; i++)
		{
			this->All_Station[i-1].Station_ID     = data[13 + 2*i - 1];
			this->All_Station[i-1].Station_Status = data[13 + 2*i];
		}
		this->TurntableSignal = data[54];
		return;
    }
};

// ==================== 融合数据类（核心类） ====================
class Fusions {
public:
	// 内部数据
	Slam slam;
	Obstacle obstacle;
	ZcData zcData;
	SensorFault sensorFault;
	RadarFault radarFault1;
	RadarFault radarFault2;
	RadarFault radarFault3;
	RadarFault radarFault4;

    // 状态变量（原全局变量）
	Pair targetPair;                                           // 目标工位点坐标
	int direction = 0;                                         // 行驶方向
	int historyStation = FusionConstants::INVALID_STATION;     // 历史工位点
	int historyDirection = 0;                                  // 历史行驶方向
	int historySymbol = 0;                                     // 历史符号
	uint8_t heartbeat = 0;                                     // 心跳计数器
	uint8_t isReceiveObstacle = 0;                             // 是否收到障碍物

	// Setter方法
	void setSlam(const Slam &s) { slam = s; }
	void setObstacle(const Obstacle &obj) { obstacle = obj; }
	void setSensorFault(const SensorFault &sen) { sensorFault = sen; }
	void setRadarFault1(const RadarFault &ra) { radarFault1 = ra; }
	void setRadarFault2(const RadarFault &ra) { radarFault2 = ra; }
	void setRadarFault3(const RadarFault &ra) { radarFault3 = ra; }
	void setRadarFault4(const RadarFault &ra) { radarFault4 = ra; }

	// Getter方法
	Slam& getSlam() { return slam; }
	Obstacle& getObstacle() { return obstacle; }
	ZcData& getZcData() { return zcData; }
	SensorFault& getSensorFault() { return sensorFault; }

    // 拼装所有 RadarFault 对象的 16 进制字符串
    std::string toHexString() const {
        std::stringstream ss;
        ss << slam.toHexString();
        ss << obstacle.toHexString();
        ss << sensorFault.toHexString();
        ss << radarFault1.toHexString();
        ss << radarFault2.toHexString();
        ss << radarFault3.toHexString();
        ss << radarFault4.toHexString();
        return ss.str();
    }

    // 增加心跳
	uint8_t incrementHeartbeat() {
		return heartbeat++;
	}

	// 更新障碍物接收状态
	void updateObstacleStatus() {
		isReceiveObstacle = (obstacle.Obstacle_Type != 0) ? 1 : 0;
	}
};


// ==================== 主类 ====================
class MultisensorFusion {
public:
	explicit MultisensorFusion(std::string configFile); //: node(configFile)
	~MultisensorFusion();

	Adsfi::HafStatus Init()
	{
		return node.Init();
	};

	void Stop() {
		node.Stop();
	};

	bool IsError() const {
		return errorStatus;
	}

	void Process();		//进程管理

private:
    // ===== 线程函数 =====
	void mdcBroadcastData();
	void getPreFusion();
	void RecvLocation();
	void getFusionSend();
	void getZcData();
	void setupUDP();
	void udpReceiveTunableSignalThread();
	void sendUDPTunSpeed(uint8_t diRection);

	// ===== 工具函数 =====
	float getAnotherYaw(float yaw) {
		float anotherYaw = yaw + M_PI;
		anotherYaw = fmod(anotherYaw + M_PI, 2 * M_PI);
		if (anotherYaw < 0) {
			anotherYaw += 2 * M_PI;
		}
		anotherYaw -= M_PI;
		return anotherYaw;
	}

	// ===== 成员变量 =====
    bool errorStatus{false};
    Adsfi::PlanningBase node;
    std::vector <std::thread> pool;
    bool isThreadRunning = true;
    udpCommHelp udpHelp;

    std::ofstream sendFile, slamFile, recvFile;

	// UDP通信相关
	int udp_socket_ = -1;                            // UDP socket
	struct sockaddr_in udp_server_addr_;             // UDP服务器地址（接收端）
	struct sockaddr_in udp_client_addr_;             // UDP客户端地址（发送端）
	std::thread udp_receive_thread_;                 // UDP接收线程
	std::atomic<bool> udp_running_;                  // UDP线程运行标志
	int udp_server_port_ = 8889;                     // UDP接收端口
	std::string udp_client_ip_ = "192.168.30.42";    // UDP发送目标IP
	int udp_client_port_ = 8888;                     // UDP发送目标端口

	// ===== 核心数据（带锁保护） =====
	Fusions fusions_;                    // 融合数据对象
	mutable std::mutex fusions_mutex_;   // 融合数据互斥锁

	// 工位点相关
	std::string workList_;
	std::vector<Pair> workPairs_;

	// ===== 线程安全的数据访问方法 =====
	// 获取融合数据的拷贝（用于发送）
	Fusions getFusionsCopy() {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		return fusions_;
	}

	// 更新Slam数据
	void updateSlam(const std::function<void(Slam&)>& updater) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		updater(fusions_.slam);
	}

	// 更新障碍物数据
	void updateObstacle(const std::function<void(Obstacle&)>& updater) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		updater(fusions_.obstacle);
		fusions_.updateObstacleStatus();
	}

	// 更新ZcData数据
	void updateZcData(const std::function<void(ZcData&)>& updater) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		updater(fusions_.zcData);
	}

	// 更新传感器故障
	void updateSensorFault(const std::function<void(SensorFault&)>& updater) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		updater(fusions_.sensorFault);
	}

	// 获取方向（线程安全）
	int getDirection() {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		return fusions_.direction;
	}

	// 设置方向（线程安全）
	void setDirection(int dir) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		fusions_.direction = dir;
	}

	// 获取障碍物状态
	uint8_t getObstacleStatus() {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		return fusions_.isReceiveObstacle;
	}

	// 获取ZcData的拷贝
	ZcData getZcDataCopy() {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		return fusions_.zcData;
	}

	// 更新目标工位点
	void updateTargetPair(size_t index) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		if (index < workPairs_.size()) {
			fusions_.targetPair = workPairs_[index];
		}
	}

	// 获取目标工位点
	Pair getTargetPair() {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		return fusions_.targetPair;
	}

	// 更新转盘确认状态
	void updateTunableStatusConfirm(uint8_t status) {
		std::lock_guard<std::mutex> lock(fusions_mutex_);
		fusions_.slam.tunable_status_confirm = status;
	}
};

#endif
