#include <string>
#include <sstream>
#include <iomanip>
#include "ara/log/logging.h"
#include "ara/exec/execution_client.h"
// can数据接收头文件
#include "socket_can_interface.h"
#include "can_receive_utils.h"

// imu头文件
#include <ara/imu/imuinfoserviceinterface_skeleton.h>
#include <ara/imu/imuinfoserviceinterface_common.h>
// gnss头文件(具体地址 /usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/include/adsfi/arxml_include/ara/gnss)
#include <ara/gnss/gnssinfoserviceinterface_skeleton.h>
#include <ara/gnss/gnssinfoserviceinterface_common.h>

#include <ara/core/instance_specifier.h>
#include <ara/exec/execution_client.h>

// 点云Mviz可视化
#include <publisher.h> 


#define GNSSLOG
#define IMULOG



using mdc::canReceive::CanReceiveUtils;
using gnssSkeleton = ara::gnss::skeleton::GnssInfoServiceInterfaceSkeleton;
using imuSkeleton  = ara::imu::skeleton::ImuInfoServiceInterfaceSkeleton;
using namespace ara::com;
using namespace ara::core;

// cm通讯
std::shared_ptr<gnssSkeleton> gnss_Skeleton;
std::shared_ptr<imuSkeleton>  imu_Skeleton;

int  counter = 0;
float biasAx = 0.0;
float biasAy = 0.0;
float biasZAngular = 0.0;
uint32_t counterImu = 0;


ara::ins::Time GetCurrentTime();
void HCFDINSPVATB(const std::vector<uint8_t> & data);
void HCFDRAWIMUVB(const std::vector<uint8_t> & data);



/*
 * 打印can数详细信息
 */
static void PrintCanData(const timeval &tstamp, const std::string &canName, const can_frame &canFrame)
{
    std::stringstream ssData;
    CanReceiveUtils::FormatCanFrameData(ssData, canFrame);
    std::stringstream ssCanId;
    CanReceiveUtils::FormatCanId(ssCanId, canFrame.can_id);
    std::cout << " (" << CanReceiveUtils::FormatTime(tstamp) << ")  " << canName << "  " << std::hex <<
                 std::uppercase << ssCanId.str() << std::dec << "  [" <<
                 static_cast<std::uint32_t>(canFrame.can_dlc) << "]  "<< std::endl << ssData.str() << std::endl;
}



/*
 * 打印canfd数详细信息
 */
static void CanfdData(const timeval &tstamp, const std::string &canName, const canfd_frame canfdFrame)
{
    const std::uint32_t lenBase = 10;
    std::stringstream ssData;
    CanReceiveUtils::FormatCanfdFrameData(ssData, canfdFrame);
    std::stringstream ssCanId;
    CanReceiveUtils::FormatCanId(ssCanId, canfdFrame.can_id);
    //std::cout<<"原始的数据[ID:"<<canfdFrame.can_id<<"]"<<std::endl;
    // 判断是否是0x904的ID
    if(canfdFrame.can_id == 904)
    {
        //std::cout<<"< --------------- ID: 904 --------------- >"<<std::endl;
        // 数据类型转换
        std::string token;
        std::vector<uint8_t> data;
        while (ssData >> token) 
        {
            std::stringstream ss;
            ss << std::hex << token; 
            unsigned int value;
            ss >> value; 
            data.push_back(static_cast<uint8_t>(value));
        }
        // 数据解析
        HCFDINSPVATB(data);
    }
    
    // 判断是否是0x903的ID
    if(canfdFrame.can_id == 903)
    {
        //std::cout<<"< --------------- ID: 903 --------------- >"<<std::endl;
        // 数据类型转换
        std::string token;
        std::vector<uint8_t> data;
        while (ssData >> token) 
        {
            std::stringstream ss;
            ss << std::hex << token; 
            unsigned int value;
            ss >> value; 
            data.push_back(static_cast<uint8_t>(value));
        }
        // 数据解析
        HCFDRAWIMUVB(data);
    }
}

class lossPassFilter
{
public:
//float meanAx     = 0.0;
//float meanAy     = 0.0;
//float meanAngZ   = 0.0;
//float varianceAx = 0.0;
//float varianceAy = 0.0;
//float varianceAngZ = 0.0;
//std::vector<float> vctMeanAx;
//std::vector<float> vctMeanAy;
//std::vector<float> vctMeanAngZ;


// 三轴加速度
float meanAx     = 0.0;
float varianceAx = 0.0;
std::vector<float> vctMeanAx;

float meanAy     = 0.0;
float varianceAy = 0.0;
std::vector<float> vctMeanAy;

float meanAz     = 0.0;
float varianceAz = 0.0;
std::vector<float> vctMeanAz;

// 三轴角速度
float meanAngX   = 0.0;
float varianceAngX = 0.0;
std::vector<float> vctMeanAngX;

float meanAngY   = 0.0;
float varianceAngY = 0.0;
std::vector<float> vctMeanAngY;

float meanAngZ   = 0.0;
float varianceAngZ = 0.0;
std::vector<float> vctMeanAngZ;



public:
lossPassFilter(){}
~lossPassFilter(){}

// 计算平均值
float calculateMean(const std::vector<float>& data)
{
	float sum = 0.0;
	for(float value : data)
	{
		sum += value;
	}
	return sum / data.size();
}

// 计算方差
float calculateVariance(const std::vector<float>& data, float mean)
{
	float sumSquaredDifferences = 0.0;
    for(float value : data)
    {
    	float difference = value - mean;
        sumSquaredDifferences += difference * difference;
    }
    return sumSquaredDifferences / data.size();
}

float LowPassFilter_kalman(float data, float R_)
{
	static float Xlast=0.0;
	static float P=0.1;
	float Q = 0.01;
	float R = R_;
	float Kg,PP;
	//1式A=1 无输入
	PP=P+Q; //Ppre=Plast+Q （2）
	Kg=PP/(PP+R); //更新Kg Ppre/Ppre+R（4）
	Xlast = Xlast+Kg*(data-Xlast); // Xnow = Xpre+Kg*(Z(k)-H*Xpre)(3)
	P=(1-Kg)*PP; //Pnow=(I-Kg)*Ppre(5) 由于Pnow不会再使用，所以更新Plast=Pnow
	return Xlast;
}

};

lossPassFilter filter;

float convertHeading(float heading) {
    // 将航向角从正北轴转换为正东轴的角度
    float newHeading = heading + 90.0f;

    // 将角度调整到 -180 到 180 度范围
    if (newHeading > 180.0f) {
        newHeading -= 360.0f;
    } else if (newHeading < -180.0f) {
        newHeading += 360.0f;
    }

    return newHeading;
}



int main(int argc, char* argv[])
{
    // can通讯相关配置
    InitLogging("CANR", "CAN_RECEIVE_DEMO", ara::log::LogLevel::kVerbose, (ara::log::LogMode::kRemote));
    ara::exec::ExecutionClient execClient;
    execClient.ReportExecutionState(ara::exec::ExecutionState::kRunning);
    ara::log::Logger& mainLog {ara::log::CreateLogger("can receive", "can receive sample context", ara::log::LogLevel::kVerbose)};
    std::string canName = "can7";// 引脚标签 gps11 对应soc上的 can7
    mdc::canDemo::SocketCanInterface socketCan;
    if (!socketCan.Init(canName)) {
        mainLog.LogError() << canName << "Init failed!";
        return -1;
    }
    timeval tv = {2, 0};
    socketCan.SetRecvTimeout(tv);
    // 注册GNSS的CM服务，目的是往下游节点发送数据
    gnss_Skeleton = std::make_shared<gnssSkeleton>(InstanceSpecifier("ins_a_cm/ins_a_cm/GnssRawToAppPPort"), MethodCallProcessingMode::kEvent);
    gnss_Skeleton->OfferService();// gnss 的CM服务开启
    // 注册IMU的CM服务，目的是往下游节点发送数据
    imu_Skeleton  = std::make_shared<imuSkeleton>(InstanceSpecifier("ins_a_cm/ins_a_cm/ImuToAppPPort"), MethodCallProcessingMode::kEvent);
    imu_Skeleton->OfferService();// imu 的CM服务开启
    
    // 注册ros话题
    //imuPub = mdc::visual::Publisher::Advertise<mdc::visual::Twist>(ara::core::String("imu"));
    
    int counter = 0;
    while(true)
    {
        canfd_frame receiveFrame;
        struct timeval tstamp;
        std::int32_t readBytes = 0;
        const auto ret = socketCan.ReadCan(receiveFrame, tstamp, readBytes);
        if (ret == -1)
        {
            // 读取超时
            std::cout<<"发送异常标志位"<<std::endl;
            //Event内存
            auto sampleGnss = gnss_Skeleton->mdcEvent.Allocate();
            sampleGnss->header.seq = 1;// 1表示异常，uint32_t
            sampleGnss->longitude = -1;// 经度信息
            sampleGnss->latitude  = -1;// 纬度信息
            sampleGnss->elevation = -1;
            sampleGnss->linearVelocity.x = -1;
            sampleGnss->linearVelocity.y = -1;
            sampleGnss->linearVelocity.z = -1;
            gnss_Skeleton->mdcEvent.Send(std::move(sampleGnss));
            continue;
        }
        if (readBytes == sizeof(can_frame))
        {
            PrintCanData(tstamp, canName, *(reinterpret_cast<can_frame*>(&receiveFrame)));
            //Event内存
            auto sampleGnss = gnss_Skeleton->mdcEvent.Allocate();
            sampleGnss->header.seq = 0;// 1表示异常，uint32_t
            gnss_Skeleton->mdcEvent.Send(std::move(sampleGnss));
//            std::cout<<"解析can"<<std::endl;
            counter = 0;
        }
        else
        {
            CanfdData(tstamp, canName, receiveFrame);
//            std::cout<<"解析canFD"<<std::endl;
            counter = 0;
        }
    }
    gnss_Skeleton->StopOfferService();//gnss的CM服务关闭
    imu_Skeleton->StopOfferService();//gnss的CM服务关闭
    mainLog.LogInfo() << "Can receive sample done...";
    execClient.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
    return 0;
}


ara::ins::Time GetCurrentTime()
{
    const auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    uint32_t sec = std::chrono::duration_cast<std::chrono::seconds>(now).count();
    uint32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1000000000UL;
    return ara::ins::Time { sec, nsec };
}

void HCFDINSPVATB(const std::vector<uint8_t> & data)
{
    //Event内存
    auto sampleGnss = gnss_Skeleton->mdcEvent.Allocate();
    // 参数1:GPS周（单位: 周） ---------------已通过
    uint16_t gpsWeek = 0x00;
    gpsWeek = ((data[0] << 8) + (data[1]));
    gpsWeek = gpsWeek * 1 + 0;
//    std::cout<<"gpsWeek:"<<gpsWeek<<std::endl;
    // 参数2:GPS周内秒（单位: s） ---------------已通过
    uint32_t gpsTime = 0x00;
    gpsTime = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + (data[5]);
    gpsTime = gpsTime * 0.001 + 0;
//    std::cout<<"gpsTime:"<<gpsTime<<std::endl;
    /*
        有符号的物理量用有符号整形接收16进制的原始字节数
    */
    //----------------------------------------------------------------------------------------------------------------------------------------
    //    得配置组合模式才能输出： vgyro_x  vgyro_y  vgyro_z  accel_x  accel_y  accel_z 这几个物理量是车辆坐标系下的，而不是组合惯导主机         //
    //
    //参数3:x轴角速度（单位:deg/s）
    int32_t vgyro_x_hex = 0x00;
    vgyro_x_hex = (data[6] << 8) + (data[7]);
    vgyro_x_hex = (vgyro_x_hex << 4) + ((data[8]>>4) & 0x0f);
    float vgyro_x = (float)(vgyro_x_hex * 0.001 - 500);
//    std::cout<<"vgyro_x:"<<vgyro_x<<"  ";
    // 参数4:y轴角速度（单位:deg/s）
    int32_t vgyro_y_hex = 0x00;
    vgyro_y_hex = data[8] & 0x0f;
    vgyro_y_hex = (vgyro_y_hex << 8) + data[9];
    vgyro_y_hex = (vgyro_y_hex << 8) + data[10];
    float vgyro_y =  (float)(vgyro_y_hex * 0.001 - 500);
//    std::cout<<"vgyro_y:"<<vgyro_y<<"  ";
    // 参数5:z轴角速度（单位:deg/s）
    int32_t vgyro_z_hex = 0x00;
    vgyro_z_hex = (data[11] << 8) + data[12];
    vgyro_z_hex = (vgyro_z_hex << 4) + ((data[13] >> 4) & 0x0f);
    float vgyro_z =  (float)(vgyro_z_hex * 0.001 - 500);
//    std::cout<<"vgyro_z:"<<vgyro_z<<std::endl;
    // 参数6:x轴加速度(单位: g)
    int32_t accel_x_hex = 0x00;
    accel_x_hex = data[13] & 0x0f;
    accel_x_hex = (accel_x_hex << 8) + data[14];
    accel_x_hex = (accel_x_hex << 8) + data[15];
    float accel_x = (float)(accel_x_hex * 0.00002 - 10);
//    std::cout<<"accel_x:"<<accel_x<<"  ";
    // 参数7:y轴加速度, no g (单位: g)
    int32_t accel_y_hex = 0x00;
    accel_y_hex = (data[16] << 8) + data[17];
    accel_y_hex = (accel_y_hex << 4) + ((data[18] >> 4) & 0x0f);
    float accel_y = (float)(accel_y_hex * 0.00002 - 10);
//    std::cout<<"accel_y:"<<accel_y<<"  ";
    // 参数8:z轴加速度(单位: g)
    int32_t accel_z_hex = 0x00;
    accel_z_hex = data[18] & 0x0f;
    accel_z_hex = (accel_z_hex << 8) + data[19];
    accel_z_hex = (accel_z_hex << 8) + data[20];
    float accel_z = (float)(accel_z_hex * 0.00002 - 10);
//    std::cout<<"accel_z:"<<accel_z<<std::endl;
    //----------------------------------------------------------------------------------------------------------------------------------------
    //                                             只有锁定卫星后才能输出：lon lat alt vel_E_N_U                                             //
    // 参数10:lon 经度（单位:deg,范围：-180 到 180） ---------------已通过
    int64_t lon_hex = 0x00;
    lon_hex = data[25] & 0x0f;
    lon_hex = (lon_hex << 8) + data[26];
    lon_hex = (lon_hex << 8) + data[27];
    lon_hex = (lon_hex << 8) + data[28];
    lon_hex = (lon_hex << 8) + data[29];
    double lon = (double)(lon_hex * (1E-08) - 180);
//    std::cout<<std::fixed << std::setprecision(7) <<"lon:"<<lon<<"  ";
    // 参数9:lat 纬度（单位:deg，范围：-90 到 90） ---------------已通过
    int64_t lat_hex = 0x00;
    lat_hex = (data[21] << 24) + (data[22] << 16) + (data[23] << 8) + data[24];
    lat_hex = (lat_hex << 4) + ((data[25] >> 4) & 0x0f);
    double lat = (double)(lat_hex * (1E-08) - 90);
//    std::cout<<std::fixed << std::setprecision(7) <<"lat:"<<lat<<"  ";
    // 参数11：大地高（单位:米）
    int32_t alt_hex = 0x00;
    alt_hex = (data[30] << 16) + (data[31] << 8) + data[32];
    float alt = (float) (alt_hex * 0.001 - 5000);
    //std::cout<<std::fixed << std::setprecision(7) <<"alt:"<<alt<<std::endl;
    
    // 参数12:(东向速度)
    int16_t vel_E_hex = 0x00;
    vel_E_hex = (data[33] << 8) + data[34];
    float vel_E = (float) (vel_E_hex * 0.005 - 0);
//    std::cout<<"vel_E:"<<vel_E<<"  ";
    // 参数13:(北向速度)
    int16_t vel_N_hex = 0x00;
    vel_N_hex = (data[35] << 8) + data[36];
    float vel_N = (float) (vel_N_hex * 0.005 - 0);
//    std::cout<<"vel_N:"<<vel_N<<"  ";
    // 参数14:(天向速度)
    int16_t vel_U_hex = 0x00;
    vel_U_hex = (data[37] << 8) + data[38];
    float vel_U = (float) (vel_U_hex * 0.005 - 0);
//    std::cout<<"vel_U:"<<vel_U<<std::endl;
    sampleGnss->longitude = lon;// 经度信息
    sampleGnss->latitude  = lat;// 纬度信息
    sampleGnss->elevation = (double)alt;
    sampleGnss->linearVelocity.x = (double)vel_E_hex;
    sampleGnss->linearVelocity.y = (double)vel_N_hex;
    sampleGnss->linearVelocity.z = (double)vel_U_hex;
    //------------------------------------------------------------------------------------------------------------------------------
    //                                             IMU直接输出：pitch roll ，而 yaw，就是航向角（不搜星是不输出的）                                             //
    // 参数15:(俯仰角pitch,单位：deg) 
    int16_t pitch_hex = 0x00;
    pitch_hex = (data[39] << 8) + data[40];
    float pitch =  (float)( pitch_hex * 0.01 + 0);
    //std::cout<<"pitch:"<<pitch<<"  ";
    // 参数16:(横滚角roll,单位：deg)
    int16_t roll_hex = 0x00;
    roll_hex = (data[41] << 8) + data[42];
    float roll = (float) (roll_hex * 0.01 + 0);
    //std::cout<<"roll:"<<roll<<"  ";
    // 参数17:(航向角yaw,单位[-180,180]：deg)
    int16_t yaw_hex = 0x00;
    yaw_hex = (data[43] << 8) + data[44];
    float yaw = (float) (yaw_hex * 0.01 + 0);
    // 将航向角转为以正东为基准轴 ， 按右手坐标系 ， 东偏北为正 
    yaw = convertHeading(yaw); // 此时单位为: deg 
//    std::cout<<"yaw:"<<yaw<<std::endl; //
    //x分量：车辆水平时，横滚角为零，左侧高于右侧时为正，相反为负。
    //y分量：车辆水平时，俯仰角为零，上坡俯仰角为正，下坡俯仰角为负。
    //z分量：车辆指向北方，方位角为零，车辆指向东方，方位角为正。
    sampleGnss->attitude.x = (double)pitch;
    sampleGnss->attitude.y = (double)roll;
    sampleGnss->attitude.z = (double)yaw;
    //------------------------------------------------------------------------------------------------------------------------------
    //                                             参数方差                                             //
    // 参数18:(lat位置标准差,单位：米)
    int16_t std_lat_hex = 0x00;
    std_lat_hex = (data[45] << 4) + ((data[46] >> 4) & 0x0f);
    float std_lat = (float) (std_lat_hex * 0.01 + 0);
    // 参数19:(lon位置标准差,单位：米)
    int16_t std_lon_hex = 0x00;
    std_lon_hex = ((data[46] & 0x0f) << 8) + data[47];
    float std_lon = (float) (std_lon_hex * 0.01 + 0);
    // 参数20(alt位置标准差,单位：米)
    int16_t std_alt_hex = 0x00;
    std_alt_hex = (data[48] << 4) + ((data[49] >> 4) & 0x0f);
    float std_alt = (float) (std_alt_hex * 0.01 + 0);
    // 参数21(东向速度标准差,单位:米)
    uint32_t std_vel_E_hex = 0x00;
    std_vel_E_hex = ((data[49] & 0x0f) << 8) + data[50];
    float std_vel_E = (float) (std_vel_E_hex * 0.01 -150);
    // 参数22(北向速度标准差,单位:米)
    uint32_t std_vel_N_hex = 0x00;
    std_vel_N_hex =  (data[51] << 4) + ((data[52] >> 4) & 0x0f);
    float std_vel_N = (float) (std_vel_N_hex * 0.01 -150);
    // 参数23(天向速度标准差,单位:米)
    uint32_t std_vel_U_hex = 0x00;
    std_vel_U_hex = ((data[52] & 0x0f) << 8) + data[53];
    float std_vel_U = (float) (std_vel_U_hex * 0.01 -150);
    // 参数24:(俯仰角标准差,单位:deg)
    int16_t std_pitch_hex = 0x00;
    std_pitch_hex = (data[54] << 4) + ((data[55] >> 4) & 0x0f);
    float std_pitch = (float) (std_pitch_hex * 0.01 + 0);
    // 参数25:(横滚角标准差,单位:deg)
    int16_t std_roll_hex = 0x00;
    std_roll_hex = ((data[55] & 0x0f) << 8) + data[56];
    float std_roll = (float) (std_roll_hex * 0.01 + 0);
    // 参数26(航向角标准差,单位:deg)
    int16_t std_yaw_hex = 0x00;
    std_yaw_hex = (data[57] << 4) + ((data[58] >> 4) & 0x0f);
    float std_yaw = (float) (std_yaw_hex * 0.01 + 0);
    //----------------------------------------------------------------------------------------------------------------------------------------
    //                               只有锁定卫星后才能输出heading、speed: 且与定向天线有关，需配置相关参数                                    //
    // 参数27(航迹角（0`360）,单位:deg)
    int16_t heading_hex = 0x00;
    heading_hex = ((data[58] & 0x0f) << 8) + data[59];
    heading_hex = (heading_hex << 4) + ((data[60] >> 4) & 0x0f);
    float heading = (float) (heading_hex * 0.01 + 0);
    //std::cout<<"  航迹角:"<<heading<<std::endl;
    // 参数28(地面速度，单位:m/s)
    int16_t speed_hex = 0x00;
    speed_hex = ((data[60] & 0x0f) << 8) + data[61];
    speed_hex = (speed_hex << 2) + ((data[62] >> 6) & 0x03);
    float speed = (float) (speed_hex * 0.01 + 0);
//    std::cout<<"地面速度:"<<speed<<std::endl;
    //----------------------------------------------------------------------------------------------------------------------------------------
    
    // 参数29(组合状态)
    /*
    stat_ins = 0: 初始化
    stat_ins = 1: 卫导模式
    stat_ins = 2: 组合导航模式    √    
    stat_ins = 3: 纯惯导模式
    */
    int stat_ins;
    stat_ins         = (int)((data[62] >> 4) & 0x03);
//    std::cout<<"0: 初始化  1: 卫导模式  2: 组合导航模式  3: 纯惯导模式"<<std::endl;
//    std::cout<<"stat_ins = "<<stat_ins<<std::endl;
    // 参数30(gnss状态)
    /*
    GNSS状态：
    0：不定位不定向；设备当前没有进行定位，也没有进行定向。通常在设备启动时或信号不足的情况下出现
    1：单点定位定向；适用于需要简单定位且方向信息的场景
    2：伪距差分定位定向；设备使用伪距差分技术进行定位，伪距差分定位是通过计算基准站和移动站之间的伪距差异来提高定位精度，同时进行定向
    3：组合推算；设备使用组合推算方法进行定位和定向。这可能包括将不同定位技术（如GNSS、惯性导航系统等）结合起来进行更高精度的定位和方向推算
    4：RTK 稳定解定位定向   √    稳定解表示定位结果较为可靠和准确。
    5：RTK浮点解定位定向；浮点解可能在环境变化或信号质量不佳时不如稳定解准确
    6：单点定位不定向；
    7：伪距差分定位不定向；
    8：RTK稳定解定位不定向；
    9：RTK浮点解定位不定向
    */
    //   stat_ins = 2: 组合导航模式    stat_pos = 4: RTK 稳定解定位定向     则表示定位效果很精确
    int stat_pos; // 定位状态
    stat_pos         = (int)((data[62]) & 0x0f);
    // 从以上解释结果来看 1，2，3，4，5 都可以适应需求 
//    std::cout<<"0: 不定位不定向    1: 单点定位定向       2: 伪距差分定位定向     3: 组合推算  4:RTK 稳定解定位定向  5:RTK浮点解定位定向"<<std::endl
//             <<"6：单点定位不定向  7：伪距差分定位不定向  8：RTK稳定解定位不定向  9：RTK浮点解定位不定向"<<std::endl;
//             std::cout<<"stat_pos = "<<stat_pos<<std::endl;
    sampleGnss->positionType = stat_pos; // uint16_t
    int sensor_used;
    sensor_used         = data[63];
    sampleGnss->header.stamp = GetCurrentTime();
    sampleGnss->header.seq = 0;//0 表示正常
    /* 对于该报文，我们只需要经纬度、航向角和定位状态下发给下一个节点 */
    gnss_Skeleton->mdcEvent.Send(std::move(sampleGnss));
    
    
    
    
#ifdef GNSSLOG



#endif




}


void HCFDRAWIMUVB(const std::vector<uint8_t> & data)
{
//##############################################################################################################
//   惯导主机安装情况:惯导主机的y轴正方向朝向为车头正前方，x轴正方向朝向为车头右方
//   车子左转yawRate为正值，右转为负值
//   车子向前加速: yAccl为正值     车子向减速  : yAccl为负值
//   车子向右加速: xAccl为正值     车子向左加速: xAccl为负值
//###############################################################################################################
    //Event内存
    auto sampleImu = imu_Skeleton->mdcEvent.Allocate();
    //参数1:gps周
    uint16_t gpsWeek = 0x00;
    gpsWeek = ((data[0] << 8) + (data[1]));
    gpsWeek = gpsWeek * 1 + 0;
    //std::cout<<"gpsWeek:"<<gpsWeek<<std::endl;
    //参数2:gps Time
    uint32_t gpsTime = 0x00;
    gpsTime = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + (data[5]);
    gpsTime = gpsTime * 0.001 + 0;
    //std::cout<<"gpsTime:"<<gpsTime<<std::endl;
    //参数3:x轴角速度（单位:deg/s）
    int32_t vgyro_x_hex = 0x00;
    vgyro_x_hex = (data[6] << 8) + (data[7]);
    vgyro_x_hex = (vgyro_x_hex << 4) + ((data[8]>>4) & 0x0f);
    float vgyro_x = (float)(vgyro_x_hex * 0.001 - 500);
    //std::cout<<"vgyro_x:"<<vgyro_x<<"  ";
    //参数4:y轴角速度（单位:deg/s）
    int32_t vgyro_y_hex = 0x00;
    vgyro_y_hex = data[8] & 0x0f;
    vgyro_y_hex = (vgyro_y_hex << 8) + data[9];
    vgyro_y_hex = (vgyro_y_hex << 8) + data[10];
    float vgyro_y =  (float)(vgyro_y_hex * 0.001 - 500);
    //std::cout<<"vgyro_y:"<<vgyro_y<<"  ";
    // 参数5:z轴角速度（单位:deg/s）
    int32_t vgyro_z_hex = 0x00;
    vgyro_z_hex = (data[11] << 8) + data[12];
    vgyro_z_hex = (vgyro_z_hex << 4) + ((data[13] >> 4) & 0x0f);
    float vgyro_z =  (float)(vgyro_z_hex * 0.001 - 500);
    //std::cout<<"vgyro_z:"<<vgyro_z<<std::endl;
    // 参数6:x轴加速度(单位: g)
    int32_t accel_x_hex = 0x00;
    accel_x_hex = data[13] & 0x0f;
    accel_x_hex = (accel_x_hex << 8) + data[14];
    accel_x_hex = (accel_x_hex << 8) + data[15];
    float accel_x = (float)(accel_x_hex * 0.00002 - 10);
    //std::cout<<"accel_x:"<<accel_x<<"  ";
    // 参数7:y轴加速度, no g (单位: g)
    int32_t accel_y_hex = 0x00;
    accel_y_hex = (data[16] << 8) + data[17];
    accel_y_hex = (accel_y_hex << 4) + ((data[18] >> 4) & 0x0f);
    float accel_y = (float)(accel_y_hex * 0.00002 - 10);
    //std::cout<<"accel_y:"<<accel_y<<"  ";
    // 参数8:z轴加速度(单位: g)
    int32_t accel_z_hex = 0x00;
    accel_z_hex = data[18] & 0x0f;
    accel_z_hex = (accel_z_hex << 8) + data[19];
    accel_z_hex = (accel_z_hex << 8) + data[20];
    float accel_z = (float)(accel_z_hex * 0.00002 - 10);
    //std::cout<<"accel_z:"<<accel_z<<std::endl;
    
    // 填入传感器数据
    // 三轴角速度(rad/s)
    sampleImu->angularVelocity.x = vgyro_x*(3.1415926536/180.0);//度 转 弧度
    sampleImu->angularVelocity.y = vgyro_y*(3.1415926536/180.0); 
    sampleImu->angularVelocity.z = vgyro_z*(3.1415926536/180.0);
    // 三轴加速度(m/s²)
    sampleImu->acceleration.x = accel_x * 9.81;// g 转 （m/s²）
    sampleImu->acceleration.y = accel_y * 9.81;
    sampleImu->acceleration.z = accel_z * 9.81;
    
    if(counter == 400 )
    {
		// 零漂校正：
		// 三轴加速度(m/s²)
		sampleImu->acceleration.x    = sampleImu->acceleration.x    - filter.meanAx;
		sampleImu->acceleration.y    = sampleImu->acceleration.y    - filter.meanAy;
		sampleImu->acceleration.z    = sampleImu->acceleration.z    - filter.meanAz;
		// 三轴角速度(rad/s)
		sampleImu->angularVelocity.x = sampleImu->angularVelocity.x - filter.meanAngX;
		sampleImu->angularVelocity.y = sampleImu->angularVelocity.y - filter.meanAngY;
		sampleImu->angularVelocity.z = sampleImu->angularVelocity.z - filter.meanAngZ;
//		std::cout<<"------零漂校正"<<"  ax = "<<sampleImu->acceleration.x
//				                 <<"  ay = "<<sampleImu->acceleration.y
//								 <<"  az = "<<sampleImu->acceleration.z
//								 <<"  xAngular = "<<sampleImu->acceleration.x
//								 <<"  yAngular = "<<sampleImu->acceleration.y
//								 <<"  zAngular = "<<sampleImu->angularVelocity.z<<std::endl;
		// 低通滤波
		// 三轴加速度(m/s²)
		sampleImu->acceleration.x    = filter.LowPassFilter_kalman(sampleImu->acceleration.x,    filter.varianceAx);
		sampleImu->acceleration.y    = filter.LowPassFilter_kalman(sampleImu->acceleration.y,    filter.varianceAy);
		sampleImu->acceleration.z    = filter.LowPassFilter_kalman(sampleImu->acceleration.z,    filter.varianceAz);
		// 三轴角速度(rad/s)
		sampleImu->angularVelocity.x = filter.LowPassFilter_kalman(sampleImu->angularVelocity.x, filter.varianceAngX);
		sampleImu->angularVelocity.y = filter.LowPassFilter_kalman(sampleImu->angularVelocity.y, filter.varianceAngY);
		sampleImu->angularVelocity.z = filter.LowPassFilter_kalman(sampleImu->angularVelocity.z, filter.varianceAngZ);
		//std::cout<<"低通滤波"<<"ax = "<<sampleImu->acceleration.x<<"  ay = "<<sampleImu->acceleration.y<<"  zAngular = "<<sampleImu->angularVelocity.z<<std::endl;
//		std::cout<<"------低通滤波"<<"  ax = "<<sampleImu->acceleration.x
//				                 <<"  ay = "<<sampleImu->acceleration.y
//								 <<"  az = "<<sampleImu->acceleration.z
//								 <<"  xAngular = "<<sampleImu->acceleration.x
//								 <<"  yAngular = "<<sampleImu->acceleration.y
//								 <<"  zAngular = "<<sampleImu->angularVelocity.z<<std::endl;
		// 发送数据
	    counterImu++;
        
        if(counterImu == 10)
        {
            counterImu = 0;
        }
		sampleImu->header.stamp = GetCurrentTime();
		sampleImu->header.seq   = counterImu;
		imu_Skeleton->mdcEvent.Send(std::move(sampleImu));
    }
    else
    {
    	counter++;
    	// 三轴加速度(m/s²)
    	filter.vctMeanAx.push_back(sampleImu->acceleration.x);
    	filter.vctMeanAy.push_back(sampleImu->acceleration.y);
    	filter.vctMeanAz.push_back(sampleImu->acceleration.z);
    	
    	
    	// 三轴角速度(rad/s)
    	filter.vctMeanAngX.push_back(sampleImu->angularVelocity.x);
    	filter.vctMeanAngY.push_back(sampleImu->angularVelocity.y);
    	filter.vctMeanAngZ.push_back(sampleImu->angularVelocity.z);
    	if(counter == 400)
    	{
    		// 计算均值
    		// 三轴加速度(m/s²)
    		filter.meanAx = filter.calculateMean(filter.vctMeanAx);
    		filter.meanAy = filter.calculateMean(filter.vctMeanAy);
    		filter.meanAz = filter.calculateMean(filter.vctMeanAz);
    		// 三轴角速度(rad/s)
    		filter.meanAngX = filter.calculateMean(filter.vctMeanAngX);
    		filter.meanAngY = filter.calculateMean(filter.vctMeanAngY);
    		filter.meanAngZ = filter.calculateMean(filter.vctMeanAngZ);
    		
    		
    		// 计算方差
    		// 三轴加速度(m/s²)
    		filter.varianceAx = filter.calculateVariance(filter.vctMeanAx, filter.meanAx);
    		filter.varianceAy = filter.calculateVariance(filter.vctMeanAy, filter.meanAy);
    		filter.varianceAz = filter.calculateVariance(filter.vctMeanAz, filter.meanAz);
    		
    		
    		// 三轴角速度(rad/s)
    		filter.varianceAngX = filter.calculateVariance(filter.vctMeanAngZ, filter.meanAngX);
    		filter.varianceAngY = filter.calculateVariance(filter.vctMeanAngZ, filter.meanAngY);
    		filter.varianceAngZ = filter.calculateVariance(filter.vctMeanAngZ, filter.meanAngZ);
    	}
	}
    
    
    
#ifdef IMULOG

#endif
}

















