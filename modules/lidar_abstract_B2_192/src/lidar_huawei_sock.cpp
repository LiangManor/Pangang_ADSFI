#include "lidar_huawei_sock.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>


int m_sock_msop;
int m_sock_difop;
int m_sock;
std::vector<std::vector<float> > lidar_angle;    //角度
std::vector<std::vector<float> > lidar_dist;        //距离
std::vector<std::vector<int> > lidar_inst;        //强度
std::vector<std::vector<double> > lidar_mtimestamp;  //时间戳
uint8_t pkt_year, pkt_month, pkt_day, pkt_hour, pkt_min, pkt_sec;
uint64_t packet_time_us;
uint64_t packet_time_s;
int return_mode_;
int point_num;
double cos_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];
double sin_scan_altitude_caliration[LSC16_SCANS_PER_FIRING];
double hortical_angle_caliration[LSC16_SCANS_PER_FIRING];



void InitSocket() {
    //创建socket
    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    m_sock_difop = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_sock== -1 || m_sock_difop == -1)
    {
        perror("socket");
        return;
    }
    
    // vlan
	const char *interface_name = "eth0.15"; // A4虚拟网卡
	if (setsockopt(m_sock, SOL_SOCKET, SO_BINDTODEVICE, interface_name, strlen(interface_name)) < 0) 
    {
        perror("setsockopt error!\n");
        return;
    }
    
    
    //将sock绑定组播地址与端口
    struct sockaddr_in sockAddr_msop;
    sockAddr_msop.sin_family = AF_INET;
    sockAddr_msop.sin_port = htons(2368);
    sockAddr_msop.sin_addr.s_addr = inet_addr("239.255.0.1");
    int retVal = bind(m_sock, (struct sockaddr *) &sockAddr_msop, sizeof(sockAddr_msop));
    if (retVal == 0){
        std::thread lidar_receive_thread(LidarReceiveMsopThread);
        lidar_receive_thread.detach();
    }
    else if(retVal == -1){
        perror("msopbind error");
    }

    // 将sock加入到多播组, 通过组播地址来区分
    struct ip_mreqn opt;
    inet_pton(AF_INET, "239.255.0.1", &opt.imr_multiaddr.s_addr);
    opt.imr_address.s_addr = htonl(INADDR_ANY);
    opt.imr_ifindex = if_nametoindex("eth0.15");// B2虚拟网卡
    if (setsockopt(m_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &opt, sizeof(opt)) < 0)
    {
        perror("setsockopt error!\n");
        return;
    }
}



void LidarReceiveMsopThread() {
    struct sockaddr_in addrFrom;
    unsigned int len = sizeof(sockaddr_in);
    // 接收数据
    char recvBuf[1021] = {0};
    int16_t HorizontalAngleOffset[4][1500] = {0};
    int16_t FovVerticalAngleOffset[4][192] = {0};
    int recvLen;
	double point_time = 0.0;
	float azimuth; 
	float azimuth_corrected_f;
	float last_azimuth;
	float azimuth_diff;
    std::vector<std::vector<float> > angle;
    std::vector<std::vector<float> > distance;
    std::vector<std::vector<int> > intensity;
    std::vector<std::vector<double> > mtimestamp;
    angle.resize(192);
    distance.resize(192);
    intensity.resize(192);
    mtimestamp.resize(192);
    bool first_time=true;

    while (true) {
        //获取套接字接收内容
        recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (struct sockaddr *) &addrFrom, &len);
        if (recvLen > 0) 
        {
        	unsigned char data[998];
            for (int i = 0; i < 998; i++) {
            	data[i] = recvBuf[23+i];
                //data.push_back(recvBuf[i]);
            }

			const raw_packet_t* raw = (const raw_packet_t*)&data[0];
            
            struct tm cur_time;
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = pkt_year + 2000 - 1900;
            cur_time.tm_mon = pkt_month - 1;
            cur_time.tm_mday = pkt_day;
            cur_time.tm_hour = pkt_hour;
            cur_time.tm_min = pkt_min;
            cur_time.tm_sec = pkt_sec;
			
            packet_time_s = timegm(&cur_time);
            packet_time_us = 0;//data[1203] * pow(2, 24) + data[1202] * pow(2, 16) + data[1201] * pow(2, 8) + data[1200];
            double packet_timestamp = packet_time_s + static_cast<double>(packet_time_us * 1e-6);
            if(first_time)
            {
               for(int i=0; i<1500; i++)
               {
               			HorizontalAngleOffset[0][i] = (int16_t)((FovVerticalAngleOffset1[2*i] << 8) | FovVerticalAngleOffset1[2*i+1]);
               			HorizontalAngleOffset[1][i] = (int16_t)((FovVerticalAngleOffset2[2*i] << 8) | FovVerticalAngleOffset2[2*i+1]);
               			HorizontalAngleOffset[2][i] = (int16_t)((FovVerticalAngleOffset3[2*i] << 8) | FovVerticalAngleOffset3[2*i+1]);
               			HorizontalAngleOffset[3][i] = (int16_t)((FovVerticalAngleOffset4[2*i] << 8) | FovVerticalAngleOffset4[2*i+1]);
               }
               for(int i=0; i<192; i++)
			   {
			  			FovVerticalAngleOffset[0][i] =  (int16_t)((FovHorizontalAngleOffset1[2*i] << 8) | FovHorizontalAngleOffset1[2*i+1]);
			  			FovVerticalAngleOffset[1][i] =  (int16_t)((FovHorizontalAngleOffset2[2*i] << 8) | FovHorizontalAngleOffset2[2*i+1]);
			  			FovVerticalAngleOffset[2][i] =  (int16_t)((FovHorizontalAngleOffset3[2*i] << 8) | FovHorizontalAngleOffset3[2*i+1]);
			  			FovVerticalAngleOffset[3][i] =  (int16_t)((FovHorizontalAngleOffset4[2*i] << 8) | FovHorizontalAngleOffset4[2*i+1]);
			   }
               first_time = false;
            }
            float angle_degree = PI/180.f;
			for(int i = 0; i < 192; i++)
			{
				double VerticalAngleoffsets = FovVerticalAngleOffset[raw->lutIndex][i]/1000.0;
				double VerticalAngles = VerticalAngleoffsets + ((int16_t)((FovHorizontalAngleData[2*i] << 8) | FovHorizontalAngleData[2*i+1]))*0.00390625;
				//double VerticalAngles = VerticalAngleoffsets - i * 0.10417 + 20.0;
				cos_scan_altitude_caliration[i] = cos(angle_degree * VerticalAngles);
				sin_scan_altitude_caliration[i] = sin(angle_degree * VerticalAngles);
			}


			for (int block = 0; block < 1; block++)  // 1 packet:12 data blocks
			{
				if (UPPER_BANK != raw->blocks[block].header)
				{
				  std::cout << "---------------" << raw->blocks[block].header <<std::endl;
				  std::cout << "header error" << std::endl;
				  break;
				}
				
				azimuth = (float)(256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);
				azimuth = azimuth/256;

				int Azimuth1 = raw->Azimuth11 * 256 + raw->Azimuth12;
				int Azimuth2 = raw->Azimuth21 * 256 + raw->Azimuth22;

				if(block == 0)
					azimuth = azimuth + HorizontalAngleOffset[raw->lutIndex][Azimuth1]/1000;
				else
					azimuth = azimuth + HorizontalAngleOffset[raw->lutIndex][Azimuth2]/1000;
			
				if(azimuth > 120.0)
					azimuth = azimuth - 256;

				if(azimuth < 0.0)
					azimuth = azimuth + 360;
				//std::cout << "azimuth " << azimuth << std::endl;
				
				for (int dsr = 0, k = 0; dsr < SCANS_PER_BLOCK; dsr++, k += 5)  // 96  3
				{
					azimuth_corrected_f = azimuth >360.0 ? azimuth - 360.0 : azimuth;

					union two_bytes tmp;
					tmp.bytes[1] = raw->blocks[block].data[k];
					tmp.bytes[0] = raw->blocks[block].data[k + 1];
					int distance3 = tmp.uint; 

					// read intensity
					int intensity1 = raw->blocks[block].data[k + 4];
					float distance2 =distance3 * distance_unit_;	
					point_num++;

					if (last_azimuth - azimuth_corrected_f > 10 && point_num > 3000)
					{
						//std::cout << azimuth_corrected_f << " -  "<< last_azimuth  << " -  "<< point_num << std::endl;
						if (lidar_angle.empty()) {
							lidar_dist = distance;
							lidar_inst = intensity;
							lidar_angle = angle;
							lidar_mtimestamp = mtimestamp;
						}
						angle.clear();
						angle.resize(192);
						distance.clear();
						distance.resize(192);
						intensity.clear();
						intensity.resize(192);
						mtimestamp.clear();
						mtimestamp.resize(192);
						point_num = 0;
					} 
					else {
						angle[dsr].push_back(azimuth_corrected_f);
						distance[dsr].push_back(distance2);
						intensity[dsr].push_back(intensity1);
						mtimestamp[dsr].push_back(point_time);
					}
					last_azimuth = azimuth_corrected_f;
				}
			}
		}
    }
}


void LidarReceiveDifopThread() {
    struct sockaddr addrFrom;
    unsigned int len = sizeof(sockaddr_in);
    //接收数据
    char recvBuf[1248] = {0};
    int recvLen;

    while (true) {

        recvLen = recvfrom(m_sock_difop, recvBuf, sizeof(recvBuf), 0, (sockaddr *) &addrFrom, &len);
		
        //DIFOP数据 0xA5 0xFF 0x00 0x5A
        if (recvLen > 0 && (recvBuf[0] == -91 || recvBuf[0] == 165))// && recvBuf[1] == -1 && recvBuf[2] == 0 && recvBuf[3] == 90) 
		{
            std::vector<char> data;
            for (int i = 0; i < 1248; i++) {
                data.push_back(recvBuf[i]);
            }
	  
			return_mode_ = 1;//data[185]+1;
			
			for(int i = 0; i < LSC16_SCANS_PER_FIRING; i++)
			{
				union vertical_angle
				{
					uint8_t data[4];
					uint32_t vertical_angle;
				} ver_angle;
				ver_angle.data[0] = data[468 + i * 3 + 2];
				ver_angle.data[1] = data[468 + i * 3 + 1];
				ver_angle.data[2] = data[468 + i * 3];
				ver_angle.data[3] = 0;

				int neg = 1;
				int vertical_angles = (int)(ver_angle.vertical_angle);
				if(i > 7)
				{
					neg = -1;
					vertical_angles = vertical_angles - 0xffff;
				}
				
				float angle_degree = PI/180.f;
			}
			for(int i = 0; i < LSC16_SCANS_PER_FIRING; i++)
			{
				union hortical_angle
				{
					uint8_t data[4];
					uint32_t hortical_angle;
				} hor_angle;
				hor_angle.data[0] = data[564 + i * 3 + 2];
				hor_angle.data[1] = data[564 + i * 3 + 1];
				hor_angle.data[2] = data[564 + i * 3];
				hor_angle.data[3] = 0;

				int neg = 1;
				int hortical_angles = (int)(hor_angle.hortical_angle);
				if(hortical_angles > 0xffff)
				{
					neg = -1;
					hortical_angles = hortical_angles - 0xffff;
				}
				
				hortical_angle_caliration[i] = hortical_angles * neg;
				hortical_angle_caliration[i] = hortical_angles * neg;
			}

            //年月日
            pkt_year = data[52];
            pkt_month = data[53];
            pkt_day = data[54];
			pkt_hour = data[55];
            pkt_min = data[56];
            pkt_sec = data[57];
        } else {
            sleep(1);
        }
    }
}
