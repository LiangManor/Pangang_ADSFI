#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <chrono> 
#include <thread>
#include <iostream>
#include <fstream>//c++文件操作的头文件
#include <vector>
#include <sstream>

using namespace std;



struct ONLINEPOINT
{
    std::vector<double> lat;
    std::vector<double> lon;
    std::vector<double> heading;
};




class RFID
{
public:
    // 设置RFID的 IP 地址和端口。
    std::string ip = "192.168.1.117";
    int port       = 10001;
    int client_socket;
    // xml 指令: 建立连接
    std::string connect_message = "\
    <frame>\
    <cmd>\
    <id> 1 </id>\
    <hostGreetings>\
    <supportedVersions>\
    <version> V2.2 </version>\
    </supportedVersions>\
    </hostGreetings>\
    </cmd>\
    </frame>";
    // xml 指令: 获取ID  天线1
    std::string ID_message1 = "\
    <frame>\
    <cmd>\
    <id> 2 </id>\
    <readTagIDs>\
    <sourceName> Readpoint_1 </sourceName>\
    </readTagIDs>\
    </cmd>\
    </frame>";
    // xml 指令: 获取ID  天线2
    std::string ID_message2 = "\
    <frame>\
    <cmd>\
    <id> 3 </id>\
    <readTagIDs>\
    <sourceName> Readpoint_2 </sourceName>\
    </readTagIDs>\
    </cmd>\
    </frame>";

    // xml 指令: 取消连接
    std::string disConnect_message = "\
    <frame>\
    <cmd>\
    <id> 3 </id>\
    <hostGoodbye>\
    </hostGoodbye>\
    </cmd>\
    </frame>";
    
    ONLINEPOINT op;
    
    bool isUsable = false;
    
public:
    RFID()
    {
        // 创建客户端sock
        this->client_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (this->client_socket == -1)
        {
            std::cout<<"socket creation failed"<<std::endl;
            return;
        }
        
        // vlan
        const char *interface_name = "eth0.12"; 
        if (setsockopt(this->client_socket, SOL_SOCKET, SO_BINDTODEVICE, interface_name, strlen(interface_name)) < 0) 
        {
            perror("setsockopt error!\n");
            close(client_socket);
            return;
        }
        
        sockaddr_in server_address{};
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &server_address.sin_addr) <= 0) 
        {
            std::cerr << "Invalid IP address format" << std::endl;
            close(client_socket);
            return;
        }
        // 连接
        if (connect(this->client_socket, reinterpret_cast<sockaddr*>(&server_address), sizeof(server_address)) != 0) 
        {
            std::cout<<" ------------------------------- Error connecting to the server"<<std::endl;
            close(client_socket);
            return;
        }
        
        this->isUsable = true;//如果连接上rfid主机，则使能rfid标签
    }
    // 发送请求连接
    void connectRF650R()
    {
        send(this->client_socket, this->connect_message.c_str(), this->connect_message.length(), 0);
        char response_data[1024]{};
        if(recv(this->client_socket, response_data, 1024, 0) < 0)
        {
            std::cout<<"Error receiving response from the server1"<<std::endl;
        }
        else 
        {
            // std::cout<<"response_data:"<<response_data<<std::endl;
        }
        // 接收反馈
        char response_data1[1024]{};
        if(recv(this->client_socket, response_data1, 1024, 0) < 0) 
        {
            std::cout<<"Error receiving response from the server2"<<std::endl;
        }
        else 
        {
            // std::cout<<"response_data1:"<<response_data1<<std::endl;
        }
    }
    // 获取卡片ID
    void getID(int & id_, int & intensity_)
    {
        send(this->client_socket, ID_message1.c_str(), ID_message1.length(), 0);
        char response_data2[1024]{};
        int bytes_received = recv(this->client_socket, response_data2, 1024, 0);
        if(bytes_received < 0) 
        {
/*            std::cout<<"Error receiving response from the server3"<<std::endl;*/
        }
        else
        {
            std::string data(response_data2, bytes_received);// 取 response_data2 中的前bytes_received个字节
            std::string startTag ;
            std::string endTag   ;
            // 【解析 rssi】
            // 起始终值位标签
            startTag = "<rSSI>";
            endTag   = "</rSSI>";
            std::string sub1;
            // 起始终止位index
            size_t startPos;
            size_t endPos;
            startPos = data.find(startTag);
            if (startPos == std::string::npos)// 判断是否找到起始标签
            {
                sub1 = "-1";  // 如果未找到，设置为 0 
            }
            else
            {
                endPos = data.find(endTag, startPos);
                if (endPos == std::string::npos)
                {
                    sub1 = "-1";  // 如果未找到，设置为 0 
                }
                else
                {
                    sub1 = data.substr(startPos + startTag.length(), endPos - startPos - startTag.length());
                }
            }
            
            
            int64_t sub11;
            try 
            {
                sub11 = std::stoll(sub1);
            }
            catch (const std::invalid_argument&) 
            {
                sub11 = -1;  // 如果转换失败，将sub1设为默认值-1
            }
            intensity_ = sub11;
            // 【解析 tagid】 
            startTag = "<tagID>";
            endTag   = "</tagID>";
            startPos = data.find(startTag);
            std::string sub2;
            if (startPos == std::string::npos)
            {
                sub2 = "-1";  // 如果未找到，设置为 -1
            }
            else 
            {
                size_t endPos = data.find(endTag, startPos);
                if (endPos == std::string::npos) 
                {
                    sub2 = "-1";  // 如果未找到，设置为 -1
                }
                else 
                {
                    sub2 = data.substr(startPos + startTag.length(), endPos - startPos - startTag.length());
                }
            }
            id_ = std::stoi(sub2);
        }
        if((id_ == -1) || (intensity_ == -1))
        {
            send(this->client_socket, ID_message2.c_str(), ID_message2.length(), 0);
            char response_data2[1024]{};
            int bytes_received = recv(this->client_socket, response_data2, 1024, 0);
            if(bytes_received < 0) 
            {
/*                std::cout<<"Error receiving response from the server3"<<std::endl;*/
            }
            else
            {
                std::string data(response_data2, bytes_received);// 取 response_data2 中的前bytes_received个字节
                std::string startTag ;
                std::string endTag   ;
                // 【解析 rssi】
                // 起始终值位标签()
                startTag = "<rSSI>";
                endTag   = "</rSSI>";
                std::string sub1;
                // 起始终止位index
                size_t startPos;
                size_t endPos;
                startPos = data.find(startTag);
                if (startPos == std::string::npos)// 判断是否找到起始标签
                {
                    sub1 = "-1";  // 如果未找到，设置为 -1
                }
                else
                {
                    endPos = data.find(endTag, startPos);
                    if (endPos == std::string::npos)
                    {
                        sub1 = "-1";  // 如果未找到，设置为 -1
                    }
                    else
                    {
                        sub1 = data.substr(startPos + startTag.length(), endPos - startPos - startTag.length());
                    }
                }
                int64_t sub11;
                try 
                {
                    sub11 = std::stoll(sub1);
                }
                catch (const std::invalid_argument&) 
                {
                    sub11 = -1;  // 如果转换失败，将sub1设为默认值0
                }
                intensity_ = sub11;
                //std::cout<<"rssi = "<<sub11<<std::endl;
                // 【解析 tagid】 
                startTag = "<tagID>";
                endTag   = "</tagID>";
                startPos = data.find(startTag);
                std::string sub2;
                if (startPos == std::string::npos)
                {
                    sub2 = "-1";  // 如果未找到，设置为 0
                }
                else 
                {
                    size_t endPos = data.find(endTag, startPos);
                    if (endPos == std::string::npos) 
                    {
                        sub2 = "-1";  // 如果未找到，设置为 0
                    }
                    else 
                    {
                        sub2 = data.substr(startPos + startTag.length(), endPos - startPos - startTag.length());
                    }
                }
                id_ = std::stoi(sub2);
            }
        }
    }
    
    void disConnectRF650R()
    {
        send(this->client_socket, disConnect_message.c_str(), disConnect_message.length(), 0);
        char response_data3[1024]{};
        if(recv(this->client_socket, response_data3, 1024, 0) < 0) 
        {
            std::cout<<"Error receiving response from the server4"<<std::endl;
        } 
        else
        {
            //std::cout<<"response_data3"<<response_data3<<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        close(this->client_socket);
    }
    
    
    
    // 字符串分割
    std::vector<std::string> split(const std::string &readData)
    {
           std::vector<std::string> str_vec_ptr;
           std::string token;
           std::stringstream ss(readData);  //stringstream::str将流中数据转换成string字符串
            // 如果输入数据为空，则直接返回空的vector
            if (readData.empty()) 
            {
                return str_vec_ptr;
            }
           while (getline(ss, token, ',')){//从字符串中ss读取字符，以逗号为结束符，将读到的数据存入token变量;直到读取完一整条nmea数据
                 str_vec_ptr.push_back(token);
           }
           return str_vec_ptr;
    }
    // 0.读取所有上线点的坐标信息
    void getOnlinePoint(std::string path)
    {
        std::ifstream file_read(path);
        std::string line;
        while(file_read)
        {
            if(getline(file_read,line)) //从文件file_path中获取以'\n'为结尾的字符串,存入line中,然后文件指针自动下移至下一行
            {
                std::vector<std::string> str_vec_ptr = split(line);
                if(str_vec_ptr.empty()) continue;
                this->op.lat.push_back(std::stod(str_vec_ptr[0]));
                this->op.lon.push_back(std::stod(str_vec_ptr[1]));
                this->op.heading.push_back(std::stod(str_vec_ptr[2]));
            }
        }
    }
    
    bool run(double & lat, double & lon, double & heading,int & id,int & intensity)
    {
        getID(id,intensity);
        if(id != -1)
        {
            lat     = this->op.lat[id];
            lon     = this->op.lon[id];
            heading = this->op.heading[id];
//            std::cout<<"  (rfid.h)lat = "<<std::fixed << std::setprecision(7) <<lat<<
//            		   "  lon = "<<std::fixed << std::setprecision(7)<<lon<<
//					   "  heading = "<<std::fixed << std::setprecision(7)<<heading<<std::endl;
            return true;
        }
        return false;
    }
    
    
    ~RFID()
    {
        close(this->client_socket);
    }
};
