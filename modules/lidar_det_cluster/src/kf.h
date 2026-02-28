#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
/* 
问题记录:
1.通常所使用的匀速CV是线性模型，没有考虑到目标的转向运动，因此在转向时，会有一定的预测误差。后期可以将cv模型替换为ctrv模型
*/
using namespace std; 

struct historyPt 
{
    float timeStamp;
    float position_x;
    float position_y;
};



struct Point3D_ {
    float x, y, z;
};

struct BoundingBox
{
    std::vector<Point3D_> eightPts;
    float w;
    float h;
    float l;
    float yaw;
    float v;
    float erchengfa_v;
    Point3D_ center;
    std::string matchObjType;
    std::string realType;
    int trackID;
    float score;
    BoundingBox()
    {
       eightPts.resize(8);
       matchObjType = "unKnown";
       v = 0.0;
       trackID = -1;
    }
};

enum TrackingState:int
{
    IsFalseDet = -1,      // Start tracking:刚检测到的目标，都认为可能是误检
    Die = 0,              // No longer tracking：不再追踪
    Stable = 5            //丢失跟踪目标后，只对丢失的目标预测5帧，此处若想修改，则也需要在tracker.h的241行的判断条件“if(targets_[idx].tracking_state_ == 3)”进行修改
};

class KF
{
public:
    std::vector<historyPt> sideWindows;
    int kf_id_;//目标ID
    BoundingBox in_box_;
    int tracking_state_;//目标状态
    bool second_init_;//因为只有两帧才可以计算出速度
    Eigen::MatrixXf F;
    Eigen::MatrixXf H;//状态向量to传感器估计值的映射模型
    Eigen::MatrixXf R;//传感器噪声矩阵
    Eigen::MatrixXf Q;    
    Eigen::MatrixXf kGain;//卡尔曼增益
    Eigen::MatrixXf x_pre_;
    Eigen::MatrixXf x_post_;
    Eigen::MatrixXf p_pre_;
    Eigen::MatrixXf p_post_;
    Eigen::MatrixXf z_pre_;
    Eigen::MatrixXf s_pre_;
    
    
    int firstTest;
    float lastVx;
    float lastVy;
    float current_v;

public:
    KF();
    void initialize(const int target_id);
    void secondInit(float dt);    
    void prediction(const float dt);   
    void update(float dt);     
    ~KF(){}
};


KF::KF()
{
    firstTest = 0;
    sideWindows.resize(6);
    F.resize(4, 4);
    float reserved = 0;
    F << 1, 0, reserved, 0,
         0, 1, 0, reserved,
         0, 0, 1, 0,
         0, 0, 0, 1;
    current_v = 0.0;
    H.resize(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
    //定义传感器噪声矩阵R
    R.resize(2, 2);
    //过程噪声 0.05 > 模型噪声 0.01说明：更信任模型推算出来的值
    R <<  0.001, 0,
          0, 0.001;         
    //定义运动模型噪声矩阵Q
    
    Q.resize(4, 4);     
    Q<<0.1,0.,0.,0.,//关于cv模型坐标x的噪声
       0.,0.1,0.,0.,//关于cv模型坐标y的噪声
       0.,0.,0.1,0.,//关于cv模型x轴速度分量Vx的噪声
       0.,0.,0.,0.1;//关于cv模型y轴速度分量Vy的噪声
       
    kGain.resize(4, 2);
    kGain<<1,0,
           0,1,
           0,0,
           0,0;
    
    x_pre_.resize(4, 1);   
    x_post_.resize(4, 1);   
    p_pre_.resize(4, 4);   
    p_post_.resize(4, 4);   
    z_pre_.resize(2, 1);  
    s_pre_.resize(2, 2); 
    second_init_ = false;
    //初始化跟踪器状态:因为可能会有误检的情况
    tracking_state_ = TrackingState::IsFalseDet;  
}



void KF::initialize( const int target_id)
{
    this->kf_id_ = target_id;
    x_post_<< this->in_box_.center.x, this->in_box_.center.y, 0., 0.;//初始化后验估计
    p_post_<<1, 0, 0, 0,//初始化值越大，则表明刚开始更相信测量值；
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    sideWindows[0].timeStamp = 0;
    sideWindows[0].position_x = this->in_box_.center.x;
    sideWindows[0].position_y = this->in_box_.center.y;
             
    return;
}


void KF::secondInit(float dt)
{
    float current_x = this->in_box_.center.x;
    float diff_x = current_x - x_post_(0,0);
    float vx = diff_x/dt;
    
    float current_y = this->in_box_.center.y;
    float diff_y = current_y - x_post_(1,0);
    float vy   = diff_y/dt;
    
    this->x_post_ << current_x, current_y, vx ,vy;
    this->second_init_ = true;
    
    current_v = sqrt(vx*vx + vy*vy);
    
    //记录上一时刻的速度
    sideWindows[1].timeStamp = 0.1;
    sideWindows[1].position_x = this->in_box_.center.x;
    sideWindows[1].position_y = this->in_box_.center.y;

    return;
}




void KF::prediction(const float dt)
{
    //cv运动学模型
    F(0,2) = dt;
    F(1,3) = dt;
    if(this->tracking_state_ == TrackingState::Stable)
    {
       R <<  0.035, 0,
             0, 0.035;         
       Q<<0.0015,0.,0.,0.,
          0.,0.0015,0.,0.,
          0.,0.,0.0015,0.,
          0.,0.,0.,0.0015;
/*       float diff_vx = this->x_post_(2,0) - lastVx;*/
/*       float diff_vy = this->x_post_(3,0) - lastVy;*/
/*       float ax = diff_vx*10;*/
/*       float ay = diff_vy*10;*/
/*       Q<<0.5*0.1*0.1*ax,0.,0.,0.,*/
/*          0.,0.5*0.1*0.1*ay,0.,0.,*/
/*          0.,0.,0.1*ax,0.,*/
/*          0.,0.,0.,0.1*ay;*/
       
       
    }
    //计算先验估计 与 先验估计协方差矩阵
    float  dtTemp = dt ;
    bool   init = true;
    while( dtTemp > 0)
    { 
         //cv运动学模型
         F(0,2) = 0.05;
         F(1,3) = 0.05;
         if(init == true)
         {   
             //计算先验估计 与 先验估计协方差矩阵
             this->x_pre_ = F*this->x_post_;
             this->p_pre_ = F*(this->p_post_)*F.transpose() + Q;
             init = false;
         }
         else
         {
             //计算先验估计 与 先验估计协方差矩阵
             this->x_pre_ = F*this->x_pre_;
             this->p_pre_ = F*(this->p_pre_)*F.transpose() + Q;
         }
         dtTemp -= 0.05;
    }
    //获取传感器测量值估计值
    this->z_pre_ = H*this->x_pre_;
    this->s_pre_ = H*p_pre_*H.transpose() + R;
}






void KF::update(float dt)
{


    //上一时刻位姿
    float diff_x = this->in_box_.center.x - x_post_(0,0);
    float diff_y = this->in_box_.center.y - x_post_(1,0);
    float vx = diff_x/dt;
    float vy   = diff_y/dt;
    current_v = sqrt(vx*vx + vy*vy);
    
    
    
    
    //传感器测量值
    Eigen::MatrixXf meas;
    meas.resize(2, 1);
    meas<< this->in_box_.center.x, this->in_box_.center.y;
    
    //计算卡尔曼增益
    this->kGain = this->p_pre_ * H.transpose() * this->s_pre_.inverse();;
    
    //计算后验估计 与 后验估计协方差矩阵
    this->x_post_ = this->x_pre_ + this->kGain * ( meas - this->z_pre_);
    this->p_post_ = this->p_pre_ - this->kGain * H * this->p_pre_;
}

