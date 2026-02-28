//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
//component
#include "kf.h"
#include "hungarian_optimizer.h"
#include <fstream>  

using namespace std;

class TRACKER
{
public:
    float last_timestamp_;
    bool   first_run_;
    int    target_id_;
    std::vector<KF> targets_;// 目标维护列表
    float prevent_explosion_thres_;


public:
    TRACKER();//构造函数

    void iniTracker(std::vector<BoundingBox> & detctions);
    void track(std::vector<BoundingBox> & detctions,
               std::vector<BoundingBox> & track_result,
               float timestamp);
    void outPutObj(std::vector<BoundingBox> & track_result);
    ~TRACKER(){}//析构函数
};

/****************************************************
*                    实例化
*****************************************************/
// 构造函数实例化
TRACKER::TRACKER()
{
    first_run_ = true;
    target_id_ = 0;
    prevent_explosion_thres_ = 1000;
}


// 算法第一次运行
void TRACKER::iniTracker(std::vector<BoundingBox> & detctions)//已经进行推演
{
    for(int idx = 0; idx < detctions.size(); idx++)
    {
        KF oneObj;
        //初始化KF
        oneObj.in_box_ = detctions[idx];//传感器测到的参数
        oneObj.initialize(this->target_id_);//该函数使用之前需要传入传感器参数
        this->target_id_++;
        this->targets_.push_back(oneObj);
    }
}

// 输出稳定跟踪的目标
void TRACKER::outPutObj(std::vector<BoundingBox> & track_result)//已经进行推演
{
    
    if(this->target_id_ >=120) this->target_id_ = 0;
    for(int idx = 0; idx < this->targets_.size(); idx++)
    {
        if(targets_[idx].tracking_state_ == 10)
        {
            //如果跟踪目标是处于稳定跟踪状态，则后验估计一直使用传感器检测值
            targets_[idx].in_box_.trackID  = targets_[idx].kf_id_;
            targets_[idx].in_box_.v = targets_[idx].current_v;//因为lidar的采样频率为10Hz
            //目的是可视化预测框:
            targets_[idx].in_box_.center.x = targets_[idx].x_post_(0,0);
            targets_[idx].in_box_.center.y = targets_[idx].x_post_(1,0);
            track_result.push_back(targets_[idx].in_box_);
        }
    }
}


void TRACKER::track(std::vector<BoundingBox> & detctions,
                    std::vector<BoundingBox> & track_result,
                    float timestamp)
{

    if(first_run_)//判断跟踪算法是否是第一次运行
    {
        this->first_run_ = false;
        last_timestamp_  = timestamp;    
        iniTracker(detctions);
        outPutObj(track_result);
        return;
    }
    
    float dt = 0.1;
    /*****************************************************************************
    *                         初步筛选出上一时刻的跟踪目标
    ****************************************************************************/
    std::vector<KF> thisRoundTrackingTargets;//本轮最终得到的跟踪目标结果
    std::vector<KF> selectedTargets;//备选跟踪目标 
    std::vector<KF> outlyingTracker;//离群的跟踪目标
    std::vector<std::vector<float>> associationMatVec;
    for(int i = 0; i < this->targets_.size(); i++)
    {
        /***1***判断后验估计协方差矩阵行列式的值是否过大 or yawd的值是否过大，过大则认为跟踪失败******/
/*        std::cout<<"ID["<<targets_[i].kf_id_<<"]的分数："<<targets_[i].p_post_(2,0)<<std::endl;*/
        if(targets_[i].p_post_.determinant() > prevent_explosion_thres_)  
        {
            continue;
        }  
        /***2***获取先验估计**********/
        targets_[i].prediction(dt);       
        /***3***判断传感器估计值的协方差矩阵是否过大，过大则认为预测失败**********/
        float tempVal = targets_[i].s_pre_.determinant();
/*        targets_[i].score = tempVal;*/
        if( std::isnan(tempVal) ||
            (tempVal > prevent_explosion_thres_) )
        {
            continue;
        }
        
        bool enable = false;
        std::vector<float> disValue;
        for(int j = 0; j < detctions.size(); j++)
        {
            // 传感器测量值
            Eigen::MatrixXf meas;
            meas.resize(2,1);
            meas << detctions[j].center.x, detctions[j].center.y;    
            //残差 = 传感器测量值 - 传感器测量值的估计值
            Eigen::MatrixXf diff = meas - targets_[i].z_pre_;
            float dis;
            //情况1
            if(targets_[i].second_init_ == false)
            {
               float diff_x = diff(0,0);
               float diff_y = diff(1,0);
               dis = sqrt(diff_x*diff_x + diff_y*diff_y);
               if( dis < 4 && enable == false)
                  {
                      selectedTargets.push_back(targets_[i]);
                      enable = true;
                  }
            }
            else
            {
               //情况2
               //计算马氏距离:将马氏距离作为匈牙利算法的权重
               Eigen::MatrixXf M_Dis = diff.transpose() * targets_[i].s_pre_.inverse() * diff;
               dis = std::sqrt(M_Dis(0,0));    
               if( dis < 5 && enable == false)
               {
                   selectedTargets.push_back(targets_[i]);
                   enable = true;
               } 
            }
            
            
            disValue.push_back(dis);
        }
        //***4***:筛选出离群目标
        if(enable == false)
        {
            outlyingTracker.push_back(targets_[i]);
        }        
        else
        {
            associationMatVec.push_back(disValue);
        }
    }
    
    /*****************************************************************************
    *                              计算权重矩阵
    ****************************************************************************/      
    //初始化代价矩阵
    HungarianOptimizer<float> optimizer;
    optimizer.costs()->Resize(selectedTargets.size(), detctions.size());
    for(int i = 0; i < selectedTargets.size(); i++)//行
    {
        for(int j = 0; j < detctions.size(); j++)//列
        {
            (*(optimizer.costs()))(i, j) = associationMatVec[i][j];
        }
    }
    //求最小权匹配问题
    std::vector<std::pair<size_t, size_t>> assignments;
    optimizer.Minimize(&assignments); 
    std::vector<bool> uesdTargets(selectedTargets.size(), false);
    std::vector<bool> uesdDets(detctions.size(), false);
    
    //<------------情况1:匹配上了的上一时刻跟踪目标------------>
    for (const auto& assignment : assignments)
    {
        if(associationMatVec[assignment.first][assignment.second] >= 4) continue;
        selectedTargets[assignment.first].in_box_ = detctions[assignment.second];//存入传感器测量值
        if(selectedTargets[assignment.first].tracking_state_ != 10)
        {
            selectedTargets[assignment.first].tracking_state_++;
        }
        selectedTargets[assignment.first].in_box_.matchObjType = detctions[assignment.second].realType;
        uesdTargets[assignment.first] = true;
        uesdDets[assignment.second]   = true;
        thisRoundTrackingTargets.push_back(selectedTargets[assignment.first]);
    }    
    // 卡尔曼更新
    for(int i = 0; i < thisRoundTrackingTargets.size(); i++)
    {
        // 跟踪器是否完成了第二次初始化
        if(thisRoundTrackingTargets[i].second_init_ == false)
        {
            thisRoundTrackingTargets[i].secondInit(dt);//该函数使用之前需要传入传感器参数(已经在上面传入该参数)
        }
        else
        {
            thisRoundTrackingTargets[i].update(dt);//该函数使用之前需要传入传感器参数(已经在上面传入该参数)
        }
    }
    
    // <------------情况2:没匹配上的本时刻的检测目标-outlyingTracker----------->outlyingTracker
    for(int i = 0; i < detctions.size(); i++)
    {
        if(uesdDets[i] == false)
        {
            KF oneObj;
            // 初始化kf 
            oneObj.in_box_ = detctions[i];// 存入传感器测量值 
            oneObj.initialize(this->target_id_);
            this->target_id_++;
            thisRoundTrackingTargets.push_back(oneObj);  
        }
    }
    // <------------情况3:没匹配上的上一时刻的跟踪目标------------> 
    for(int i = 0; i < outlyingTracker.size(); i++)
    {
        if(outlyingTracker[i].tracking_state_ <= 0)//判断上一帧是否是误检 或者 判断匹配错误
        {
            continue;
        }
        outlyingTracker[i].tracking_state_ = outlyingTracker[i].tracking_state_ - 1;
        outlyingTracker[i].x_post_ = outlyingTracker[i].x_pre_;
        outlyingTracker[i].p_post_ = outlyingTracker[i].p_pre_;
        thisRoundTrackingTargets.push_back(outlyingTracker[i]);
    }
    targets_.clear();
    for(int i = 0; i < thisRoundTrackingTargets.size();i++)
    {
        targets_.push_back(thisRoundTrackingTargets[i]);
    }
    outPutObj(track_result);
    return;
}

















