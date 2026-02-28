#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <Eigen/Dense>



#define UN_PROCESSED 0     //未访问的点
#define PROCESSING   1     //纳入点云簇但未发展下线的点
#define PROCESSED    2     //已经处理的点

typedef pcl::PointXYZ PointT;

using namespace std;
using namespace pcl;

struct BBox
{
   PointT center;
   double roll;//单位:rad
   double pitch;
   double yaw;
   double length;
   double width;
   double height;
   double pca_x;
   double pca_y;
   double pca_z;
};

class KDTREEDBSCAN
{
public:
    //构造函数:初始化数据
/*    KDTREEDBSCAN(double xGate_, double yGate_, int minPoints_, int clusterMinPoint_, int clusterMaxPoint_);*/
     KDTREEDBSCAN();
    ~KDTREEDBSCAN();
   //--------------------参数设置
public:
   int    minPoints;       //核心点周围的最小点数（不包括核心点）
   int    clusterMinPoint; //点云簇最小点云数量
   int    clusterMaxPoint; //点云簇最大点云数量
   float xGate;//x轴方向门限
   float yGate; //y轴方向门限
   std::vector<pcl::PointCloud<PointT>::Ptr> cloudClusters;//用于计算L_shape计算
   std::vector<BBox> obj;
   
   // 体素滤波参数 与 IOU 区域
   float filterRes;//参数2：体素滤波阈值
   float roiXMin,roiYMin;
   float roiXMax,roiYMax;
   float egoXMin,egoYMin,egoZMin;
   float egoXMax,egoYMax,egoZMax;

   //L_shape迭代步长
   float angle_increment;
   
public:
   //功能函数1：设置聚类参数
   void setParam(float xGate_, 
                 float yGate_,
                 float minPoints_,
                 float clusterMinPoint_,
                 float clusterMaxPoint_);
   //功能函数2：实现聚类（kdtree + dbscan）
   void clustering(pcl::PointCloud<PointT>::Ptr cloud_filtered);
   //功能函数2.1：xy门限
   int XYSearch(   
               int index,               //当前待选核心点序号
               std::vector<int> & types,//某个点云的状态
               pcl::PointCloud<PointT>::Ptr cloud,
               std::list<int> & seed_queue);
   
   //功能函数3：实现L_shape
   vector<BBox> BoundingBox();
   //功能函数3.1：迭代计算旋转角度
   double calcClosenessCriterion(const std::deque<double>& C_1, const std::deque<double>& C_2);
};

