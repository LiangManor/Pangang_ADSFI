#ifndef AN_H  // 头文件保护宏，必须是唯一的，通常用头文件名的大写加下划线
#define AN_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono> 
#include <pcl/kdtree/kdtree_flann.h>  // 新增KdTree头文件
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
using namespace std;

struct PointAN {
    double x, y;
    PointAN(double x_, double y_) : x(x_), y(y_) {}
};

struct AnchorBox {
    float center_x;
    float center_y;
    float center_z;
    float width;
    float height;
    float point_count;
    bool is_valid; // 新增有效性标志
};

class AN
{
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr anchor_boxes_select_center;
        pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        std::vector<AnchorBox> anchor_boxes;
        
    public:
    void init(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_)
    {
        // 初始化智能指针
        this->target_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        this->anchor_boxes_select_center = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        this->ndt_omp      = pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        // 初始化点云配准器
        this->ndt_omp->setResolution(5);//设置NDT网格结构的分辨率（必须大于0.1）目前最优值：5
        this->ndt_omp->setTransformationEpsilon (0.0001);//设置求解精度   目前最优值：0.0001
        this->ndt_omp->setStepSize (0.5);//为 More-Thuente 线搜索设置最大步长，太小容易陷入局部最优解  目前最优值：0.5
        this->ndt_omp->setMaximumIterations (20);//设置匹配迭代的最大次数 目前最优值：20
        this->ndt_omp->setNumThreads(6);//------------------------设置线程数 目前最优值：10
        this->ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        // 设定全局地图
        set_target_cloud(target_cloud_);
        // 生成锚框
        anchor_generate();
    }
    AN(){};
    ~AN(){};
    // 设定全局地图（该地图必须是原始地图，没经过滤波的）
    void set_target_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud_)
    {
        *this->target_cloud = *target_cloud_;
        // 初始化点云滤波器
        this->voxel_filter.setLeafSize(1.5f, 1.5f, 1.5f);
        this->voxel_filter.setInputCloud(this->target_cloud);
        this->voxel_filter.filter(*this->target_cloud);
        this->ndt_omp->setInputTarget(this->target_cloud);//设置地图
    }
    // 生成锚框
    void anchor_generate()
    {
        std::vector<Eigen::Vector3f> initPoints;
//        initPoints.push_back(Eigen::Vector3f(100,100,0));//D6
//        initPoints.push_back(Eigen::Vector3f(-124.831688,24.027381,-1.825784));//D10
//        initPoints.push_back(Eigen::Vector3f(100,100,0));//D16
//        initPoints.push_back(Eigen::Vector3f(-32.090272,12.906418,-1.112038));//D20
//        initPoints.push_back(Eigen::Vector3f(-6.38113,20.563415,-0.878961));//D22
//        initPoints.push_back(Eigen::Vector3f(12.508912,-0.79961,-0.595789));//W1
//        initPoints.push_back(Eigen::Vector3f(8.369863,11.984032,-0.702978));//W2
//        initPoints.push_back(Eigen::Vector3f(16.812908,20.143393,-0.614665));//W3
//		initPoints.push_back(Eigen::Vector3f(0.052003,-0.408061,-0.651298));//E1
//		initPoints.push_back(Eigen::Vector3f(0.02632,11.919968,-0.779458));//E2
//		initPoints.push_back(Eigen::Vector3f(-0.410687,20.729,-0.817513));//E3
//        initPoints.push_back(Eigen::Vector3f(-252.811486,35.404516,-6.912572));//1号炼钢
//        initPoints.push_back(Eigen::Vector3f(-243.707509,39.469838,-6.804280));//2号炼钢
//        initPoints.push_back(Eigen::Vector3f(-259.924334,41.576952,-7.077676));//3号炼钢
//        initPoints.push_back(Eigen::Vector3f(24.368005,-1.228467,-0.626337));//1号转盘
//        initPoints.push_back(Eigen::Vector3f(20.548017,11.650597,-0.608166));//2号转盘
//        initPoints.push_back(Eigen::Vector3f(28.799568,19.896493,-0.578445));//3号转盘
//        initPoints.push_back(Eigen::Vector3f(-196.192668,33.614549,-2.640604));//D4
//        initPoints.push_back(Eigen::Vector3f(-143.124963,26.418399,-2.117388));//D12

        for(int i = -100; i <= 100; i += 10){
        	for(int j = -100; j <= 100; j+= 10){
        		initPoints.push_back(Eigen::Vector3f(i,j,0));
        	}
        }

		for (int i = 0; i < initPoints.size(); ++i) {
			AnchorBox box;
			box.center_x = initPoints[i].x();
			box.center_y = initPoints[i].y();
			box.center_z = 180.0;
			box.width = 10;
			box.height = 10;
			box.point_count = initPoints[i].z() * 3.14 / 180.0;
			box.is_valid = true;
			this->anchor_boxes.push_back(box);
		}
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr align(pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration, 
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud ,
                                              float x,
                                              float y,
                                              float z,
                                              float roll,
                                              float pitch,
                                              float yaw) {
       registration->setInputSource(scan_cloud);
       pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    //    匹配算法数值初始化
          Eigen::Translation3f init_translation(x, y, z);
          Eigen::AngleAxisf init_rotation_x(roll, Eigen::Vector3f::UnitX());//roll
          Eigen::AngleAxisf init_rotation_y(pitch, Eigen::Vector3f::UnitY());//pitch
          Eigen::AngleAxisf init_rotation_z(yaw, Eigen::Vector3f::UnitZ());//yaw
          Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();
          registration->align(*aligned, init_guess);
          return aligned;
    }
    // 判断点是否在多边形内
    bool get_init_pose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud_aligned,
                       Eigen::Matrix4f& RT)
    {
    	bool isInitSuccess = false;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
        *scan_cloud_temp = *scan_cloud;
        this->voxel_filter.setInputCloud(scan_cloud_temp);
        this->voxel_filter.filter(*scan_cloud_temp);
        float minScore = 9999.;
        float x;
        float y;
        float z;
        pcl::PointCloud<pcl::PointXYZ>::Ptr alignedBest(new pcl::PointCloud<pcl::PointXYZ>());
        Eigen::Matrix4f bestLocation;
        std::cout << "开始计算初始化位置" << std::endl;
        for (size_t i = 0; i < this->anchor_boxes.size(); ++i)
        {
            // 判断anchor是否在规定区域内
            const auto& box = this->anchor_boxes[i];
            if (box.is_valid) // 判断毛框内的点是否达到阈值
            {
				pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
				aligned = align(ndt_omp, scan_cloud_temp,box.center_x,box.center_y,-2.32445,0,0,box.point_count);//返回转化后的点云
				//保存当前定位结果
				float score  = ndt_omp->getFitnessScore ();
				Eigen::Matrix4f lastLocation = ndt_omp->getFinalTransformation();
				if(score < 5000)
				{
					isInitSuccess = true;
					if(minScore > score)
					{
						minScore = score;
						scan_cloud_aligned->points.clear();
						*scan_cloud_aligned = *aligned;
						RT = lastLocation;
						x = box.center_x;
						y = box.center_y;
						z = box.center_z;
						std::cout << "存在更优的初始化位置，分数为：" << score << std::endl;
					}
				}
				std::cout << "location[" << i+1 << "] = " << score << std::endl;
            }
            pcl::PointXYZ pt;
            pt.x = box.center_x ;
            pt.y = box.center_y ;
            pt.z = -1.68076;
            this->anchor_boxes_select_center->points.push_back(pt);
        }
        return isInitSuccess;
    }
};


#endif // AN_H
