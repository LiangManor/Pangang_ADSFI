#include "dbscan_inwinic.h"


//构造函数:初始化数据
/*KDTREEDBSCAN::KDTREEDBSCAN(double xGate_, double yGate_, int minPoints_, int clusterMinPoint_, int clusterMaxPoint_)*/
KDTREEDBSCAN::KDTREEDBSCAN()
{
   //镭神激光雷达坐标系
   /*  z
    *  ^  
    *  |    ^ y(车头朝向)
    *  |   /
    *  |  /
    *  | /
    *   -----> x (车头右边)以下参数针对的是此坐标系*/
   //kitti数据集激光雷达坐标系
   /*  z
    *  ^  
    *  |    ^ y(车头左边)
    *  |   /
    *  |  /
    *  | /
    *   -----> x (车头朝向)以下参数针对的是此坐标系*/
   this->xGate = .1;//x轴方向门限【门限最小值要 >= 体素滤波的设定值(filterRes)】0.4
   this->yGate = .3; //y轴方向门限 0.4
   this->minPoints       = 4;//参数3：核心点周围的最小点数（不包括核心点）
   this->clusterMinPoint = 5;//参数4：点云簇最小点云数量 8
   this->clusterMaxPoint = 1000;//参数5：点云簇最大点云数量

   this->angle_increment = 5.*(M_PI / 180.0);//1度1度de开始迭代*/
}
//析构函数
KDTREEDBSCAN::~KDTREEDBSCAN(){}

   //功能函数1：设置聚类参数
   void KDTREEDBSCAN::setParam(float xGate_, 
                             float yGate_,
                             float minPoints_,
                             float clusterMinPoint_,
                             float clusterMaxPoint_)
    {
        this->xGate           = xGate_;//x轴方向门限【门限最小值要 >= 体素滤波的设定值(filterRes)】0.4
        this->yGate           = yGate_; //y轴方向门限 0.4
        this->minPoints       = minPoints_;//参数3：核心点周围的最小点数（不包括核心点）
        this->clusterMinPoint = clusterMinPoint_;//参数4：点云簇最小点云数量 8
        this->clusterMaxPoint = clusterMaxPoint_;//参数5：点云簇最大点云数量
    }

//功能函数2：实现聚类（kdtree + dbscan）
void KDTREEDBSCAN::clustering(pcl::PointCloud<PointT>::Ptr cloud_filtered)
{
   std::vector<pcl::PointIndices>  cluster_indices;
   this->cloudClusters.clear();//清空上一时刻的点云簇队列
   pcl::ExtractIndices<PointT> extract;
   pcl::PointIndices::Ptr needRemovePtr (new pcl::PointIndices());
   std::list<int> seed_queue;
   std::vector<int> types;
   for (int i = 0; i < cloud_filtered->points.size();)
   {
      needRemovePtr->indices.clear();
      types.clear();
      types.resize(cloud_filtered->points.size(), UN_PROCESSED);
      seed_queue.clear();
      seed_queue.emplace_back(i);
      //E邻域检索
      int nn_size = XYSearch(i, types, cloud_filtered, seed_queue);
      //判断是不是噪声点
      if (nn_size < minPoints)
      {
         types[i] = PROCESSED;
         //移除当前帧中，已经探索过的点云簇
         std::vector<int> seedvector;
         std::list<int>::iterator it = seed_queue.begin();
         while(it != seed_queue.end())
         {
            seedvector.emplace_back(*it);
            it++;
         }
         needRemovePtr->indices = seedvector;
         extract.setInputCloud(cloud_filtered);
         extract.setIndices(needRemovePtr);
         extract.setNegative(true);//设置为true来删除指定的索引点
         extract.filter(*cloud_filtered);
         continue;
      }
      //把核心点纳入点云簇中
      types[i] = PROCESSED;//给核心点i打上已经处理的标签
      std::list<int>::iterator it = seed_queue.begin();
      it++;
      while(it != seed_queue.end())
      {
         int cloud_index = *it;
         nn_size = XYSearch(cloud_index, types, cloud_filtered, seed_queue);
         types[cloud_index] = PROCESSED;
         it++;
      }
      std::vector<int> seedvector;
      std::list<int>::iterator it2 = seed_queue.begin();
      pcl::PointCloud<PointT>::Ptr oneCluster(new pcl::PointCloud<PointT>);
      while(it2 != seed_queue.end())
      {
         seedvector.emplace_back(*it2);
         oneCluster->points.emplace_back(cloud_filtered->points[*it2]);
         it2++;
      }
      cloudClusters.emplace_back(oneCluster);//压入一个点云簇
      needRemovePtr->indices = seedvector;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(needRemovePtr);
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
      if(cloud_filtered->points.size() < clusterMinPoint)
      {
         break;
      }
   }
}
//功能函数2.1：搜索方法
int KDTREEDBSCAN::XYSearch(  int   index, //当前待选核心点序号
                             std::vector<int> & types,//某个点云的状态
                             pcl::PointCloud<PointT>::Ptr cloud,
                             std::list<int> & seed_queue)
{
     int number = 0;
     for (int i = 0; i < cloud->points.size(); i++) 
     {
            if( (types[i] != UN_PROCESSED) || (i == index))//避免对同一个点进行多次压入
            {
               continue;
            }
            if( ( fabs(cloud->points[i].x - cloud->points[index].x) < xGate) && ( fabs(cloud->points[i].y - cloud->points[index].y) < yGate) )
            {
               seed_queue.emplace_back(i);
               types[i] = PROCESSING;
               number++;
            }
     }
     return number;
}
//功能函数3:L_shape
vector<BBox> KDTREEDBSCAN::BoundingBox()
{
   this->obj.clear();
   const double max_angle = M_PI / 2.0;//迭代搜索的终止角度

   double max_q;// q越大意味着点云与框越贴合
   double theta_star;
   Eigen::Vector2d e_1;
   Eigen::Vector2d e_2;
   Eigen::Vector2d e_1_unitVector;
   Eigen::Vector2d e_2_unitVector;
   Eigen::Vector2d e_1_star; 
   Eigen::Vector2d e_2_star;
   for(int index = 0; index < cloudClusters.size(); index++)//一次循环：画一个点云簇的方框
   {
	   if(cloudClusters[index]->size() < this->clusterMinPoint) continue;
		// 创建PCA对象
		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		*cloud = *cloudClusters[index];
		// 计算点云的协方差矩阵
		Eigen::Matrix3f covariance;
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
		// 使用 Eigen 库计算特征值和特征向量
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();

		for (double theta = 0; theta < max_angle; theta += this->angle_increment)
		{
			e_1 <<  std::cos(theta), std::sin(theta);
			e_2 << -std::sin(theta), std::cos(theta);
			std::deque<double> C_1;//C_1的物理含义为点投影到e_1，e_2坐标系后，在e_1轴上的坐标值
			std::deque<double> C_2;
			for (const auto& point : *cloudClusters[index])
			{
				C_1.emplace_back(point.x * e_1.x() + point.y * e_1.y());
				C_2.emplace_back(point.x * e_2.x() + point.y * e_2.y());
			}
			double q = calcClosenessCriterion(C_1, C_2);
			if( max_q < q || theta == 0)
			{
				max_q = q;
				theta_star = theta;
			}
		}
		//二维旋转矩阵
		e_1_star <<  std::cos(theta_star), std::sin(theta_star);
		e_2_star << -std::sin(theta_star), std::cos(theta_star);
		//求坐标在新坐标系下的表达
		std::vector<double> C_1_star;
		std::vector<double> C_2_star;
		//找出点云簇z轴方向上的最小值和最大值
		double min_z =  999999;
		double max_z = -999999;
		for (size_t i = 0; i < cloudClusters[index]->size(); ++i)
		{
			C_1_star.emplace_back(cloudClusters[index]->points[i].x * e_1_star.x() + cloudClusters[index]->points[i].y * e_1_star.y());
			C_2_star.emplace_back(cloudClusters[index]->points[i].x * e_2_star.x() + cloudClusters[index]->points[i].y * e_2_star.y());
			//求z轴方向的最大最小值
			if (cloudClusters[index]->points[i].z < min_z || i == 0) min_z = cloudClusters[index]->points[i].z;//
			if (max_z < cloudClusters[index]->points[i].z || i == 0) max_z = cloudClusters[index]->points[i].z;
		}
		/*
		* 获取拟合矩形四条边的线性表示：直线方程组一般式
		* e1轴对应的直线的序号为1，e2轴对应的直线的序号为2，然后顺时针依次为3,4
		*/
		const double min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
		const double max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
		const double min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
		const double max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

		const double a_1 = std::cos(theta_star);
		const double b_1 = std::sin(theta_star);
		const double c_1 = min_C_1_star;

		const double a_2 = -1.0 * std::sin(theta_star);
		const double b_2 = std::cos(theta_star);
		const double c_2 = min_C_2_star;

		const double a_3 = std::cos(theta_star);
		const double b_3 = std::sin(theta_star);
		const double c_3 = max_C_1_star;

		const double a_4 = -1.0 * std::sin(theta_star);
		const double b_4 = std::cos(theta_star);
		const double c_4 = max_C_2_star;

		e_1_unitVector << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
		e_2_unitVector << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
		//-----求BBox的航向角
		BBox oneBbox;
		oneBbox.roll  = 0.0;
		oneBbox.pitch = 0.0;
		oneBbox.yaw   = std::atan2(e_1_star.y(), e_1_star.x());
		//-----求BBox的长宽高
		// 获取序号为1，2直线的交点
		double intersection_x_12 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
		double intersection_y_12 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
		//获取序号为2，3直线的交点
		double intersection_x_23 = -1*(b_2*c_3 - b_3*c_2) / (a_2*b_3 - a_3*b_2);
		double intersection_y_23 = -1*(a_3*c_2 - a_2*c_3) / (a_2*b_3 - a_3*b_2);
		// 获取序号为3，4直线的交点
		double intersection_x_34 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
		double intersection_y_34 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

		double dis1 = std::sqrt((intersection_x_12 - intersection_x_23)*(intersection_x_12 - intersection_x_23)
				 +(intersection_y_12 - intersection_y_23)*(intersection_y_12 - intersection_y_23));
		double dis2 = std::sqrt((intersection_x_23 - intersection_x_34)*(intersection_x_23 - intersection_x_34)
				 +(intersection_y_23 - intersection_y_34)*(intersection_y_23 - intersection_y_34));
		/* 此处的长宽并不是说长度较长的就是长，短的就是宽
		* 因为L_shape是个反复迭代计算的过程，一开始长边平行于x轴，宽边平行于y轴，经过反复迭代后，得到边框的航向角yaw
		*  z
		*  ^
		*  |    ^ y
		*  |   /   -------
		*  |  /  /       /
		*  | /  /       /
		*  |/   --------
		*   -----> x
		*/
		oneBbox.length = dis1;
		oneBbox.width  = dis2;
		oneBbox.height = max_z - min_z;
        oneBbox.pca_x = eigen_values[0];
        oneBbox.pca_y = eigen_values[1];
        oneBbox.pca_z = eigen_values[2];
		//-----求BBox的中心点坐标
		oneBbox.center.x = (intersection_x_12 + intersection_x_34) / 2.0;
		oneBbox.center.y = (intersection_y_12 + intersection_y_34) / 2.0;
		oneBbox.center.z = (min_z + max_z)/2.0;
		this->obj.emplace_back(oneBbox);
   }
   return obj;
}

double KDTREEDBSCAN::calcClosenessCriterion(const std::deque<double>& C_1, const std::deque<double>& C_2)//目标函数：最小距离（贴进度）
{
  
    const double min_c_1 = *std::min_element(C_1.begin(), C_1.end());  
    const double max_c_1 = *std::max_element(C_1.begin(), C_1.end());  
    const double min_c_2 = *std::min_element(C_2.begin(), C_2.end());  
    const double max_c_2 = *std::max_element(C_2.begin(), C_2.end());  

    double v1;
    double v2;
    double beta = 0;
    auto C_1_iterator = C_1.begin();
    auto C_2_iterator = C_2.begin();

    for (int i = 0; i < C_1.size(); i++) 
    {
        v1 = std::min(max_c_1 - *C_1_iterator, *C_1_iterator - min_c_1);
        v2 = std::min(max_c_2 - *C_2_iterator, *C_2_iterator - min_c_2);
        beta +=min(v1, v2); 
        C_1_iterator++;
        C_2_iterator++;
    }
    beta = 1/beta;//取1/beta是为了将最小值变为最大值。
    return beta;
}



