#include <cfloat>
#include <chrono>
#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>

class WEEDREMOVAL
{
public:
    // lidar参数特性
     float ang_res_h     = 0.2; // 水平分辨率
     float ang_res_v     = 0.2; // 垂直分辨率
     float ang_bottom_v    = 15; // 最低点（或最高点）与 水平面的夹角的绝对值   //这里应该写激光雷达垂直视场角/2
     float ang_bottom_h    = 25; // 最左点（或最右点）与 垂直面的夹角的绝对值
     size_t Horizon_SCAN = int(ang_bottom_h*2/0.08);//每行最多点数 120/0.09 //当前 625 个点
     size_t N_SCAN       = int(ang_bottom_v*2/0.08);//每列最多点数 25/0.08  //当前 375 个点
     size_t max_ground_lines = 40;//最多有n条线打到地面上
    // 
     float imageMinRange = .1; //对该欧氏距离(单位:米)以外的点进行图像编码
     float segmentAlphaX = ang_res_h / 180.0 * M_PI;
     float segmentAlphaY = ang_res_v / 180.0 * M_PI;
     float segmentTheta = 30.0/180.0*M_PI; // 该值越大，说明平面特征越明显，即面特征的过滤条件越严格
     float groundAngle = 10./180.0*M_PI; // 地平面的角度阈值
    // 经过杂草去除后，剩下的面特征通过以下两个特性进行进一步筛选
    int segmentValidPointNum = 10;// 疑似面特征点的点云总数阈值
    int segmentValidLineNum = 3; // 疑似面特征所占据的行数
    uint16_t *queueIndX;
    uint16_t *queueIndY;
    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    std::vector<std::pair<int8_t, int8_t> > neighborIterator;
    std::pair<int8_t, int8_t> neighbor;
    int labelCount;

public:
    //构造函数与非构造函数
    WEEDREMOVAL()
    {
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    };
    ~WEEDREMOVAL(){};
    //步骤4:杂草滤波
    void weedRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_pc,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_pc_weeded);
    void projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud_);
    void labelComponents(int row, int col, cv::Mat & labelMat);
    void paramReset();
};




void WEEDREMOVAL::weedRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_pc,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_pc_weeded)
{

    // ------------------------------------------- 对非地面点进行图像编码 ---------------------------------------------- //
    pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    image_cloud->points.resize(this->N_SCAN * this->Horizon_SCAN);
    pcl::PointXYZI nanPoint;
    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;
    std::fill(image_cloud->points.begin(), image_cloud->points.end(), nanPoint);
    // 获取深度图 rangeMat
    this->rangeMat = cv::Mat(this->N_SCAN,this->Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX)); // 深度图刷新
    projectPointCloud(non_ground_pc, image_cloud);
    
    // ------------------------------------------- 通过深度图去除 位于水平面的点（） ---------------------------------------------- //
    size_t lowerInd, upperInd;//位于下方的线的idx，位于上方的线的idx
    float diffX, diffY, diffZ, angle;
    this->groundMat = cv::Mat(this->N_SCAN,this->Horizon_SCAN, CV_8S, cv::Scalar::all(0));//（1表示地面点）
    for (size_t j = 0; j < this->Horizon_SCAN; ++j)//遍历列，一列 375 个点
    {
        for (size_t i = 0; i < this->max_ground_lines; ++i)//遍历行，假定最多有 40 条线打到地面上(每一列的纵向最低40个点)
        {
            //同一列相连两行的点云的ID
            lowerInd = j + ( i )*this->Horizon_SCAN;//下面那行
            upperInd = j + (i+1)*this->Horizon_SCAN;//上面那行
            // 如果所取的两点存在无效点, 该点lowerInd或者(i,j)在点云地图groundMat中也为无效点
            if (image_cloud->points[lowerInd].intensity == -1 ||
                image_cloud->points[upperInd].intensity == -1)
            {
                // no info to check, invalid points
                this->groundMat.at<int8_t>(i,j) = -1;
                continue;
            }
            // 由上下两线之间点的XYZ位置得到两线之间的俯仰角(夹角)
            diffX = image_cloud->points[upperInd].x - image_cloud->points[lowerInd].x;
            diffY = image_cloud->points[upperInd].y - image_cloud->points[lowerInd].y;
            diffZ = image_cloud->points[upperInd].z - image_cloud->points[lowerInd].z;
            // 计算两点lowerInd和upperInd垂直高度diffZ和水平距离的夹角
            angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
            // 如果上述夹角小于10, 则(i, j) 和 (i+1, j)设置为地面标志1
            if (abs(angle) <= groundAngle)
            {
                this->groundMat.at<int8_t>(i,j) = 1;
                this->groundMat.at<int8_t>(i+1,j) = 1;
            }
        }
    }
    // ------------------------------------------- 通过BFS 去除团状杂草 ---------------------------------------------- //
    this->labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));//（-1表示地面点和无效点，0及其以上表示未被标记的点）
    // 先标记 labelMat 中的地面点
    for (size_t i = 0; i < this->N_SCAN; ++i)
    {
        for (size_t j = 0; j <this-> Horizon_SCAN; ++j)
        {
            if (this->groundMat.at<int8_t>(i,j) == 1 || this->rangeMat.at<float>(i,j) == FLT_MAX)   // rangeMat 是在转伪图像的过程中填充了点的欧式距离值
            {
                this->labelMat.at<int>(i,j) = -1;
            }
        }
    }
    this->queueIndX     = new uint16_t[this->N_SCAN * this->Horizon_SCAN];
    this->queueIndY     = new uint16_t[this->N_SCAN * this->Horizon_SCAN];
    this->allPushedIndX = new uint16_t[this->N_SCAN * this->Horizon_SCAN];
    this->allPushedIndY = new uint16_t[this->N_SCAN * this->Horizon_SCAN];
    //这些对象代表了二维网格中某个点的基本相邻位置（上、下、左、右）。
    labelCount = 1;
    // 1. 按行遍历所有点进行分割聚类，更新labelMat
    for (size_t i = 0; i < this->N_SCAN; ++i) 
    {
        for (size_t j = 0; j < this->Horizon_SCAN; ++j) 
        {
            // 对未被标记的像素进行BFS搜索
            if (this->labelMat.at<int>(i,j) == 0) 
            {
                // 经过labelComponents函数，同类点云具有相同的标号，噪声的标号为 999999。
                labelComponents( i, j, this->labelMat);
            }
        }
    }
    for (size_t i = 0; i < this->N_SCAN; ++i)//遍历行
    {
        for (size_t j = 0; j < this->Horizon_SCAN; ++j)//遍历列
        {
            if(this->labelMat.at<int>(i, j) == -1 ||  // 表示地面点
               this->labelMat.at<int>(i, j) == 999999)// 表示无效点
            {
                continue;
            }
            else 
            {
                pcl::PointXYZ pt;
                pt.x = image_cloud->points[j  + i * this->Horizon_SCAN].x;
                pt.y = image_cloud->points[j  + i * this->Horizon_SCAN].y;
                pt.z = image_cloud->points[j  + i * this->Horizon_SCAN].z;
                non_ground_pc_weeded->points.push_back(pt);
            }
        }
    }
    delete[] queueIndX;
    delete[] queueIndY;
    delete[] allPushedIndX;
    delete[] allPushedIndY;
    
    
}

void WEEDREMOVAL::projectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud_)
{
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    /*
    将点云编码成以下伪图片形式
                                                                    行（一共96行）
                                                                    ^
                                                                    |
                                                                    |
                                         index = (行数 * 480 + 列数)|
                                                     3 2 1 0 ......|
    列（一共480）<--------------------------------------------------0
    */
    cloudSize = non_ground_cloud_->points.size();
    pcl::PointXYZI thisPoint;
    for (size_t i = 0; i < cloudSize; ++i)
    {
        thisPoint.x = non_ground_cloud_->points[i].x;
        thisPoint.y = non_ground_cloud_->points[i].y;
        thisPoint.z = non_ground_cloud_->points[i].z;
        /************************************************************/
        /*               计算当前点所在 行 的索引rowIdn,相关解释        */
        /************************************************************/
        /*
        因为编码时，行序号从0开始，而下面代码中“verticalAngle + ang_bottom ”的原因是
        verticalAngle 最小值为-16.6，最大值为16.6，“verticalAngle + ang_bottom”这个操作
        是以 “-16.6”为原点，把y轴坐标系平移到-16.6；
        */
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI; // 单位变换为 度 // 该点在垂直方向与z=0平面的夹角
        rowIdn = (verticalAngle + ang_bottom_v) / ang_res_v;        // 将该点移到正值范围，然后判断落在哪个 0.2*0.2 的角度桶里，作为索引（第几行）
        if (rowIdn < 0 || rowIdn >= N_SCAN) continue;// 避免行编码越界
        /************************************************************/
        /*                    计算当前点所在 列 的索引                 */
        /************************************************************/
        horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI; // 单位变换为 度
        columnIdn = (horizonAngle + ang_bottom_h) / ang_res_h;      // 保留整数（第几列）
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;
        /************************************************************/
        /*                  计算当前点对应的深度信息                 */
        /************************************************************/
        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);    //点到激光雷达坐标系原点的欧式距离
        if (range < imageMinRange) continue;
        /*******************************************************/
        /*           满足以上条件则对该点进行编码               */
        /*******************************************************/
        // 首先: 编码range表格 （FLT_MAX表示无效点）
        // rangeMat 深度图（存放点云的欧氏距离的值，若欧式距离为 FLT_MAX，则表示无效点）
        this->rangeMat.at<float>(rowIdn, columnIdn) = range;
        // 其次: 编码点云
        // 编码为伪2维图像，获得当前点所在2D图像的索引号
        // fullCloud 点云的强度字段存储的是: 【该点云的行列信息】
        index = columnIdn  + rowIdn * Horizon_SCAN;
        thisPoint.intensity = range;
        image_cloud_->points[index] = thisPoint;
    }
}


void WEEDREMOVAL::labelComponents(int row, int col, cv::Mat & labelMat)
{
   bool lineCountFlag[375] = {false};
   // 使用数组模拟队列
   this->queueIndX[0] = row;//行
   this->queueIndY[0] = col;//列
   int queueSize = 1;
   int queueStartInd = 0;// pop 数据的哨兵
   int queueEndInd = 1;// push 数据的哨兵
   // 存储分割的点云
   this->allPushedIndX[0] = row;
   this->allPushedIndY[0] = col;
   int allPushedIndSize = 1;// 记录当前簇的点云数量
        
   while(queueSize > 0)
   {
      // Pop point 提取一个数据
      //模拟queue.top()函数
      int fromIndX = queueIndX[queueStartInd];
      int fromIndY = queueIndY[queueStartInd];
      
      //模拟queue.pop()函数
      ++queueStartInd;
      
      //模拟queue.size()函数
      --queueSize;
      
      // 标记点
      this->labelMat.at<int>(fromIndX, fromIndY) = labelCount;
      // neighbor=[[-1,0];[0,1];[0,-1];[1,0]]
      // 遍历点[fromIndX,fromIndY]边上的四个邻点
      for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter)
      {
         //2.1. 提供有效的邻居点(正确的坐标和未被标记)
         // new index
         int thisIndX = fromIndX + (*iter).first;// 纵轴坐标
         int thisIndY = fromIndY + (*iter).second;// 横轴坐标
         // 避免内存越界
         if (thisIndX < 0 || thisIndX >= N_SCAN)
            continue;
         // 由于雷达水平角度360度相连的特性是个环状的图片，左右连通
         // 调整横轴边界数据是相连的
         if (thisIndY < 0)
            thisIndY = Horizon_SCAN - 1;
         if (thisIndY >= Horizon_SCAN)
            thisIndY = 0;
         // 保证所取点的有效性:必须是未被标记的点
         // labelMat中，-1代表无效点，0代表未进行标记过，其余为其他的标记
         // 如果labelMat已经标记为正整数，则已经聚类完成，不需要再次对该点聚类
         if (this->labelMat.at<int>(thisIndX, thisIndY) != 0)
            continue;
         // 2.2. 计算d1(长边)和d2(短边)
         float d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
         float d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), rangeMat.at<float>(thisIndX, thisIndY));
         // 2.3. 根据所取点的位置设置alpha值
         float alpha, angle;
         if ((*iter).first == 0)
            alpha = segmentAlphaX;
         else
            alpha = segmentAlphaY;
            
         // 通过下面的公式计算这两点之间是否有平面特征，是否是同一类
         // atan2(y,x)的值越大，d1，d2之间的差距越小,越接近，越平坦
         angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));         // 两点连线，与原点组成三角形，此处计算 近点-远点-坐标原点 构成的夹角
         if (angle > segmentTheta)// 此处segmentTheta = 30.0
         {
            //模拟queue.push_back()函数
            queueIndX[queueEndInd] = thisIndX;
            queueIndY[queueEndInd] = thisIndY;
            ++queueEndInd;
            
            ++queueSize;
            // 标记此点与中心点同样的label
            labelMat.at<int>(thisIndX, thisIndY) = labelCount;
            lineCountFlag[thisIndX] = true;
            // 追踪分割的点云
            allPushedIndX[allPushedIndSize] = thisIndX;
            allPushedIndY[allPushedIndSize] = thisIndY;
            ++allPushedIndSize;
         }
      }
   }

   //3. 判断分隔是否有效
/*
获取分割结果之后，我们还需要作进一步的验证，如果所分割的点大于30个，则认为此次分割结果是有效的；如果分割的点数小于 30，大于5，则要作进一步的分析；
如果，分割的点所占据的行数大于3 ，也认为此次分割是有效的。因为像树干这类目标，水平点的分布很少，但是竖直点的分布较多，激光雷达在垂直方向的分辨率又比较低，所以，这个地方的线束阈值设为3 。
这样可以有效排除掉树叶，树枝等特征不稳定的点云。
*/
   bool feasibleSegment = false;
   // 如果分割出的点云个数大于30个则认为是此分割是有效的
   if (allPushedIndSize >= 20)
   {
      feasibleSegment = true;
   }
   // 如果分割的点云数量不满足30且大于最小点数要求(15个点)，则进一步分析
   else if (allPushedIndSize >= segmentValidPointNum)
   {
      // 统计行数
      int lineCount = 0;
      for (size_t i = 0; i < N_SCAN; ++i)
      {
         if (lineCountFlag[i] == true)
         {
            ++lineCount;
         }
      }
      // 如果行数大于3，则认为也是有效的点云分割
      if (lineCount >= segmentValidLineNum)
      {
         feasibleSegment = true;
      }
   }
   // 4. 如果分隔有效, 则更新labelCount; 如果无效, 则标记为噪声
   if (feasibleSegment == true)
   {
      ++labelCount;
   }
   else
   { // segment is invalid, mark these points
      for (size_t i = 0; i < allPushedIndSize; ++i)
      {
         // 标记为999999的是需要舍弃的聚类的点，因为他们的数量小于30个，且竖直方向少于3
         labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
      }
   }
}







































