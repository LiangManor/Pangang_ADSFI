#ifndef GROUND_SEGMENTATION_SEGMENT_H_
#define GROUND_SEGMENTATION_SEGMENT_H_

#include <list>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>

//——————————————————————————组件1————————————————————————————
class Bin {
public:
  struct MinZPoint {//默认构造函数：不接受参数，将z和d都初始化为0。
    MinZPoint()//MinZPoint的无参构造函数
    {
       z = 0.0;
       d = 0,0;
    } 
    MinZPoint( float dTemp,  float zTemp)//MinZPoint的有参构造函数
    {
       d = dTemp;
       z = zTemp;
    }

    float z;
    float d;
  };

  bool has_point_;
  float min_z;
  float min_z_range;

public:

  Bin(): min_z(std::numeric_limits<float>::max()), has_point_(false) {}//构造函数

   void addPoint(const float& d, const float& z)
   {
      has_point_ = true;
      if (z < min_z)
      {
         min_z = z;
         min_z_range = d;
      }
   }

   MinZPoint getMinZPoint()
   {
      MinZPoint point;
      if (has_point_)
      {
         point.z = min_z;
         point.d = min_z_range;
      }
      return point;
   }
   bool hasPoint() {return has_point_;}

};


//——————————————————————————组件2————————————————————————————
class Segment
{
public:
   typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
   typedef std::pair<float, float> LocalLine;
   float min_slope_;
   float max_slope_;
   float max_error_;
   float long_threshold_;
   float max_long_height_;
   float max_start_height_;
   float sensor_height_;
   std::vector<Bin> bins_;
   std::list<std::pair<Bin::MinZPoint, Bin::MinZPoint>> lines_;


public:
//构造函数
Segment( unsigned int n_bins,
        float min_slope,
        float max_slope,
        float max_error,
        float long_threshold,
        float max_long_height,
        float max_start_height,
        float sensor_height)
{
  bins_.resize(n_bins) ;
  min_slope_ = min_slope;
  max_slope_ = max_slope;
  max_error_ = max_error;
  long_threshold_ = long_threshold;
  max_long_height_ = max_long_height;
  max_start_height_ = max_start_height;
  sensor_height_ = sensor_height;
}//构造函数





//拟合当前segment的直线
void fitSegmentLines()
{
   //寻找当前segment的第一个点
   auto   bin_start = bins_.begin();
   while (!bin_start->hasPoint())
   {
      ++  bin_start;
      if (  bin_start == bins_.end())
      return;
   }
   // Fill lines.
   bool is_long_line = false;
   float cur_ground_height = -sensor_height_;
   std::list<Bin::MinZPoint> current_line_points(1,   bin_start->getMinZPoint());
   LocalLine cur_line = std::make_pair(0, 0); //(k,b)
   //
   for (auto   bin_iter =   bin_start + 1;   bin_iter != bins_.end(); ++  bin_iter)
   {
      if (  !bin_iter->hasPoint()) continue;
      Bin::MinZPoint cur_point =   bin_iter->getMinZPoint();
      if (cur_point.d - current_line_points.back().d > long_threshold_) is_long_line = true;
      //else//当拟合线的点数少于2时
      if (current_line_points.size() < 2)
      {
         if (cur_point.d - current_line_points.back().d < long_threshold_ 
             &&
             std::fabs(current_line_points.back().z - cur_ground_height) < max_start_height_
            )
         {
            current_line_points.push_back(cur_point);
         }
         else
         {
            current_line_points.clear();
            current_line_points.push_back(cur_point);
         }
      }
      else
      {
         float expected_z = std::numeric_limits<float>::max();
         if (is_long_line )
         {
            expected_z = cur_line.first * cur_point.d + cur_line.second;
         }
         current_line_points.push_back(cur_point);
         cur_line = fitLocalLine(current_line_points);
         float error = getMaxResidual(current_line_points, cur_line);
         if (   (error > max_error_ )
             || (std::fabs(cur_line.first) > max_slope_ )
             || (current_line_points.size() > 2 && std::fabs(cur_line.first) < min_slope_) 
             || (is_long_line) 
             && (std::fabs(expected_z - cur_point.z) > max_long_height_)
            )
         {
            current_line_points.pop_back();
            if (current_line_points.size() >= 3)
            {
               LocalLine new_line = fitLocalLine(current_line_points);
               lines_.push_back(fitLineEndPts(new_line, current_line_points));
               cur_ground_height = new_line.first * current_line_points.back().d + new_line.second;
            }
            is_long_line = false;
            current_line_points.erase(current_line_points.begin(), --current_line_points.end());
            --bin_iter;
         }
      }
   }
   // Add last line.
   if (current_line_points.size() > 2)
   {
      LocalLine new_line = fitLocalLine(current_line_points);
      lines_.push_back(fitLineEndPts(new_line, current_line_points));
   }
}


//返回当前被拟合的两个端点
std::pair<Bin::MinZPoint, Bin::MinZPoint> fitLineEndPts( LocalLine &local_line,  std::list<Bin::MinZPoint> &line_points)
{
   Line line;
   float first_d = line_points.front().d;
   float first_z = local_line.first * first_d + local_line.second;
   
   float second_d = line_points.back().d;
   float second_z = local_line.first * second_d + local_line.second;
   line.first.z = first_z;
   line.first.d = first_d;
   line.second.z = second_z;
   line.second.d = second_d;
   return line;
}

//返回残差的平方
float getMaxResidual( std::list<Bin::MinZPoint> &points,  LocalLine &line)
{
   float max_error = 0;
   for (auto it = points.begin(); it != points.end(); ++it)
   {
      float residual = (line.first * it->d + line.second) - it->z;
      float error = residual * residual;
   if (error > max_error)
      max_error = error;
   }
   return max_error;
}

//拟合当前直线:返回直线的斜率k和截距b
LocalLine fitLocalLine( std::list<Bin::MinZPoint> &points)
{
   unsigned int n_points = points.size();
   Eigen::MatrixXd X(n_points, 2);
   Eigen::VectorXd Y(n_points);
   unsigned int counter = 0;
   for (auto iter = points.begin(); iter != points.end(); ++iter)
   {
       X(counter, 0) = iter->d;
       X(counter, 1) = 1;
       Y(counter) = iter->z;
       ++counter;
   }
   Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
   LocalLine line_result;
   line_result.first = result(0);
   line_result.second = result(1);
   return line_result;
}


//获取 当前点云的z值  与  当前点云的数值d在拟合直线上对应的z值    的差值
float verticalDistanceToLine( float &d,  float &z)
{
   float kMargin = 0.3; // 点在直线跨度区间可进行地面点判断
   float distance = -1;
   for (auto it = lines_.begin(); it != lines_.end(); ++it)
   {
      if (it->first.d - kMargin < d && it->second.d + kMargin > d)
      {
         float delta_z = it->second.z - it->first.z;
         float delta_d = it->second.d - it->first.d;
         float expected_z = (d - it->first.d) / delta_d * delta_z + it->first.z;
         distance = std::fabs(z - expected_z);
         return distance;
      }
   }
   return distance;
}



};









#endif /* GROUND_SEGMENTATION_SEGMENT_H_ */
