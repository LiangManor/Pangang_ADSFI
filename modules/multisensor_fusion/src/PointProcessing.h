#ifndef POINT_PROCESSING_H
#define POINT_PROCESSING_H

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>


// 使用 PCL 定义点类型
using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<PointXYZ>;

struct CalibrationData
{
    cv::Mat RT;           // 4x4
    cv::Mat intrinsic;    // 3x3
    cv::Mat distortion;   // 1xN
};

// 定义点处理类
class PointProcessing {
public:
    // 从 yaml 文件加载标定参数
    static bool loadTransformFromYaml(const std::string& configFile);

    static CalibrationData loadCalibrationFromYAML(const std::string& yamlFile,
                                        const std::string& lidar_type);

    // 从二进制文件加载点
    static bool loadPoints(const std::string &filename, PointCloudXYZ::Ptr point_list);

    // 点之间的欧几里得距离
    static float distance(const PointXYZ &p1, const PointXYZ &p2);

    // 转换点到新的坐标系
    static PointXYZ transformPoint(const PointXYZ &p, float tx, float ty, float yaw);
    
    // 坐标变换函数（旋转+平移）
    static PointXYZ rotateAndTranslate(const PointXYZ &point, int model);

    // // 使用最小二乘法拟合多项式 y = f(x)
    // static Eigen::VectorXd fitPolynomial(const PointCloudXYZ::Ptr points, int degree);
    
    // // 使用最小二乘法拟合多项式 x = f(y)
    // static Eigen::VectorXd fitPolynomial_img_lan(const PointCloudXYZ::Ptr points, int degree);

    // 计算点到多项式的最近距离
    // static float pointToPolynomialDistance(const PointXYZ &p, const Eigen::VectorXd &coeffs);
    static float pointToRoadDistance( const PointXYZ &p, const PointCloudXYZ::Ptr &road_point_list);

    // 查找最近点并拟合多项式
    static void findClosestAndFit(const PointCloudXYZ::Ptr point_list, const PointXYZ &target,
                                   float yaw, int model, PointCloudXYZ::Ptr transformed_points);

private:
    static Eigen::Matrix4f transform_model1;
    static Eigen::Matrix4f transform_model2;

};

#endif // POINT_PROCESSING_H
