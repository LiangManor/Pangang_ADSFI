#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <map>
#include <algorithm>
#include <cmath>

class GroundFilter {
private:
    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    pcl::ExtractIndices<pcl::PointXYZ> extract_;
    
    // 铁路检测参数
    float rail_height_ = 0.2f;          // 默认轨道高度（相对）
    Eigen::Vector3f rail_direction_ = Eigen::Vector3f::UnitX(); // 默认轨道方向(X轴)
    bool enable_rail_mode_ = false;     // 默认关闭铁路模式

    // 新增：地面厚度阈值
    float ground_thickness_ = 0.25f;    // 平面上下 ±0.25m 范围内的点视为地面

public:
    GroundFilter() {
        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setDistanceThreshold(0.4);             //距离RANSAC平面 0.4 距离内的点算是平面点。
        seg_.setMaxIterations(1000);
        // === 约束平面法向量方向（Z轴向上 ±10°） ===
        seg_.setAxis(Eigen::Vector3f(0, 0, 1));
        seg_.setEpsAngle(10.0 * M_PI / 180.0);
    }

    // 启用铁路检测模式
    void enableRailDetection(float rail_height = 1.5f, 
                           const Eigen::Vector3f& direction = Eigen::Vector3f::UnitX()) {
        enable_rail_mode_ = true;
        rail_height_ = rail_height;
        rail_direction_ = direction.normalized();
    }

    void disableRailDetection() {
        enable_rail_mode_ = false;
    }

    // 新增：地面厚度设置
    void setGroundThickness(float thickness) {
        ground_thickness_ = thickness;
    }

    void segment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground) {
        
        if (!enable_rail_mode_) {
            basicSegmentWithThickness(input, ground, no_ground);
        } else {
            railSegment(input, ground, no_ground);
        }
    }

private:
    // ===== 原始地面分割 + 厚度范围过滤 =====
    void basicSegmentWithThickness(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        std::vector<float> zs;      //输出z系列点云
        seg_.setInputCloud(input);
        seg_.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            *no_ground = *input;
            return;
        }

        // 提取平面参数
        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
        float norm = std::sqrt(a * a + b * b + c * c);

        // 基于距离范围重新筛选地面点
        ground->clear();
        no_ground->clear();

        for (const auto& p : *input) {
            // 点到平面距离（带符号）
            float signed_dist = (a * p.x + b * p.y + c * p.z + d) / norm;

            // 如果点在平面上方且高于0.2m，则保留为非地面点
            if (signed_dist >= ground_thickness_) {
                no_ground->push_back(p);
            }
            // 否则认为是地面或地面以下
            else {
                ground->push_back(p);
            }
        }
    }

    // ===== 铁路增强分割 =====
    void railSegment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        
        // 1. 近处（0-30m）普通地面分割
        pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(input);
        pass.setFilterFieldName("x"); 
        pass.setFilterLimits(0, 30);
        pass.filter(*near_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_near(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_near(new pcl::PointCloud<pcl::PointXYZ>);
        basicSegmentWithThickness(near_cloud, ground_near, no_ground_near);
        *ground += *ground_near;

        // 2. 远处（30-60m）铁路增强
        pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setFilterLimits(30, 60);
        pass.filter(*far_cloud);

        if (!far_cloud->empty()) {
            pcl::SACSegmentation<pcl::PointXYZ> rail_seg;
            rail_seg.setOptimizeCoefficients(true);
            rail_seg.setModelType(pcl::SACMODEL_PLANE);
            rail_seg.setMethodType(pcl::SAC_RANSAC);
            rail_seg.setAxis(rail_direction_);
            rail_seg.setEpsAngle(10.0 * M_PI / 180.0);
            rail_seg.setDistanceThreshold(0.3);
            rail_seg.setInputCloud(far_cloud);

            pcl::PointIndices::Ptr rail_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr rail_coeff(new pcl::ModelCoefficients);
            rail_seg.segment(*rail_inliers, *rail_coeff);

            // 直方图峰值法备用
            std::map<float, int> height_hist;
            for (const auto& p : *far_cloud) {
                float rounded_z = std::round(p.z * 10) / 10;
                height_hist[rounded_z]++;
            }
            auto max_peak = *std::max_element(height_hist.begin(), height_hist.end(), 
                [](auto& a, auto& b) { return a.second < b.second; });
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr rail_plane(new pcl::PointCloud<pcl::PointXYZ>);
            if (rail_inliers->indices.size() > max_peak.second) {
                extract_.setInputCloud(far_cloud);
                extract_.setIndices(rail_inliers);
                extract_.filter(*rail_plane);
            } else {
                for (const auto& p : *far_cloud) {
                    if (std::fabs(p.z - max_peak.first) < 0.3) {
                        rail_plane->push_back(p);
                    }
                }
            }
            *ground += *rail_plane;
        }

        // 3. 基于轨道高度过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_ground(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : *ground) {
            if (p.z >= (rail_height_ - 0.15)) {
                final_ground->push_back(p);
            }
        }
        *ground = *final_ground;
    }
};
