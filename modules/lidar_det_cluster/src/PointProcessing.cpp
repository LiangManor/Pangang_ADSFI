#include "PointProcessing.h"
#include "yaml/haf_yaml.h"
#include <yaml-cpp/yaml.h> // 使用 yaml-cpp 库

// ===== static 变量初始化 =====
Eigen::Matrix4f PointProcessing::transform_model1 = Eigen::Matrix4f::Identity();
Eigen::Matrix4f PointProcessing::transform_model2 = Eigen::Matrix4f::Identity();


// ==================================
// 读取  lidar - lidar 定位到检测
// ==================================
// bool PointProcessing::loadTransformFromYaml(const std::string &filePath)
// {
//     try {
//         Adsfi::HafYamlNode config(filePath);

//         // --------------------------------------------------------------------
//         // 1. 读取 car_id
//         // --------------------------------------------------------------------
//         int car_id = 1;
//         config.GetValue<int>("car_id", car_id);

//         std::string train_key = "trains.train" + std::to_string(car_id);

//         // --------------------------------------------------------------------
//         // 2. 读取 A4 的 4×4 矩阵
//         // --------------------------------------------------------------------
//         std::vector<std::vector<float>> A4_vec;
//         config.GetValue<std::vector<std::vector<float>>>(train_key + ".A4", A4_vec);

//         if (A4_vec.size() != 4 || A4_vec[0].size() != 4) {
//             std::cerr << "[ERROR] A4 matrix must be 4×4!" << std::endl;
//             return false;
//         }

//         for (int i = 0; i < 4; i++)
//             for (int j = 0; j < 4; j++)
//                 transform_model1(i, j) = A4_vec[i][j];

//         // --------------------------------------------------------------------
//         // 3. 读取 B2 的 4×4 矩阵
//         // --------------------------------------------------------------------
//         std::vector<std::vector<float>> B2_vec;
//         config.GetValue<std::vector<std::vector<float>>>(train_key + ".B2", B2_vec);

//         if (B2_vec.size() != 4 || B2_vec[0].size() != 4) {
//             std::cerr << "[ERROR] B2 matrix must be 4×4!" << std::endl;
//             return false;
//         }

//         for (int i = 0; i < 4; i++)
//             for (int j = 0; j < 4; j++)
//                 transform_model2(i, j) = B2_vec[i][j];

//         std::cout << "[INFO] Successfully loaded calibration for train"
//                   << car_id << std::endl;

//         return true;
//     }
//     catch (const std::exception &e) {
//         std::cerr << "[EXCEPTION] loadCalibrationFromYAML: " << e.what() << std::endl;
//         return false;
//     }
// }

bool PointProcessing::loadTransformFromYaml(const std::string &filePath)
{
    try {
        YAML::Node config = YAML::LoadFile(filePath);

        if (!config["car_id"]) {
            std::cerr << "YAML missing car_id" << std::endl;
            return false;
        }

        int carId = config["car_id"].as<int>();  // 读取用户选择的车辆编号

        if (!config["trains"]) {
            std::cerr << "YAML missing trains section" << std::endl;
            return false;
        }

        std::string trainKey = "train" + std::to_string(carId);

        if (!config["trains"][trainKey]) {
            std::cerr << "YAML missing " << trainKey << std::endl;
            return false;
        }

        YAML::Node train = config["trains"][trainKey];

        // 读取 A4
        if (!train["A4"]) {
            std::cerr << "Missing A4 in " << trainKey << std::endl;
            return false;
        }

        YAML::Node A4 = train["A4"];
        if (A4.size() != 4) { 
            std::cerr << "A4 must be 4 rows" << std::endl;
            return false;
        }

        for (int i = 0; i < 4; i++) {
            if (A4[i].size() != 4) {
                std::cerr << "A4 row must have 4 elements" << std::endl;
                return false;
            }
            for (int j = 0; j < 4; j++) {
                transform_model1(i, j) = A4[i][j].as<float>();
            }
        }

        // 读取 B2
        if (!train["B2"]) {
            std::cerr << "Missing B2 in " << trainKey << std::endl;
            return false;
        }

        YAML::Node B2 = train["B2"];
        if (B2.size() != 4) {
            std::cerr << "B2 must be 4 rows" << std::endl;
            return false;
        }

        for (int i = 0; i < 4; i++) {
            if (B2[i].size() != 4) {
                std::cerr << "B2 row must have 4 elements" << std::endl;
                return false;
            }
            for (int j = 0; j < 4; j++) {
                transform_model2(i, j) = B2[i][j].as<float>();
            }
        }

        std::cout << "Loaded calibration for car " << carId << " successfully." << std::endl;

        return true;
    }
    catch (const std::exception &e) {
        std::cerr << "Exception loading YAML: " << e.what() << std::endl;
        return false;
    }
}


// 从二进制文件加载点
bool PointProcessing::loadPoints(const std::string &filename, PointCloudXYZ::Ptr point_list) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return false;
    }

    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    size_t num_points = file_size / (sizeof(float) * 3);

    file.seekg(0, std::ios::beg);
    for (size_t i = 0; i < num_points; ++i) {
        float x, y, z;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        point_list->push_back(PointXYZ(x, y, z));
    }

    file.close();
    return true;
}

// 点之间的欧几里得距离
float PointProcessing::distance(const PointXYZ &p1, const PointXYZ &p2) {
    // return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 转换点到新的坐标系
PointXYZ PointProcessing::transformPoint(const PointXYZ &p, float tx, float ty, float yaw) {
    float cos_a = std::cos(-yaw);
    float sin_a = std::sin(-yaw);
    float dx = p.x - tx;
    float dy = p.y - ty;
    return PointXYZ(cos_a * dx - sin_a * dy, sin_a * dx + cos_a * dy, p.z);
}

// 坐标变换函数（旋转+平移）
PointXYZ PointProcessing::rotateAndTranslate(const PointXYZ &point, int model) {

    const Eigen::Matrix4f& transformation_matrix = (model == 1) ? transform_model1 : transform_model2;

    // 将点转换为齐次坐标
    Eigen::Vector4f homogenous_point(point.x, point.y, 0.0f, 1.0f);

    // 进行旋转平移变换
    Eigen::Vector4f transformed_point = transformation_matrix * homogenous_point;

    // 返回结果，仅保留 x 和 y
    return PointXYZ(transformed_point(0), transformed_point(1), point.z);
}

// 方向拟合并生成旋转矩阵
Eigen::Matrix3f estimateRotationMatrix(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->size() < 2) {
        return Eigen::Matrix3f::Identity(); // 无法拟合方向，返回单位矩阵
    }

    int n = std::min(5, static_cast<int>(cloud->size()));
    Eigen::Vector2f avg_dir(0.0, 0.0);

    for (int i = 1; i < n; ++i) {
        Eigen::Vector2f dir(cloud->points[i].x - cloud->points[0].x,
                            cloud->points[i].y - cloud->points[0].y);
        if (dir.norm() > 1e-6) {
            dir.normalize();
            avg_dir += dir;
        }
    }
    if (avg_dir.norm() < 1e-6) {
        return Eigen::Matrix3f::Identity();
    }
    avg_dir.normalize();

    Eigen::Vector2f forward(1.0, 0.0);
    float angle = std::atan2(avg_dir.y(), avg_dir.x()) - std::atan2(forward.y(), forward.x());

    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
    rotation(0,0) = std::cos(-angle);
    rotation(0,1) = -std::sin(-angle);
    rotation(1,0) = std::sin(-angle);
    rotation(1,1) = std::cos(-angle);

    return rotation;
}

// 应用旋转和平移进行矫正
pcl::PointCloud<pcl::PointXYZ>::Ptr correctTrajectory(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr corrected(new pcl::PointCloud<pcl::PointXYZ>);
    if (original->empty()) return corrected;

    // 第一个点
    Eigen::Vector3f p0 = original->points[0].getVector3fMap();

    // 先把点平移到以 p0 为原点的坐标系，估计旋转（仅 x,y）
    pcl::PointCloud<pcl::PointXYZ>::Ptr translated(new pcl::PointCloud<pcl::PointXYZ>);
    translated->reserve(original->size());
    for (const auto& pt : *original) {
        pcl::PointXYZ tmp;
        tmp.x = pt.x - p0.x();
        tmp.y = pt.y - p0.y();
        tmp.z = pt.z - p0.z();
        translated->push_back(tmp);
    }

    Eigen::Matrix3f rotation = estimateRotationMatrix(translated);

    // 合成平移： p0 - R * p0
    Eigen::Vector3f translation = p0 - rotation * p0;

    // 构造变换
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = rotation;
    transform.translation() = translation;

    pcl::transformPointCloud(*original, *corrected, transform);
    return corrected;
}


// // 应用旋转和平移进行矫正
// pcl::PointCloud<pcl::PointXYZ>::Ptr correctTrajectory(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr corrected(new pcl::PointCloud<pcl::PointXYZ>);

//     if (original->empty()) return corrected;

//     // 平移向量（将第一个点移到原点）
//     Eigen::Vector3f translation = -original->points[0].getVector3fMap();

//     // 方向拟合计算旋转
//     pcl::PointCloud<pcl::PointXYZ>::Ptr translated(new pcl::PointCloud<pcl::PointXYZ>);
//     for (const auto& pt : *original) {
//         pcl::PointXYZ tmp;
//         tmp.getVector3fMap() = pt.getVector3fMap() + translation;
//         translated->push_back(tmp);
//     }

//     Eigen::Matrix3f rotation = estimateRotationMatrix(translated);

//     // 构造变换矩阵
//     Eigen::Affine3f transform = Eigen::Affine3f::Identity();
//     transform.linear() = rotation;
//     transform.translation() = translation;

//     pcl::transformPointCloud(*original, *corrected, transform);
//     return corrected;
// }

// 查找最近点，并转换坐标
void PointProcessing::findClosestAndFit(const PointCloudXYZ::Ptr point_list, 
                                        const PointXYZ &target, float yaw, 
                                        int model, PointCloudXYZ::Ptr transformed_points) {
    if (!point_list || point_list->empty()) return;

    auto it = std::min_element(point_list->points.begin(), point_list->points.end(),
                               [&target](const PointXYZ &p1, const PointXYZ &p2) {
                                   return distance(p1, target) < distance(p2, target);
                               });

    if (it != point_list->points.end()) {
        int start_index = std::distance(point_list->points.begin(), it);
        // const float scale = 0.98828125f;  // 缩放系数
        const float scale = 1.0f;  // 缩放系数

        if (model == 1) {  // 从最近点向终点方向
            for (int i = start_index; i < std::min(static_cast<int>(point_list->points.size()), start_index + 120); i++) {
                PointXYZ transformed = transformPoint(point_list->points[i], target.x, target.y, yaw);
                PointXYZ transformed_A3_A4 = rotateAndTranslate(transformed, model);
                // transformed_A3_A4.z -= point_list->points[start_index].z;
                // // 对坐标进行缩放
                // transformed_A3_A4.x *= scale;
                // transformed_A3_A4.y *= scale;
                // transformed_A3_A4.z *= scale;
                transformed_points->push_back(transformed_A3_A4);
            }
        } else if (model == 2) {  // 从最近点向起点方向
            bool flag = true;
            for (int i = start_index; i >= std::max(0, start_index - 130); i--) {
                PointXYZ transformed = transformPoint(point_list->points[i], target.x, target.y, yaw);
                PointXYZ transformed_A3_B2 = rotateAndTranslate(transformed, model);
                //过滤掉车体占用轨迹点
                if (transformed_A3_B2.x < 0) continue;
                if (flag){
                    flag = false;
                    start_index = i;
                }
                // 求相对于当前车体坐标的轨迹相对 z轴 高度
                transformed_A3_B2.z -= point_list->points[start_index].z;
                // 对坐标进行缩放
                transformed_A3_B2.x *= scale;
                transformed_A3_B2.y *= scale;
                transformed_A3_B2.z *= scale;
                transformed_points->push_back(transformed_A3_B2);
            }
        }
        // //基于标定的旋转平移轨迹纠偏
        transformed_points = correctTrajectory(transformed_points);
    }
}
