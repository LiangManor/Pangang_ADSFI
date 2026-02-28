/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  camera检测demo源文件
 */

#include "camera_detection_app.h"
#include <chrono>
#include <thread>
#include <array>
#include <vector>
#include <algorithm>
#include <condition_variable>
#include <sys/time.h>
#include <csignal>
#include "imageprocess/image.h"
#include "imageprocess/convert_color.h"
#include "imageprocess/resize.h"
#include "core/core.h"
#include "core/timer.h"
#include "object/object.h"
#include "dnn/dnn.h"
#include "core/logger.h"
#include "camera_det_util.h"
#include "image_nn_inference.h"
#include <publisher.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream> 
#include <iostream>
#include <numeric>
#include <tuple>
#include <cstring>

#include <mviz/mviz.h>
#include <mviz_msgs/ImageFreespace.h>
#include <mviz_msgs/SemanticSegmentation.h>

using Adsfi::Haf3dDetectionOut;
using Adsfi::ImageData;
using Adsfi::ImageFrame;
using namespace CameraDetection;
using namespace Adsfi;
//using namespace mdc::visual;

mdc::visual::Publisher objectArrayPub {};
mviz_msgs::SemanticSegmentation semanticSegmentation{};
mviz::Publisher semanticSegmentationPub{};
mviz::NodeHandle nodeHandle{};

std::shared_ptr<Adsfi::ImageData> img2_static;

CameraDetectionApp::~CameraDetectionApp()
{
    Stop();
    HAF_LOG_INFO << "CameraDetectionApp destrutoring. Thread pool size: " << threadPool.size();
    for (auto& t : threadPool) {
        if (t.joinable()) {
            t.join();
        }
    }
    HAF_LOG_INFO << "Begin to reset ACL device";
    // 总体释放NN资源
    if (HafRelease(context) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destructoring NN device failed.";
    }
}

bool CameraDetectionApp::ReadConfig()
{
    auto config = Adsfi::HafYamlNode(configFile);

    // 使用此宏，必须有前置步骤  auto config = Adsfi::HafYamlNode(configFile);
#define READ_CONFIG_ERR_RETURN(str, var)                                       \
    do {                                                                       \
        HAF_LOG_INFO << "Begin to read '" << (str) << "' from config file.";   \
        if (!config.GetValue<decltype(var)>((str), (var))) {                   \
            HAF_LOG_ERROR << "Cannot find '" << (str) << "' in Config File!!"; \
            return false;                                                      \
        }                                                                      \
    } while (false)

    READ_CONFIG_ERR_RETURN("imageWidth", imageWidth);
    READ_CONFIG_ERR_RETURN("imageHeight", imageHeight);
    READ_CONFIG_ERR_RETURN("model", this->model);
    instanceID1 = config["instanceID1"].as<uint32_t>();
    instanceID2 = config["instanceID2"].as<uint32_t>();

    std::cout << "Read config result. Image width:  " << imageWidth << "  height: "
    << imageHeight << "  instanceID1: " << instanceID1 << "  instanceID2:  " << instanceID2 << std::endl;
    return true;
}

/**
 * @brief 捕捉 Ctrl+C 信号
 *
 */
static void IntSigHandler(int32_t num)
{
    StopUtility::Instance().Stop();
    HAF_LOG_WARN << "=====Signal Interactive attention received. signal: " << num << " =====";
    return;
}

HafStatus CameraDetectionApp::Initialize()
{
    if (camDetBase.Init() != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Init detection frameword failed.";
        return HAF_ERROR;
    }
    signal(SIGINT, IntSigHandler);
    if (HafInitialize(context, contextParam) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Initialize NN device failed.";
        return HAF_ERROR;
    }
    if (!ReadConfig()) {
        HAF_LOG_ERROR << "Read config file failed.";
        return HAF_ERROR;
    }

    this->receiver.initSock();
    objectArrayPub = mdc::visual::Publisher::Advertise<mdc::visual::ObjectArray>("camera_det_topic");
    semanticSegmentationPub = nodeHandle.Advertise<mviz_msgs::SemanticSegmentation>("image_semanticsegmentation");
    bool connect_state = mdc::visual::Connect();
    // mviz::NodeHandle::Connect();
    // std::cout << "create_det_topic!!!!" << connect_state << "----------------" << std::endl;

    return HAF_SUCCESS;
}

/*
定时删除存放的检测到的障碍物数据
*/
void CameraDetectionApp::cleanPngDirectoryThread()
{
    using namespace std::chrono_literals;
    const std::string dirPath{"have_obj_data"};
    size_t maxFiles = 5000;


    while (true) 
    {
        try {
            // ---- 1. 等待 30 分钟 ----
            std::this_thread::sleep_for(30min);

            // ---- 2. 检查目录是否存在 ----
            if (!std::filesystem::exists(dirPath)) {
                std::cout << "[PNG Cleaner] Directory not found: " << dirPath << std::endl;
                continue;
            }

            std::vector<std::filesystem::directory_entry> pnfFiles;

            // ---- 3. 读取所有 .png 文件 ----
            for (auto& entry : std::filesystem::directory_iterator(dirPath)) {
                if (entry.is_regular_file() && entry.path().extension() == ".png") {
                    pnfFiles.push_back(entry);
                }
            }

            size_t fileCount = pnfFiles.size();
            if (fileCount <= maxFiles) {
                // 文件数量正常
                std::cout << "[PNG Cleaner] File count: " << fileCount << " (OK)" << std::endl;
                continue;
            }

            // ---- 4. 按修改时间排序 → 最早的排前面 ----
            std::sort(pnfFiles.begin(), pnfFiles.end(),
                [](auto& a, auto& b) {
                    return std::filesystem::last_write_time(a) <
                           std::filesystem::last_write_time(b);
                });

            // ---- 5. 需要删除的数量 ----
            size_t toDelete = fileCount - maxFiles;

            std::cout << "[PNG Cleaner] Found " << fileCount
                      << " files, deleting " << toDelete
                      << " oldest files..." << std::endl;

            // ---- 6. 删除最老的文件 ----
            for (size_t i = 0; i < toDelete; i++) {
                try {
                    std::filesystem::remove(pnfFiles[i]);
                } catch (std::exception& e) {
                    std::cerr << "[PNG Cleaner] Failed to delete "
                              << pnfFiles[i].path() << ": " << e.what() << std::endl;
                }
            }

            std::cout << "[PNG Cleaner] Cleanup completed. Remaining: "
                      << (fileCount - toDelete) << std::endl;

        } catch (std::exception& e) {
            std::cerr << "[PNG Cleaner] Exception: " << e.what() << std::endl;
        }
    }
}

void CameraDetectionApp::Stop()
{
    StopUtility::Instance().Stop();
    camDetBase.Stop();
    return;
}

bool CameraDetectionApp::IsStop() const
{
    return StopUtility::Instance().IsStop() || camDetBase.IsStop();
}

void CameraDetectionApp::ShowImageInfo(std::shared_ptr<ImageFrameV2> data) const
{
    const auto width = data->data.width;
    const auto height = data->data.height;
    const auto frameId = data->frameID;
    const auto dataSize = data->data.dataSize;
    const auto type = data->data.imageType;
    const auto seq = data->data.seq;

    timeval now;
    gettimeofday(&now, NULL);
    const auto srcTimeNsec = data->timestamp.nsec;
    const auto srcTimeSec = data->timestamp.sec;
    const auto timeCost = (now.tv_sec - srcTimeSec) * 1000U * 1000U + (now.tv_usec - srcTimeNsec / 1000U);

    HAF_LOG_INFO << "Camera detection demo. image received. Image info. FrameId: " << frameId << " seq: " << seq
                 << " width: " << width << " height: " << height << " data size: " << dataSize
                 << " Image type: " << type << " Time costed from generated(μs): " << timeCost;
    return;
}

// ////////////////////////////////////////////////
// // Function to save RGB image
// void saveRGBAsImage(const uint8_t* rgbData, int width, int height, size_t dataSize, const std::string& outputFilename) {
// // Create an empty Mat with the same size as the input image
// cv::Mat rgbImage(height, width, CV_8UC3, (void*)rgbData);

// // Save the RGB image as a PNG or JPEG file
// cv::imwrite(outputFilename, rgbImage);
// }
// ////////////////////////////////////////////////
// Function to save RGB image in time_name
void saveRGBAsImage(const uint8_t* rgbData, int width, int height, size_t dataSize)
{
    static std::chrono::steady_clock::time_point lastSaveTime = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    // static std::mutex saveMutex;   // 多线程安全
    // std::lock_guard<std::mutex> lock(saveMutex);

    // ---- 1. 判断是否小于 2 秒 ----
    auto now = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - lastSaveTime);
    if (diff.count() < 2) {
        // 2秒内保存过，不再保存
        return;
    }

    lastSaveTime = now; // 更新保存时间

    // ---- 检查并创建目录 ----
    std::string dirName = "have_obj_data";
    if (!std::filesystem::exists(dirName)) {
        std::filesystem::create_directories(dirName);
        std::cout << "Created directory: " << dirName << std::endl;
    }

    // ---- 2. 生成文件名 年_月_日_时_分_秒.png ----
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tmTime;
#ifdef _WIN32
    localtime_s(&tmTime, &t);
#else
    localtime_r(&t, &tmTime);
#endif

    char filename[256];
    snprintf(filename, sizeof(filename),
             "%s/%04d_%02d_%02d_%02d_%02d_%02d.png",
             dirName.c_str(),
             tmTime.tm_year + 1900,
             tmTime.tm_mon + 1,
             tmTime.tm_mday,
             tmTime.tm_hour,
             tmTime.tm_min,
             tmTime.tm_sec
    );

    std::string outputFilename(filename);

    // ---- 3. 直接包装为 Mat 并保存 ----
    cv::Mat image(height, width, CV_8UC3);
    cv::cvtColor(cv::Mat(height, width, CV_8UC3, (void*)rgbData),
             image, cv::COLOR_RGB2BGR);

    cv::imwrite(outputFilename, image);

    std::cout << "Saved: " << outputFilename << std::endl;
}

// ////////////////////////////////////////////////

/**
 * @brief 图片输入到网络前的预处理环节
 *
 */
bool CameraDetectionApp::PrepareImage4Model(
    CameraDetection::ImageNnInference& nnInfer, ImageData &image, ImageData& resizeOutImage)
{
    Timer t;
    // image.imageType = HAF_IMAGE_YUV420SP_NV12_UINT8;        //海康
    image.imageType = HAF_IMAGE_YUV422SP_UINT8;          //森云
    image.bufferType = HAF_BUFFER_MBUF;

    // 颜色空间转换 yuv420sp12 --> rgb 和 图像resize
    if (!nnInfer.ImageResize(image, resizeOutImage)) {
        HAF_LOG_ERROR << "Resize input image failed!";
        return false;
    }

    ////////////////////////////////////////////////
    // saveRGBAsImage(resizeOutImage.rawData, resizeOutImage.width, resizeOutImage.height, resizeOutImage.dataSize,  "resizeOutImage_rgb.png");

    ////////////////////////////////////////////////

    // 输出的 resizeOutImage ，image 存在于DVPP的buffer中
    // // std::cout << "++++++++++++ image 2 rgb and resize success,image.size: ++++++++++++++" << std::endl;
    HAF_LOG_DEBUG << "After resize. time cost(μs): " << t.Toc();
    return true;
}

/**--------------------------------------------------------------------------------------------------------------------------**/
/********************************************       Add Codel Main start       ************************************************/
// 在文件头部或函数内部添加Box结构体定义
struct Box {
float x1, y1, x2, y2;
float score;
int class_id;
};
void CameraDetectionApp::PostProcess_v8(int32_t oriHeight, int32_t oriWidth,
                    const float32_t* resultPtr_0,
                    std::list<Haf3dDetectionOut<float>>& detectionOut3d) {
const float conf_threshold = 0.5f;
const float iou_threshold = 0.4f;
const float pad_w = 1.0f;
const float pad_h = 1.0f;

const int num_classes = 80;      // 你的模型类别数
const int c = 4 + num_classes;  //
const int n = 8400;             // anchors

// 反 letterbox 的缩放比
std::vector<float> ratio = {
    static_cast<float>(modelW) / static_cast<float>(ImageNnInference::crop_area[0].bottom),
    static_cast<float>(modelH) / static_cast<float>(oriHeight)};

std::vector<Box> boxes;
boxes.reserve(256);

// --------- Step 1: 遍历每个 anchor ---------
for (int i = 0; i < n; ++i) {
    // bbox
    float cx = resultPtr_0[0 * n + i];
    float cy = resultPtr_0[1 * n + i];
    float w = resultPtr_0[2 * n + i];
    float h = resultPtr_0[3 * n + i];

    // classes
    float best_score = 0.0f;
    int best_id = -1;
    for (int cls = 0; cls < num_classes; ++cls) {
    float score = resultPtr_0[(4 + cls) * n + i];
    if (score > best_score) {
        best_score = score;
        best_id = cls;
    }
    }

    // --------- Step 3: cxcywh -> xyxy ---------
    if (best_score > conf_threshold) {
    Box b;
    b.x1 = cx - w * 0.5f;
    b.y1 = cy - h * 0.5f;
    b.x2 = cx + w * 0.5f;
    b.y2 = cy + h * 0.5f;
    b.score = best_score;
    b.class_id = best_id;
    boxes.emplace_back(b);

    // // // Debug 输出
    // std::cout << "Anchor " << i << " box: " << cx << "," << cy << "," << w
    //             << "," << h << " score=" << best_score << " class=" << best_id
    //             << std::endl;
    }
}

// --------- Step 2: NMS ---------
std::vector<cv::Rect> rects;
std::vector<float> scores;
rects.reserve(boxes.size());
scores.reserve(boxes.size());
for (const auto& b : boxes) {
    rects.emplace_back(
        cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1)),
        cv::Point(static_cast<int>(b.x2), static_cast<int>(b.y2)));
    scores.emplace_back(b.score);
}

std::vector<int> indices;
cv::dnn::NMSBoxes(rects, scores, conf_threshold, iou_threshold, indices);

std::vector<Box> nms_boxes;
for (int idx : indices) nms_boxes.emplace_back(boxes[idx]);

detectionOut3d.clear();
if (nms_boxes.empty()) return;


// --------- Step 4: 反缩放回原图 ---------
for (auto& b : nms_boxes) {
    b.x1 = (b.x1 - pad_w) / ratio[0];
    b.x1 = b.x1 + ImageNnInference::crop_area[0].left;
    b.y1 = (b.y1 - pad_h) / ratio[1];
    b.x2 = (b.x2 - pad_w) / ratio[0];
    b.x2 = b.x2 + ImageNnInference::crop_area[0].left;
    b.y2 = (b.y2 - pad_h) / ratio[1];
}

// --------- Step 5: 裁边 ---------
for (auto& b : nms_boxes) {
    b.x1 = std::clamp(b.x1, static_cast<float>(ImageNnInference::crop_area[0].left), static_cast<float>(ImageNnInference::crop_area[0].right));
    b.y1 = std::clamp(b.y1, 0.0f, static_cast<float>(oriHeight));
    b.x2 = std::clamp(b.x2, static_cast<float>(ImageNnInference::crop_area[0].left), static_cast<float>(ImageNnInference::crop_area[0].right));
    b.y2 = std::clamp(b.y2, 0.0f, static_cast<float>(oriHeight));
}

// --------- Step 6: 填充结果 ---------
for (const auto& b : nms_boxes) {
    Haf3dDetectionOut<float> out2d;
    out2d.rect.center.x = 0.5f * (b.x1 + b.x2);
    out2d.rect.center.y = 0.5f * (b.y1 + b.y2);
    out2d.rect.size.x = std::fabs(b.x2 - b.x1);
    out2d.rect.size.y = std::fabs(b.y2 - b.y1);
    out2d.cls = b.class_id;
    out2d.confidence = b.score;
    detectionOut3d.push_back(out2d);
}
}


/**--------------------------------------------------------------------------------------------------------------------------**/

float32_t* read_binary_file(const std::string& file_path, size_t expected_size) {
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + file_path);
    }

    float32_t* data = new float32_t[expected_size];
    file.read(reinterpret_cast<char*>(data), expected_size * sizeof(float32_t));
    file.close();
    return data;
}

bool CameraDetectionApp::PostPerception(CameraDetection::ImageNnInference& nnInfer, int32_t oriHeight, int32_t oriWidth,
    std::list<Haf3dDetectionOut<float>>& detection)
{
    // 解析NN结果
    void* outBuffer{};
    size_t bufferSize{};    //(1,84,8400)
    size_t index = 0;
    if (nnInfer.DNNModelGetOutputBuffer(index, &outBuffer, bufferSize) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "NN get output buffer failed! index: " << index;
        return false;
    }
    float32_t* boxNumPrt_0 = static_cast<float32_t*>(outBuffer);
    // HAF_LOG_INFO << "---------------bufferSize0 is num: ---------------" << bufferSize;

    // void* outBuffer_1{};
    // size_t bufferSize_1{};      //(1,32,160,160)
    // index = 1;
    // if (nnInfer.DNNModelGetOutputBuffer(index, &outBuffer_1, bufferSize_1) != HAF_SUCCESS) {
    //     HAF_LOG_ERROR << "NN get output buffer failed! index: " << index;
    //     return false;
    // }
    // float32_t* boxNumPrt_1 = static_cast<float32_t*>(outBuffer_1);
    // HAF_LOG_INFO << "---------------bufferSize1 is num: ---------------" << bufferSize_1;

    // 开始进行后处理过程
    // PostProcess(boxNum, oriHeight, oriWidth, boxNumPrt, detection);
    PostProcess_v8(oriHeight, oriWidth, boxNumPrt_0, detection);

/*************************************************************************************************************/
/*
直接加载 Python 模型推理数据，验证后处理过程
*/

    // size_t size_0 = 1 * 116 * 8400;
    // size_t size_1 = 1 * 32 * 160 * 160;

    // // 读取二进制文件
    // float32_t* result_0 = read_binary_file("/home/sshuser/src/result_0.bin", size_0);
    // float32_t* result_1 = read_binary_file("/home/sshuser/src/result_1.bin", size_1);
    // PostProcess_v8(oriHeight, oriWidth, result_0, result_1, detection);

/*************************************************************************************************************/
    return true;
}


/**
 * @brief 给神经网络设置输入
 *
 */
bool CameraDetectionApp::ModelUpdateInput(CameraDetection::ImageNnInference& nnInfer, ImageData& image)
{
    // constexpr uint32_t imgInfoSize{4U};
    // 更新NN输入
    size_t index = 0U;
    constexpr int32_t RGB_SCALE = 3;
    if (nnInfer.DNNModelUpdateInput(image.rawData, image.width * image.height * RGB_SCALE, index) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "NN add input image failed! Input idx: " << index;
        return false;
    }
    return true;
}

/**
 * @brief 对图片进行推理。过程包括准备图片，准备输入
 *
 */
bool CameraDetectionApp::ImageInference(CameraDetection::ImageNnInference& nnInfer,
    std::shared_ptr<Adsfi::ImageData> &img2, std::list<Haf3dDetectionOut<float>>& detection)
{
    ImageData resizeOutImage;
    if (!PrepareImage4Model(nnInfer, *img2, resizeOutImage)) {
        HAF_LOG_ERROR << "Image preprocess failed!";
        return false;
    }
    if (!ModelUpdateInput(nnInfer, resizeOutImage)) {
        HAF_LOG_ERROR << "NN add model input failed.";
        return false;
    }
    if (nnInfer.DNNModelProcess() != HAF_SUCCESS) {
        HAF_LOG_ERROR << "NN model inference failed!";
        return false;
    }
    // NN结果解析
    if (!PostPerception(nnInfer, img2->height, img2->width, detection)) {
        HAF_LOG_ERROR << "Parsing nn result failed!";
        return false;
    }
    if (have_obj > 0){
        saveRGBAsImage(resizeOutImage.rawData, resizeOutImage.width, resizeOutImage.height, resizeOutImage.dataSize);
    }
    return true;
}

bool CameraDetectionApp::PrepareDvppMemory(CameraDetection::ImageNnInference& nnInfer)
{
    // 用于图片reisze的内存，在整个推理的生命周期里面，都是固定的
    // 每个单独的推理线程，有自己独立的DVPP内存准备
    // 只需要在初始化时申请一次，在程序结束运行时销毁
    ImageData input;
    input.width = imageWidth;
    input.height = imageHeight;

    // input.imageType = HAF_IMAGE_YUV420SP_NV12_UINT8;    //海康  type 2
    input.imageType = HAF_IMAGE_YUV422SP_UINT8;      //森云  type 10

    ImageData output;
    output.width = modelW;
    output.height = modelH;

    if (!nnInfer.PrepareDvppMemory(input, output)) {
        HAF_LOG_ERROR << "Prepare DVPP memory failed.";
        return false;
    }
    return true;
}
// //////////////////////////////////////////
// Function to convert YUV422 UYVY to BGR (OpenCV format) and save it
void saveYUV422UYVYAsImage(const uint8_t* yuvData, int width, int height, size_t dataSize, const std::string& outputFilename) {
    // Create an empty Mat with the same size as the input image
    cv::Mat yuv422uyvyImage(height, width, CV_8UC2, (void*)yuvData);

    // Convert to BGR format
    cv::Mat bgrImage;
    cv::cvtColor(yuv422uyvyImage, bgrImage, cv::COLOR_YUV2BGR_UYVY);

    // Save the BGR image as a PNG or JPEG file
    cv::imwrite(outputFilename, bgrImage);
}
// //////////////////////////////////////////

//****************************** main() ********************************//
void CameraDetectionApp::ObjectDetectionThread()
{
    // uint32_t instanceId = 1;
    const auto channelMax = 255U;
    const auto channelId = static_cast<int32_t>(instanceId % channelMax);
    const auto groupId = 0U;

    // 在线程的开始，初始化神经网络推理和resize所需要的相关资源
    CameraDetection::ImageNnInference nnInfer(modelFile, channelId, groupId);
    if (!PrepareDvppMemory(nnInfer)) {
        HAF_LOG_ERROR << "Prepare DVPP memory failed.";
        Stop();
    }
    if (!nnInfer.Initialize()) {
        HAF_LOG_ERROR << "NN inference initialize failed. idx: " << instanceId;
        Stop();
        return;
    }
    //         /*----------------------------   计时   -------------------------------*/
    // std::chrono::high_resolution_clock::time_point start_time_all, end_time_all;        //计算耗时
    // std::chrono::milliseconds elapsed_ms;
    // /*----------------------------   计时   -------------------------------*/
    // 初始化结束。开始处理数据流。
    while (!IsStop()) {
        // std::cout << "++++++++++++ While-start ++++++++++++++"<<std::endl;

        //判断当前线程是否是车头朝向的摄像头
        if(model == 1)
        {
            instanceId = instanceID1;
        }
        else if(model == 2)
        {
            instanceId = instanceID2;
        }
        else
        {
            auto data = std::make_shared<Haf3dDetectionOutArray<float32_t>>();
            data->frameID = std::to_string(this->model);
            data->seq = 0;
            if (camDetBase.SendObject(data, camDetBase.GetResultObjInsIdx()) != HAF_SUCCESS) {
                HAF_LOG_ERROR << "SendObject failed. PROGRAM WILL STOP";
                Stop();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            continue;
        }
        
        // 等待最新数据过来。阻塞并等待。
        // std::cout<<"instanceId ********************************* = "<< instanceId <<std::endl;
        auto img2 = camDetBase.GetMbufImage(instanceId);
        if (img2 == nullptr) {
            HAF_LOG_INFO << "Image data pointer is nullptr. idx: " << instanceId;
            continue;
        }

        img2_static = img2;
        const auto frameId = img2->frameID;
        const auto seq = img2->seq;
        // HAF_LOG_INFO << "------------ Got image in inference thread. frameid&seq: ---------- " << frameId << " " << seq;
        Timer t;

        std::list<Haf3dDetectionOut<float32_t> > detection;
        if (!ImageInference(nnInfer, img2, detection)) {
            HAF_LOG_FATAL << "Inference image failed. PROGRAM WILL STOP instance id: " << instanceId;
            Stop();
            break;
        }
        std::cout << "障碍物总数量 : " << detection.size() << std::endl;
        auto data = std::make_shared<Haf3dDetectionOutArray<float32_t>>();
        data->detectionOut3d = detection;
        data->frameID = img2->frameID;
        data->seq = img2->seq;
        data->timestamp = img2->timestamp;
        if (camDetBase.SendObject(data, camDetBase.GetResultObjInsIdx()) != HAF_SUCCESS) {
            HAF_LOG_ERROR << "SendObject failed. PROGRAM WILL STOP";
            Stop();
            break;
        }
// /------------------------------------------------------------------------------/
// /***************************  rostopic 可视化  *********************************/
// /------------------------------------------------------------------------------/
        mdc::visual::ObjectArray objectArray = {};
        objectArray.header.seq = img2->seq;
        objectArray.header.frameId = img2->frameID;
        objectArray.header.stamp.sec = img2->timestamp.sec;
        objectArray.header.stamp.nsec = img2->timestamp.nsec;
        for(auto det : detection) {
            mdc::visual::Object object;
            object.objectId = det.objectID;
            object.classification = det.cls;
            object.classificationConfidence = det.confidence;
            object.boxImage.x = det.rect.center.x - 0.5f * det.rect.size.x;
            object.boxImage.y = det.rect.center.y - 0.5f * det.rect.size.y;
            object.boxImage.width = det.rect.size.x;
            object.boxImage.height = det.rect.size.y;
            object.textDisplay = "Any Text";
            objectArray.objectList.emplace_back(object);

        }

        //ROI 区域显示
        mdc::visual::Object object;
        object.objectId = 1;
        object.classification = 7;
        object.classificationConfidence = 1;
        object.boxImage.x = ImageNnInference::crop_area[0].left;
        object.boxImage.y = 1;
        object.boxImage.width = ImageNnInference::crop_area[0].bottom;
        object.boxImage.height = ImageNnInference::crop_area[0].bottom;
        object.textDisplay = "Any Text";
        objectArray.objectList.emplace_back(object);

        bool P_state = objectArrayPub.Publish(objectArray);
    }
    return;
}

/**
 * @brief 数据处理主入口。这里会创建多个推理线程，每个线程单独处理一路instance id发送过来的图像。
 *
 */
void CameraDetectionApp::ProcessEntrance()
{
    // // 此处只是负责启动线程，等待线程结束的工作在析构函数中
    // for (const auto instanceId : camDetBase.GetImageInsIdx()) {
    //     // 根据实际情况，将instance id和处理线程关联到一起。
    //     threadPool.emplace_back(&CameraDetectionApp::ObjectDetectionThread, this, instanceId);
    // }

    threadPool.emplace_back(&CameraDetectionApp::ObjectDetectionThread, this);
    // threadPool.emplace_back(&CameraDetectionApp::getDirect, this);
    // threadPool.emplace_back(&CameraDetectionApp::cleanPngDirectoryThread, this);
    return;
}


void CameraDetectionApp::getDirect()
{
    while(!IsStop())
    {
        auto result = receiver.receiveMessage();
        if (result[0] != -1) {
            this->model = result[0];
            have_obj = result[3];
            std::cout << "Real-time update - model: " << this->model << std::endl;
        }
        
        // 处理其他任务或短暂休息
        std::this_thread::sleep_for(std::chrono::microseconds(100)); // 100微秒
    }
}
