/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  traffic light检测 sample cpp文件
 */
#include "tl_perception.h"
#include "core/core.h"
#include "object/object.h"
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
// #include "camera_det_util.h"
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

using namespace std;
using namespace Adsfi;
using Adsfi::HafTlDetectionOutArray;
using Adsfi::ImageData;
using Adsfi::ImageFrame;
// using namespace TlDet;

TlPerception::~TlPerception()
{
    for (auto& it : pool) {
        if (it.joinable()) {
            it.join();
        }
    }
}



// Adsfi::HafStatus TlPerception::Initialize()
// {
//     if (traDetBase.Init() != HAF_SUCCESS) {
//         HAF_LOG_ERROR << "Init detection frameword failed.";
//         return HAF_ERROR;
//     }
//     return HAF_SUCCESS;
// }


/**
 * @brief 图片输入到网络前的预处理环节
 *
 */
bool TlPerception::PrepareImage4Model(
    CameraDetection::ImageNnInference& nnInfer, ImageData &image, ImageData& resizeOutImage)
{
    // image.imageType = HAF_IMAGE_YUV420SP_NV12_ProcessUINT8;        //海康
    image.imageType = HAF_IMAGE_YUV422SP_UINT8;          //森云
    image.bufferType = HAF_BUFFER_MBUF;

    // 颜色空间转换 yuv420sp12 --> rgb 和 图像resize
    if (!nnInfer.ImageResize(image, resizeOutImage)) {
        HAF_LOG_ERROR << "Resize input image failed!";
        return false;
    }

    return true;
}



bool TlPerception::PostPerception(CameraDetection::ImageNnInference& nnInfer, int32_t oriHeight, int32_t oriWidth, std::int8_t& detection)
{
    // 解析NN结果
    void* outBuffer{};
    size_t bufferSize{};
    size_t index = 0;  // 确保索引为0
    if (nnInfer.DNNModelGetOutputBuffer(index, &outBuffer, bufferSize) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "NN get output buffer failed! index: " << index;
        return false;
    }

    std::cout << "********************************  this PostProcess   !!***********" << std::endl;

    // constexpr int32_t boxNum = 25200;
    // 开始进行后处理过程
    PostProcess(oriHeight, oriWidth, static_cast<const float32_t*>(outBuffer), detection);

    // 在这里立即返回，并打印检测值
    return true;
}

void TlPerception::PostProcess(int32_t oriHeight, int32_t oriWidth, const float32_t* resultPtr, std::int8_t& detection)
{

    // 解析推理结果，解析过程因不同模型的输出不同而不同
    constexpr int32_t boxNum = 25200;
    constexpr int32_t midDiv = 2;
    constexpr int32_t dataNum = 6;
    constexpr int32_t xminOffset = 0;
    constexpr int32_t yminOffset = 1;
    constexpr int32_t xmaxOffset = 2;
    constexpr int32_t ymaxOffset = 3;
    constexpr int32_t scoreOffset = 4;
    constexpr int32_t labelOffset = 5;
    constexpr float32_t threshold = 0.4;
    // xcenter, ycenter, W, H, score, class
    float32_t boxes[boxNum][dataNum] = {0.0};
    float32_t wScale = static_cast<float32_t>(oriWidth) / modelW;
    float32_t hScale = static_cast<float32_t>(oriHeight) / modelH; 
     int8_t traffic_light[3] = {0,0,0,} ;
     int8_t max_num_cls = -1;
     detection = 1;
     // 0：绿色 1：黄色 2：红色 3：黑色
     int8_t  index_light = 1;
    HAF_LOG_INFO << "CLASS CONFIDENCE XMIN YMIN XMAX YMAX";

    for (int32_t i = 0; i < boxNum; i++) {
        if (resultPtr[scoreOffset * boxNum + i] > threshold) {
        
            if  ( resultPtr[labelOffset * boxNum + i] ==0 ){ traffic_light[0] ++; }
           else if  ( resultPtr[labelOffset * boxNum + i] ==1 ){ traffic_light[1] ++; }
            else if  ( resultPtr[labelOffset * boxNum + i] ==2 ){traffic_light[2] ++; }
                        // else if  ( resultPtr[labelOffset * boxNum + i] ==3 ){traffic_light[3] ++; }

            std::cout <<  " +++++++++++++++++++++++++++++++++   红绿灯种类是：   " << resultPtr[labelOffset * boxNum + i] + 1 <<std::endl;
        }
    }

    for(int i = 0 ; i < 4 ; i ++ ){
        if ( traffic_light[i]  > max_num_cls){
            index_light = i;
            max_num_cls = traffic_light[i];
        }

        if (max_num_cls > 2){
            detection = index_light + 1;
        }

    }
std::cout << "Final detection value in PostProcess: " << static_cast<int>(detection) << std::endl;

}


/**
 * @brief 给神经网络设置输入
 *
 */
bool TlPerception::ModelUpdateInput(CameraDetection::ImageNnInference& nnInfer, ImageData& image)
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
bool TlPerception::ImageInference(CameraDetection::ImageNnInference& nnInfer,
    std::shared_ptr<Adsfi::ImageData> &img2, std::int8_t& detection)
{
    ImageData resizeOutImage;
    if (!PrepareImage4Model(nnInfer, *img2, resizeOutImage)) {
        HAF_LOG_ERROR << "Image preprocess failed!";
        return false;
    }
    // std::cout <<  " +++++++++++++++++++++++++++++++++   PrepareImage4Model   "  <<std::endl;
    if (!ModelUpdateInput(nnInfer, resizeOutImage)) {
        HAF_LOG_ERROR << "NN add model input failed.";
        return false;
    }
    // std::cout <<  " +++++++++++++++++++++++++++++++++   ModelUpdateInput   "  <<std::endl;
    if (nnInfer.DNNModelProcess() != HAF_SUCCESS) {
        HAF_LOG_ERROR << "NN model inference failed!";
        return false;
    }
    // std::cout <<  " +++++++++++++++++++++++++++++++++   DNNModelProcess   "  <<std::endl;
    // NN结果解析
    if (!PostPerception(nnInfer, img2->height, img2->width, detection)) {
        HAF_LOG_ERROR << "Parsing nn result failed!";
        return false;
    }
    // std::cout <<  " +++++++++++++++++++++++++++++++++   PostPerception   "  <<std::endl;
    return true;
}

bool TlPerception::PrepareDvppMemory(CameraDetection::ImageNnInference& nnInfer)
{
    // 用于图片reisze的内存，在整个推理的生命周期里面，都是固定的
    // 每个单独的推理线程，有自己独立的DVPP内存准备
    // 只需要在初始化时申请一次，在程序结束运行时销毁
    ImageData input;
    input.width = 1920;
    input.height = 1080;

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


void TlPerception::Process()
{
    pool.push_back(std::thread(&TlPerception::PrintImage, this));
    return;
}


// 转换函数
Adsfi::ImageData ConvertImageDataV2ToImageData(const Adsfi::ImageDataV2& src) {
    Adsfi::ImageData dst;

    // 1. 直接拷贝的字段
    dst.width = src.width;
    dst.height = src.height;
    dst.dataSize = src.dataSize;
    dst.bufferType = src.bufferType;
    dst.imageType = src.imageType;
    dst.seq = src.seq;

    // 2. 需要特殊处理的字段
    // rawData: 从 vector 复制到动态数组
    if (!src.rawData.empty()) {
        dst.rawData = new uint8_t[src.rawData.size()];  // 动态分配内存
        memcpy(dst.rawData, src.rawData.data(), src.rawData.size());
    } else {
        dst.rawData = nullptr;
    }

    // 3. 未提供的字段（设为默认值）
    dst.timestamp = HafTime{};      // 默认时间戳
    dst.frameID = "";               // 空字符串
    dst.mbufData = 0;               // 默认为 0

    return dst;
}


void TlPerception::PrintImage()
{
    uint32_t instanceId = 33;
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
    while (!traDetBase.IsStop()) {
        std::shared_ptr<ImageFrameV2> iamge;
        if (traDetBase.GetImage(iamge) != HAF_SUCCESS) {
            errorStatus = true;
            return;
        }



        ImageData img2 = ConvertImageDataV2ToImageData(iamge->data);
        auto img2_ptr = std::make_shared<Adsfi::ImageData>(img2);

        std::int8_t detection;
        if (!ImageInference(nnInfer, img2_ptr, detection)) {
            HAF_LOG_FATAL << "Inference image failed. PROGRAM WILL STOP instance id: " << instanceId;
            Stop();
            break;
        }
        // HAF_LOG_INFO << "After inference in inference thread. frameid&seq: " << frameId << " " << seq;
        /****************************************************************************************/
        auto out = std::make_shared<HafTlDetectionOutArray<float64_t> >();
        HafTlDetectionOut<float64_t> outtl;
        out->count = detection;
        out->frameID = iamge->frameID;;
        out->seq = iamge->data.seq;
        out->stamp.sec = iamge->timestamp.sec;
        out->stamp.nsec = iamge->timestamp.nsec;

        if (traDetBase.SendObject(out) != HAF_SUCCESS) {
            HAF_LOG_ERROR << "TlPerception sendObject failed";
            errorStatus = true;
            return;
        }
        HAF_LOG_INFO << "TlPerception sending out objarray success";

    }
    return;
}

