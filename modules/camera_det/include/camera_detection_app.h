/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  camera 目标检测 demo
 */

#ifndef CAMERA_DET_CAMERA_DETECTION_APP_H
#define CAMERA_DET_CAMERA_DETECTION_APP_H

#pragma once

#include <shared_mutex>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <cstdint>
// #include <iostream>
#include "adsf/data_receive_base.h"
#include "adsf/camera_receive_base.h"
#include "core/types.h"
#include "mdc/cam/camera/cameradecodedserviceinterface_proxy.h"
#include "core/core.h"
#include "dnn/dnn.h"
#include "object/object.h"
#include "core/types.h"
#include "imageprocess/image.h"
#include "adsf/camera_det_base.h"
#include "image_nn_inference.h"
#include "camera_det_util.h"
#include "core/memory.h"
#include "udp.h"

namespace Adsfi {
class CameraDetectionApp {
public:
    explicit CameraDetectionApp(const std::string configFile0, const bool isMbuf, const size_t bufferSize,
        const std::string modelFile0): camDetBase(configFile0, isMbuf, bufferSize), modelFile(modelFile0),
        configFile(configFile0) {}
    ~CameraDetectionApp();
    HafStatus Initialize();
    void ProcessEntrance();
    void Stop();
    bool IsStop() const;

private:
    bool ImageInference(CameraDetection::ImageNnInference& nnInfer, std::shared_ptr<Adsfi::ImageData>& img2,
        std::list<Haf3dDetectionOut<float>>& out);
    void ObjectDetectionThread();
    void cleanPngDirectoryThread();
    void MonitorThread();
    bool PrepareImage4Model(CameraDetection::ImageNnInference& nnInfer, ImageData& image, ImageData& resizeOutImage);
    void PostProcess(int32_t boxNum, int32_t oriHeight, int32_t oriWidth, const float* resultPtr_0,
        std::list<Haf3dDetectionOut<float>>& detectionOut3d);
    void PostProcess_v8(int32_t oriHeight, int32_t oriWidth, const float* resultPtr,
        std::list<Haf3dDetectionOut<float>>& detectionOut3d);
    bool PostPerception(CameraDetection::ImageNnInference& nnInfer, int32_t oriHeight, int32_t oriWidth,
        std::list<Haf3dDetectionOut<float>>& out);
    void ShowImageInfo(std::shared_ptr<ImageFrameV2> data) const;
    bool ModelUpdateInput(CameraDetection::ImageNnInference& nnInfer, ImageData& resizeOutImage);
    bool PrepareDvppMemory(CameraDetection::ImageNnInference& nnInfer);
    bool ReadConfig();
    void getDirect();
    // bool drivingDiretion(int model, int ImageInsIdx);

private:
    Adsfi::CameraDetBase camDetBase{"Config.yaml"};
    std::string modelFile{"model/yolov3_symlink.om"};
    uint32_t modelH{640U};     // AI模型推理的输入尺寸。高度
    uint32_t modelW{640U};     // AI推理的输入尺寸。宽度
    int32_t imageWidth{};      // 输入图像的宽度，从配置文件读取
    int32_t imageHeight{};     // 输入图像的高度，从配置文件读取
    std::string configFile{};  // 配置文件
    Adsfi::HafContext context{};
    Adsfi::HafContextParameters contextParam{};    
    int model = 0;
    int have_obj = 0;
    uint32_t instanceId = 0;
    uint32_t instanceID1 = 0;
    uint32_t instanceID2 = 0;
    UdpMulticastReceiver receiver;
    std::vector<std::thread> threadPool;
    std::array<float32_t, 4U> imgInfo {640.0F, 640.0F, 640.0F, 640.0F};
};
}  // namespace Adsfi

#endif
