/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  Traffic Light 检测 sample头文件
 */

#ifndef SAMPLE_TL_PERCEPTION_H
#define SAMPLE_TL_PERCEPTION_H

#include <shared_mutex>
#include <vector>
#include <condition_variable>
#include <mutex>
#include "adsf/data_receive_base.h"
#include "adsf/camera_receive_base.h"
#include "core/types.h"
#include "mdc/cam/camera/cameradecodedserviceinterface_proxy.h"
#include "core/core.h"
#include "dnn/dnn.h"
#include "object/object.h"
#include "core/types.h"
#include "imageprocess/image.h"
#include "image_nn_inference.h"
#include "core/memory.h"
#include <shared_mutex>
#include <string>
#include "core/core.h"
#include "adsf/traffic_light_det_base.h"

class TlPerception {
public:
    explicit TlPerception(std::string configFile) : traDetBase(configFile) {};
    ~TlPerception();
    Adsfi::HafStatus Init()
    {
        return traDetBase.Init();
    };
    Adsfi::HafStatus Initialize();
    void Stop()
    {
        traDetBase.Stop();
        return;
    };
    void Process();
    // void SendResult();
    // void PrintLocation();
    void PrintImage();
    bool IsError() const
    {
        return errorStatus;
    }

private:
    bool ImageInference(CameraDetection::ImageNnInference& nnInfer, std::shared_ptr<Adsfi::ImageData>& img2, std::int8_t& out);
    void ObjectDetectionThread();
    void MonitorThread();
    bool PrepareImage4Model(CameraDetection::ImageNnInference& nnInfer, Adsfi::ImageData& image, Adsfi::ImageData& resizeOutImage);
    void PostProcess(int32_t oriHeight, int32_t oriWidth, const float* resultPtr,
        std::int8_t& detectionOut3d);
    void PostProcess_v8(int32_t oriHeight, int32_t oriWidth, const float* resultPtr, const float* resultPtr_1,
        std::int8_t& detectionOut3d);
    bool PostPerception(CameraDetection::ImageNnInference& nnInfer, int32_t oriHeight, int32_t oriWidth,
        std::int8_t& out);
    void ShowImageInfo(std::shared_ptr<Adsfi::ImageFrameV2> data) const;
    bool ModelUpdateInput(CameraDetection::ImageNnInference& nnInfer, Adsfi::ImageData& resizeOutImage);
    bool PrepareDvppMemory(CameraDetection::ImageNnInference& nnInfer);
    bool ReadConfig();
    void getDirect();


private:
    Adsfi::TrafficLightDetBase traDetBase{"Config.yaml"};
    std::string modelFile{"/home/sshuser/traffic_light_det/model/traffic_light.om"};
    // Adsfi::TrafficLightDetBase node{"Config.yaml"};
    std::string frameID = "TlPerception";
    bool errorStatus = false;
    uint32_t modelH{640U};  // AI模型推理的输入尺寸。高度
    uint32_t modelW{640U};  // AI推理的输入尺寸。宽度
    uint32_t seq = 1U;
    std::vector<std::thread> pool;
};
#endif
