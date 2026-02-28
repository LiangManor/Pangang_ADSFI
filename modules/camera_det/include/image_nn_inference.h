/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  神经网络推理封装
 */

#ifndef CAMERA_DET_IMAGE_NN_INFERENCE_H
#define CAMERA_DET_IMAGE_NN_INFERENCE_H

#pragma once

#include "core/core.h"
#include "dnn/dnn.h"
#include "object/object.h"
#include "core/types.h"
#include "imageprocess/image.h"
#include "core/memory.h"

#ifndef ENABLE_DVPP_INTERFACE
#define ENABLE_DVPP_INTERFACE
#include "acl/ops/acl_dvpp.h"
#endif

#include "acl/dvpp/hi_dvpp_vpc.h"

namespace CameraDetection {
/**
 * @brief 这个类是辅助神经网络推理的。如果是多个线程推理的场景，应该在每个推理线程里面去初始化。
 *
 */
class ImageNnInference {
public:
    ImageNnInference() = default;
    ImageNnInference(const std::string file, const Adsfi::int32_t channel, const Adsfi::uint32_t group)
        : modelFile(file), channelId(channel), groupId(group){};
    ~ImageNnInference();
    bool Initialize();
    Adsfi::HafStatus DNNModelUpdateInput(void * const buffer, const size_t bufferSize, const size_t index);
    Adsfi::HafStatus DNNModelProcess();
    Adsfi::HafStatus DNNModelGetOutputBuffer(const size_t index, void** buffer, size_t bufferSize);
    bool ImageResize(Adsfi::ImageData& inputImage, Adsfi::ImageData& outputImage,
        const Adsfi::HafImageInterpolationType type = Adsfi::HAF_INTER_LINEAR);
    bool PrepareDvppMemory(const Adsfi::ImageData& imgIn, const Adsfi::ImageData& imgOut);
    bool YUV422UYVY_2_NV12(Adsfi::ImageData& inputImage, Adsfi::ImageData&  dvppTemp);


private:
    bool LoadModelPrepareInputOutput();
    void DestoryDvppMemory();

private:
    std::string modelFile{};
    Adsfi::int32_t channelId{};       // 用于图像resize
    const Adsfi::uint32_t groupId{};  // 用于图像推理
    Adsfi::HafDNNHandle dnnHandle{};
    Adsfi::HafStream stream{};
    Adsfi::HafContext context{};
    Adsfi::HafContextParameters contextParam{};
    Adsfi::ImageData dvppInput;
    Adsfi::ImageData dvppOutput;
    std::vector<Adsfi::ImageData>  outputVector;
    Adsfi::ImageData dvppTemp;
    Adsfi::ImageData dvppCrop;
    // hi_void * dev_ptr = nullptr;
    bool dvppMemoryPrepared{false};
public:
    static std::vector<Adsfi::HafRoi> crop_area;
    std::array<Adsfi::float32_t, 4U> imgInfo {640.0F, 640.0F, 640.0F, 640.0F};
};
};  // namespace CameraDetection
#endif
