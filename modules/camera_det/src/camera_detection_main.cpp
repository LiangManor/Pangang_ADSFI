/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  camera 目标检测 demo
 */

#include "camera_detection_app.h"
#include <iostream>
#include "image_nn_inference.h"
#include "core/core.h"
#include "camera_det_util.h"
using namespace Adsfi;
using namespace CameraDetection;

int32_t main()
{
    CameraDetectionApp camDetApp("Config.yaml", true, 5U, "model/yolov8s-det.om");
//    CameraDetectionApp camDetApp("Config.yaml", true, 5U, "model/yolov3.om");
    if (camDetApp.Initialize() != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Camera detection. Initialize failed";
        camDetApp.Stop();
        return -1;
    }

    if (!camDetApp.IsStop()) {
        camDetApp.ProcessEntrance();
    }

    while (!camDetApp.IsStop()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    camDetApp.Stop();

    HAF_LOG_INFO << "The end of main...";
    return 0;
}
