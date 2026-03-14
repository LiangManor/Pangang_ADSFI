/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description:  神经网络推理封装
 */

#include "image_nn_inference.h"
#include "core/group.h"
#include "imageprocess/convert_color.h"
#include "imageprocess/resize.h"
#include "imageprocess/crop.h"
#include "imageprocess/image.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace CameraDetection;
using namespace Adsfi;

std::vector<Adsfi::HafRoi> ImageNnInference::crop_area = {{420, 0, 1499, 1079}};  //森云 1920*1080 crop 1080*1080
// std::vector<Adsfi::HafRoi> ImageNnInference::crop_area = {{360, 0, 1559, 1199}};  //海康 1920*1200 crop 1200*1200

/**
 * @brief 加载模型，并设置模型的输入输出
 *
 */
bool ImageNnInference::LoadModelPrepareInputOutput()
{
    HafSetGroup(groupId);
    if (HafDNNModelInitialize(dnnHandle, modelFile) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Init nn model failed!";
        return false;
    }
    constexpr uint32_t imgInfoSize{4U};
    constexpr int32_t RGB_SCALE = 3;
    size_t index = 0U;

    if (HafDNNModelCreateInput(dnnHandle, index, dvppOutput.width * dvppOutput.height * RGB_SCALE, dvppOutput.rawData) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Create nn input failed!";
        (void)HafDNNRelease(dnnHandle);
        return false;
    }

    // std::cout << "++++++++++++ HafDNNModelInitialize set nn input ++++++++++++++"<<std::endl;

    if (HafDNNModelCreateOutput(dnnHandle) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Create nn output failed!";
        (void)HafDNNRelease(dnnHandle);
        return false;
    }
    return true;
}

/**
 * @brief 这个API用于准备神经网络推理的过程。如果是多个线程的推理过程，每个线程里面要做一次初始化。
 *
 */
bool ImageNnInference::Initialize()
{
    // 每个线程需要创建一次context
    if (HafCreateContext(context) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Create context failed";
        return false;
    }
    if (!LoadModelPrepareInputOutput()) {
        HAF_LOG_ERROR << "Load model prepare input/output failed.";
        return false;
    }
    if (HafStreamCreate(stream) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Create stream failed!";
        (void)HafDNNRelease(dnnHandle);
        return false;
    }
    if (HafCreateChannel(channelId) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "HafCreateChannel fail";
        return false;
    }
    HAF_LOG_INFO << "Initialize NN finished.";
    return true;
}

HafStatus ImageNnInference::DNNModelUpdateInput(void * const buffer, const size_t bufferSize, const size_t index)
{
    return HafDNNModelUpdateInput(dnnHandle, buffer, bufferSize, index);
}

HafStatus ImageNnInference::DNNModelProcess()
{
    return HafDNNModelProcess(dnnHandle);
}

HafStatus ImageNnInference::DNNModelGetOutputBuffer(const size_t index, void** buffer, size_t bufferSize)
{
    return HafDNNModelGetOutputBuffer(dnnHandle, index, buffer, bufferSize);
}

//////////////////////      保存各种格式的图像为 png    //////////////////////////
// // Function to convert YUV420 to BGR and save it
// void saveYUV420AsImage(const uint8_t* yuvData, int width, int height, size_t dataSize, const std::string& outputFilename) {
//     // Create an empty Mat for YUV420 image
//     cv::Mat yuv420Image(height + height / 2, width, CV_8UC1, (void*)yuvData);

//     // Convert to BGR format
//     cv::Mat bgrImage;
//     cv::cvtColor(yuv420Image, bgrImage, cv::COLOR_YUV2BGR_I420);

//     // Save the BGR image as a PNG or JPEG file
//     cv::imwrite(outputFilename, bgrImage);
// }

// // Function to convert YUV422 UYVY to BGR (OpenCV format) and save it
// void saveYUV422UYVYAsImage(const uint8_t* yuvData, int width, int height, size_t dataSize, const std::string& outputFilename) {
//     // Create an empty Mat with the same size as the input image
//     cv::Mat yuv422uyvyImage(height, width, CV_8UC2, (void*)yuvData);

//     // Convert to BGR format
//     cv::Mat bgrImage;
//     cv::cvtColor(yuv422uyvyImage, bgrImage, cv::COLOR_YUV2BGR_UYVY);

//     // Save the BGR image as a PNG or JPEG file
//     cv::imwrite(outputFilename, bgrImage);
// }

// // Function to save RGB image
// void saveRGBAsImage(const uint8_t* rgbData, int width, int height, size_t dataSize, const std::string& outputFilename) {
// // Create an empty Mat with the same size as the input image
// cv::Mat rgbImage(height, width, CV_8UC3, (void*)rgbData);

// cv::Mat bgrImage;
// cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);
// // Save the RGB image as a PNG or JPEG file
// cv::imwrite(outputFilename, bgrImage);
// }

// // Function to convert YUV422 UYVY to RGB
// void convertYUV422UYVYToRGB(const uint8_t* yuvData, int width, int height, cv::Mat& rgbImage) {
//     // Create an OpenCV Mat object from the raw YUV422 UYVY data
//     cv::Mat yuv422uyvyImage(height, width, CV_8UC2, const_cast<uint8_t*>(yuvData));

//     // Convert YUV422 UYVY to RGB
//     cv::cvtColor(yuv422uyvyImage, rgbImage, cv::COLOR_YUV2RGB_UYVY);
// }
///////////////////////////////////////////////////////////////////////////////
bool ImageNnInference::YUV422UYVY_2_NV12(Adsfi::ImageData& inputImage, Adsfi::ImageData&  outputImage)
{
    hi_vpc_pic_info INyuv422 = {};
    hi_vpc_pic_info OUTyuv420 = {};
////////////////////////////////////////////////////////////////
// std::cout << "------------------------------YUV422UYVY_2_NV12_start: " << std::endl;
    INyuv422.picture_address = inputImage.rawData;
// std::cout << "------------------------------YUV422UYVY_2_NV12_INyuv422: " << std::endl;
    INyuv422.picture_buffer_size = 1920*1080*2;
    INyuv422.picture_width = 1920;
    INyuv422.picture_height = 1080;
    INyuv422.picture_width_stride = 1920*2;
    INyuv422.picture_height_stride = 1080;
    INyuv422.picture_format = HI_PIXEL_FORMAT_UYVY_PACKED_422;
// std::cout << "------------------------------YUV422UYVY_2_NV12_INyuv422_end: " << std::endl;

    OUTyuv420.picture_address = outputImage.rawData;
// std::cout << "------------------------------YUV422UYVY_2_NV12_OUTyuv420: " << std::endl;
    OUTyuv420.picture_buffer_size = 1920*1080*3/2;
    OUTyuv420.picture_width = 1920;
    OUTyuv420.picture_height = 1080;
    OUTyuv420.picture_width_stride = 1920;
    OUTyuv420.picture_height_stride = 1080;
    OUTyuv420.picture_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
// std::cout << "------------------------------YUV422UYVY_2_NV12_OUTyuv420_end: " << std::endl;
////////////////////////////////////////////////////////////////

    uint32_t task_id = 0;
// std::cout << "------------------------------hi_mpi_vpc_convert_color_to_yuv420: " << std::endl;
    // auto hi_mpi_vpc_convert_color_to_yuv420 (hi_vpc_chn chn, const hi_vpc_pic_info *source_pic, hi_vpc_pic_info *dest_pic, hi_u32 *task_id, hi_s32 milli_sec)
    hi_s32 convert_color_sign = hi_mpi_vpc_convert_color_to_yuv420 (channelId, &INyuv422, &OUTyuv420, &task_id, -1);
    if (convert_color_sign != 0) {
        HAF_LOG_ERROR << "Apply for DVPP memory failed. convert_color_sign.";
        // std::cout << "------------------------------convert_color_sign_error: " << std::hex << convert_color_sign << std::endl;
    }

    uint32_t taskIDResult = task_id;
    hi_mpi_vpc_get_process_result(channelId, taskIDResult, -1);
// std::cout << "------------------------------hi_mpi_vpc_convert_color_to_yuv420_end: " << std::endl;
    return true;
}
///////////////////////////////////////////////////////////////////////////////

bool ImageNnInference::ImageResize(
    ImageData& inputImage, ImageData& outputImage, const HafImageInterpolationType type)
{
    /*----------------------------   计时   -------------------------------*/
    std::chrono::high_resolution_clock::time_point start_time_opencv, end_time_opencv;        //计算耗时
    std::chrono::milliseconds elapsed_ms;
    /*----------------------------   计时   -------------------------------*/

    if (!dvppMemoryPrepared) {
        HAF_LOG_ERROR << "Dvpp memory not prepared. Call method PrepareDvppMemory first.";
        return false;
    }
    if (inputImage.rawData == nullptr) {
        HAF_LOG_ERROR << "Input image is null";
        return false;
    }

    // ////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
    //                 /*----------------------------   计时   -------------------------------*/
    //         start_time_opencv = std::chrono::high_resolution_clock::now();
    //         /*----------------------------   计时   -------------------------------*/
    // cv::Mat rgbImage;
    // convertYUV422UYVYToRGB(inputImage.rawData, inputImage.width, inputImage.height, rgbImage);
    // inputImage.dataSize = inputImage.width*inputImage.height*3;
    // inputImage.imageType = HAF_IMAGE_RGB_UINT8;
    // inputImage.rawData = const_cast<uint8_t*>(rgbImage.data);

    //                 /*----------------------------   计时   -------------------------------*/
    //         end_time_opencv = std::chrono::high_resolution_clock::now();
    //         elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_opencv - start_time_opencv);
    //         std::cout << "Opencv_convertYUV422UYVYToRGB_time: " << elapsed_ms.count() << " mm\n";
    //         /*----------------------------   计时   -------------------------------*/

    // if (HafMemcpyToDvpp(inputImage, dvppTemp) != HAF_SUCCESS) {
    //     HAF_LOG_ERROR << "Copy image to dvpp failed";
    //     return false;
    // }
    // ////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<</////

    ////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>////
    if (HafMemcpyToDvpp(inputImage, dvppInput) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Copy image to dvpp failed";
        return false;
    }

    ImageNnInference::YUV422UYVY_2_NV12(dvppInput, dvppTemp);
    ////<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<</////

    // mbuf图像使用结束后手动释放
    if (HafImageDestroy(inputImage) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destroy inputImage buffer failed.";
    }

    // std::cout<< "This HafImageCrop step ---------------------" <<std::endl;
    // std::cout<< "This dvppInput size:   ---------------------" << dvppInput.width << "--" << dvppInput.height << std::endl;

    if (HafImageCrop(channelId, dvppTemp, outputVector, ImageNnInference::crop_area, type) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Crop failed.";
        return false;
    }

    // std::cout<< "This HafImageCrop finish ---------------------" <<std::endl;

    if (HafImageResize(channelId, outputVector[0], dvppOutput, type) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Resize failed.";
        return false;
    }


    /////---------------  Print() -----------------////////
    // std::cout<< "-------  get dvppOutput size   --------" << dvppOutput.dataSize;
    // std::cout<< "-------  imagewidth   --------" << dvppOutput.width << "-------  imageheight   --------" << dvppOutput.height;
    // std::cout<< "----imageType----" << dvppOutput.imageType << std::endl;

    // saveRGBAsImage(dvppOutput.rawData, dvppOutput.width, dvppOutput.height, dvppOutput.dataSize,  "resizeOutImage_rgb.png");
    /////------------------------------------------///////
    outputImage = dvppOutput;
    return true;
}

bool ImageNnInference::PrepareDvppMemory(const ImageData& imgIn, const ImageData& imgOut)
{
    // 从入参获取 图像大小和类型、buffer类型
    dvppInput = imgIn;  // 获取输入图片的width/height/imageType信息--Uint8
    dvppInput.bufferType = HAF_BUFFER_DVPP;

    dvppTemp.width = imgIn.width;
    dvppTemp.height = imgIn.height;
    dvppTemp.bufferType = HAF_BUFFER_DVPP;
    dvppTemp.imageType = HAF_IMAGE_YUV420SP_NV12_UINT8;


    dvppCrop.width = 1080;  // 获取 Crop 输出图片的width/height/imageType信息
    dvppCrop.height = 1080;
    dvppCrop.bufferType = HAF_BUFFER_DVPP;
    dvppCrop.imageType = HAF_IMAGE_RGB_UINT8;

    dvppOutput = imgOut;  // 获取输出图片的width/height/imageType信息
    dvppOutput.bufferType = HAF_BUFFER_DVPP;
    dvppOutput.imageType = HAF_IMAGE_RGB_UINT8;

    // 根据图片的width/height/imageType信息计算要多少DVPP内存空间
    if (HafImageMallocAligned(dvppInput) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Apply for DVPP memory failed. dvppInput part.";
        return false;
    }
    if (HafImageMallocAligned(dvppCrop) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Apply for DVPP memory failed. dvppCrop part.";
        return false;
    }

    outputVector.push_back(dvppCrop);

    ///////////////////////////////////////////
    if (HafImageMallocAligned(dvppTemp) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Apply for DVPP memory failed. dvppTemp part.";
        return false;
    }

    if (HafImageMallocAligned(dvppOutput) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Apply for DVPP memory failed. dvppOutput part.";
        return false;
    }

    // hi_u32 dev_id = 0;
    // hi_u64 size = 1920*1080*3/2;

    // int32_t dvpp_malloc_sign = hi_mpi_dvpp_malloc(dev_id, &dev_ptr, size);
    // if (dvpp_malloc_sign != 0) {
    //     HAF_LOG_ERROR << "Apply for DVPP memory failed. dev_ptr.";
    //     std::cout << "------------------------------dvpp_malloc_error: " << std::hex << dvpp_malloc_sign << std::endl;
    // }
    ///////////////////////////////////////////
    dvppMemoryPrepared = true;
    return true;
}

void ImageNnInference::DestoryDvppMemory()
{
    if (HafImageDestroy(dvppInput) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destroy dvpp input buffer failed.";
    }
    if (HafImageDestroy(dvppOutput) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destroy dvpp output buffer failed.";
    }
    if (HafImageDestroy(dvppCrop) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destroy dvpp output buffer failed.";
    }
    ///////////////////////////////////
    if (HafImageDestroy(dvppTemp) != HAF_SUCCESS) {
        HAF_LOG_ERROR << "Destroy dvpp dvppTemp buffer failed.";
    }

    // int32_t dvpp_free_sign = hi_mpi_dvpp_free(dev_ptr);
    // if (dvpp_free_sign != 0) {
    // HAF_LOG_ERROR << "Apply for DVPP memory failed. dev_ptr.";
    // std::cout << "------------------------------dvpp_free_error: " << std::hex << dvpp_free_sign << std::endl;
    // }

    ///////////////////////////////////
    return;
}

ImageNnInference::~ImageNnInference()
{
    (void)DestoryDvppMemory();
    (void)HafSetCurrentContext(context);
    (void)HafStreamDestroy(stream);
    (void)HafDNNRelease(dnnHandle);
    (void)HafDestroyChannel(channelId);
    (void)HafDestroyContext(context);
}
