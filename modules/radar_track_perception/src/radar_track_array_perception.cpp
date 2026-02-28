/*
 * Description: Define RadarTrackArrayperception cpp
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */

#include "radar_track_array_perception.h"
#include "object/object.h"

using namespace std;
using namespace Adsfi;

namespace {
const uint32_t timeScale = 1000U;
}
RadarTrackArrayPerception::~RadarTrackArrayPerception()
{
    for (auto& it : pool) {
        if (it.joinable()) {
            it.join();
        }
    }
}
void RadarTrackArrayPerception::Process()
{
    pool.push_back(std::thread(&RadarTrackArrayPerception::PrintRadar, this));
//    pool.push_back(std::thread(&RadarTrackArrayPerception::PrintLocation, this));
//    pool.push_back(std::thread(&RadarTrackArrayPerception::SendResult, this));
    return;
}

void RadarTrackArrayPerception::PrintRadar()
{
    while (!node.IsStop()) {
        std::shared_ptr<HafRadarTrackArrayFrame> data;
        if (node.GetRadar(data) != HAF_SUCCESS) {
            errorStatus = true;
            return;
        }
        timeval now;
        gettimeofday(&now, NULL);
        auto srcTimeSec = data->timestamp.sec;
        auto srcTimeNsec = data->timestamp.nsec;
        auto timeCostInMs = (now.tv_sec - srcTimeSec) * timeScale * timeScale + (now.tv_usec - srcTimeNsec / timeScale);
        HAF_LOG_INFO << "radar track array receiveTime : " <<
            static_cast<float64_t>(timeCostInMs) / static_cast<float64_t>(timeScale) << " ms";
//        frameID = data->frameID;
        seq = data->seq;
        vector<HafRadarTrackData>::iterator iter;
        for (iter = data->trackList.begin(); iter != data->trackList.end(); iter++) {
            HAF_LOG_INFO << "id: " << iter->id;
            HAF_LOG_INFO << "position.x is " << iter->position.x;
            HAF_LOG_INFO << "position.y is " << iter->position.y;
        }
        HAF_LOG_INFO << "object_time_sec: " << data->timestamp.sec;
        HAF_LOG_INFO << "object_time_nsec: " << data->timestamp.nsec;
    }
    return;
}

void RadarTrackArrayPerception::PrintLocation()
{
//    while (!node.IsStop()) {
//        std::shared_ptr<HafLocation> data;
//        if (node.GetLocation(data) != HAF_SUCCESS) {
//            errorStatus = true;
//            return;
//        }
//        HAF_LOG_INFO << "location_x: " << data->pose.pose.position.x;
//        HAF_LOG_INFO << "location_y: " << data->pose.pose.position.y;
//        HAF_LOG_INFO << "location_z: " << data->pose.pose.position.z;
//    }
//    return;
}

void RadarTrackArrayPerception::SendResult()
{
//    while (!node.IsStop()) {
//        HAF_LOG_INFO << "Sending out objarray..";
//        auto out = std::make_shared<Haf3dDetectionOutArray<float>>();
//        Haf3dDetectionOut<float32_t> out3d;
//        out3d.objectID = 1;
//        out3d.cls = 0;
//        out3d.confidence = 0;
//        out3d.existenceProbability = 1.0;
//        out3d.rect.center.x = 1.0;
//        out3d.rect.center.y = 1.0;
//        out3d.rect.center.z = 1.0;
//        out3d.rect.size.x = 1.0;
//        out3d.rect.size.y = 1.0;
//        out3d.rect.size.z = 0.0;
//        int32_t count = 2;
//        out->count = count;
//        out->detectionOut3d.push_back(out3d);
//        out->detectionOut3d.push_back(out3d);
//        out->frameID = frameID;
//        out->seq = seq;
//        if (node.SendObject(out) != HAF_SUCCESS) {
//            errorStatus = true;
//            HAF_LOG_ERROR << "SendObject failed";
//            return;
//        }
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//    }
//    return;
}
