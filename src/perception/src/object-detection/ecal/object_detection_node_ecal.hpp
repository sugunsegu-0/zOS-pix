#pragma once
#ifndef OBJECT_DETECTION_ECAL_NODE_HPP
#define OBJECT_DETECTION_ECAL_NODE_HPP

#include "node.hpp"
#include "object-detection.hpp"
#include "perception/perceptionDS.hpp"
#include <ecal/ecal.h>
#include "commons.hpp"


class ObjectDetectionModule : public PerceptionModule
{
    public:
    ObjectDetectionModule();

    void topic_callback(const char* topic_name, const struct eCAL::SReceiveCallbackData* data_);
    bool inference(uint8_t* buffer);
    bool writeMsg();

    #ifdef RENDER_PERCEPTION_OBJECT_DETECTION
    bool render();
    #endif

    std::unique_ptr<eCAL::CSubscriber> subscriber;
    std::unique_ptr<eCAL::CPublisher> publisher;

    // Model
    // std::unique_ptr<ObjectDetection> objectDetection = std::make_unique<ObjectDetection>("/home/minuszero/Downloads/engine_files/yolov5m8jetson.engine", true, false);
    // std::unique_ptr<ObjectDetection> objectDetection = std::make_unique<ObjectDetection>("/home/minuszero/models/object-detection/yolov5jetsonDLA.engine", true, false);
    // std::unique_ptr<ObjectDetection> objectDetection = std::make_unique<ObjectDetection>("/home/minuszero/models/object-detection/yolov5m-idd-3.engine", true, false);
    std::unique_ptr<ObjectDetection> objectDetection = std::make_unique<ObjectDetection>("/home/minuszero/models/object-detection/v8/yolov8s_1epoch_1_best.engine", true, false);
    ObjectDetectionData objectData;
};

#endif