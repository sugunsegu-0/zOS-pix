#pragma once
#ifndef SEGMENTATION_ECAL_NODE_HPP
#define SEGMENTATION_ECAL_NODE_HPP

#include "node.hpp"
#include "segmentation.hpp"
#include "perception/perceptionDS.hpp"
#include <ecal/ecal.h>
#include "commons.hpp"

class SegmentationModule : public PerceptionModule
{
    public:
    SegmentationModule();

    void topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_);
    bool inference(uint8_t* buffer);
    bool writeMsg();

    #ifdef RENDER_PERCEPTION_SEGMENTATION
    bool render();
    #endif

    std::unique_ptr<eCAL::CSubscriber> subscriber;
    std::unique_ptr<eCAL::CPublisher> publisher[2];

    // Model
    // std::unique_ptr<Segmentation> segmentation = std::make_unique<Segmentation>("/home/minuszero/Downloads/engine_files/hardnet_idd8jetson.engine", true, false);
    // std::unique_ptr<Segmentation> segmentation = std::make_unique<Segmentation>("/home/minuszero/models/segmentation/hardnet_idd8jetsonDLA.engine", true, false);
    std::unique_ptr<Segmentation> segmentation = std::make_unique<Segmentation>("/home/minuszero/models/segmentation/hardnet_idd_3.engine", true, false);
    SegmentationData segmentationData;
};

#endif