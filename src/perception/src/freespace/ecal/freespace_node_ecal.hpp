#pragma once
#ifndef FREESPACE_ECAL_NODE_HPP
#define FREESPACE_ECAL_NODE_HPP

#include "node.hpp"
#include "freespace.hpp"
#include "perception/perceptionDS.hpp"
#include <opencv2/core/mat.hpp>
#include "commons.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

class FreeSpaceModule : public PerceptionModule
{
    public:
    FreeSpaceModule();

    void topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_);
    bool inference(float* buffer);
    bool writeMsg();

    #ifdef RENDER_PERCEPTION_FREESPACE
    bool render();
    #endif

    std::unique_ptr<eCAL::CSubscriber> subscriber;
    std::unique_ptr<eCAL::CPublisher> publisher;

    // Model
    std::unique_ptr<FreeSpace> freespace = std::make_unique<FreeSpace>();
    FreeSpaceData freeData;
};

#endif