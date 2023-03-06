#pragma once
#ifndef LANE_ESTIMATION_ECAL_NODE_HPP
#define LANE_ESTIMATION_ECAL_NODE_HPP

#include "node.hpp"
#include "lane-estimation.hpp"
#include "perception/perceptionDS.hpp"
#include "commons.hpp"
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>

class LaneEstimationModule : public PerceptionModule
{
    public:
    LaneEstimationModule();

    void topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_);
    bool inference(float* buffer);
    bool writeMsg();

    #ifdef RENDER_PERCEPTION_LANE_ESTIMATION
    bool render();
    #endif

    std::unique_ptr<eCAL::CSubscriber> subscriber;
    std::unique_ptr<eCAL::CPublisher> publisher;

    // Model
    // std::unique_ptr<LaneEstimation> laneEstimation = std::make_unique<LaneEstimation>("/home/minuszero/Downloads/engine_files/lstr8jetson.engine", true, false);
    std::unique_ptr<LaneEstimation> laneEstimation = std::make_unique<LaneEstimation>("/home/minuszero/models/lane-estimation/model_188849_1.engine", true, false);
    LaneEstimationData laneData;
};

#endif