#include "lane_estimation_node_ecal.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

LaneEstimationModule::LaneEstimationModule()
: PerceptionModule()
{
    if(!laneEstimation.get()->loadModel())
        throw std::runtime_error("Unable to load lane-estimation model");

    subscriber.reset(new eCAL::CSubscriber("blended-mask-topic"));
    subscriber->AddReceiveCallback(std::bind(&LaneEstimationModule::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    publisher.reset(new eCAL::CPublisher("lane-estimation-topic"));
}

void LaneEstimationModule::topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_) 
{
    auto start = std::chrono::system_clock::now();
    inference(static_cast<float*>(data_->buf));

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;

    std::cout << "Time to process last frame (seconds) - Lane Estimation: " << diff.count() 
                << " FPS: " << 1.0 / diff.count()  << "\n";

    writeMsg();
}

bool LaneEstimationModule::inference(float* buffer)
{
    if(!laneEstimation.get()->runInference(buffer))
        throw std::runtime_error("onProcess failed for lane-estimation model");
    laneEstimation->getOutput(&laneData);
    return true;
}

bool LaneEstimationModule::writeMsg()
{
    // Serialising data to send over ICE    
    Serialize<LaneEstimationData> data;
    std::stringstream ss;
    data.serialize(laneData,ss);
    std::string temp = ss.str();
    publisher->Send(temp.c_str(), sizeof(temp));
    return true;
}

#ifdef RENDER_PERCEPTION_LANE_ESTIMATION
bool LaneEstimationModule::render()
{
    laneEstimation->Render();
    return true;
}
#endif

