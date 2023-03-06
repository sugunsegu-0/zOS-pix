#include "freespace_node_ecal.hpp"

FreeSpaceModule::FreeSpaceModule()
: PerceptionModule()
{
    if(!freespace.get()->loadModel())
        throw std::runtime_error("Unable to load free space model");

    subscriber.reset(new eCAL::CSubscriber("driveable-region-topic"));
    subscriber->AddReceiveCallback(std::bind(&FreeSpaceModule::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher.reset(new eCAL::CPublisher("freespace-topic"));
}

void FreeSpaceModule::topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_) 
{
    auto start = std::chrono::system_clock::now();
    inference(static_cast<float*>(data_->buf));

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;

    std::cout << "Time to process last frame (seconds) - Freespace: " << diff.count() 
                << " FPS: " << 1.0 / diff.count()  << "\n";

    writeMsg();
}

bool FreeSpaceModule::inference(float* buffer)
{
    if(!freespace.get()->runInference(buffer))
        throw std::runtime_error("onProcess failed for freespace model");
    freespace->getOutput(&freeData);
    return true;
}

bool FreeSpaceModule::writeMsg()
{
    // Serialising data to send over ICE
    Serialize<FreeSpaceData> data;
    std::stringstream ss;
    data.serialize(freeData,ss);
    std::string temp = ss.str();
    publisher->Send(temp.c_str(), sizeof(temp));

    return true;
}

#ifdef RENDER_PERCEPTION_FREESPACE
bool FreeSpaceModule::render()
{
    freespace->Render();
    return true;
}
#endif

