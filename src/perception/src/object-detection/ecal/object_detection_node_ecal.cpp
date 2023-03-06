#include "object_detection_node_ecal.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

ObjectDetectionModule::ObjectDetectionModule()
: PerceptionModule()
{
    if(!objectDetection.get()->loadModel())
        throw std::runtime_error("Unable to load object-detection model");

    subscriber.reset(new eCAL::CSubscriber("multicam"));
    subscriber->AddReceiveCallback(std::bind(&ObjectDetectionModule::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
    publisher.reset(new eCAL::CPublisher("object-detection-topic"));
}

void ObjectDetectionModule::topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_) 
{
    auto start = std::chrono::system_clock::now();
    inference(static_cast<uint8_t*>(data_->buf));

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;

    std::cout << "Time to process last frame (seconds) - Object Detection: " << diff.count() 
                << " FPS: " << 1.0 / diff.count()  << "\n";

    writeMsg();
}

bool ObjectDetectionModule::inference(uint8_t* buffer)
{
    if(!objectDetection.get()->runInference(buffer))
        throw std::runtime_error("onProcess failed for object-detection model");
    objectDetection->getOutput(&objectData);
    return true;
}

bool ObjectDetectionModule::writeMsg()
{
    //Serialising data to send over ICE
    Serialize<ObjectDetectionData> data;
    std::stringstream ss;
    data.serialize(objectData,ss);
    std::string temp = ss.str();
    publisher->Send(temp.c_str(), sizeof(temp));

    return true;
}

#ifdef RENDER_PERCEPTION_OBJECT_DETECTION
bool ObjectDetectionModule::render()
{
    objectDetection->Render();
    return true;
}
#endif

