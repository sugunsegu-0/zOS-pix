#include "segmentation_node_ecal.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

SegmentationModule::SegmentationModule()
: PerceptionModule()
{
    if(!segmentation.get()->loadModel())
        throw std::runtime_error("Unable to load segmentation model");

    subscriber.reset(new eCAL::CSubscriber("multicam"));
    subscriber->AddReceiveCallback(std::bind(&SegmentationModule::topic_callback, this, std::placeholders::_1, std::placeholders::_2));

    publisher[0].reset(new eCAL::CPublisher("blended-mask-topic"));
    publisher[1].reset(new eCAL::CPublisher("driveable-region-topic"));
}

void SegmentationModule::topic_callback(const char* topic_name_, const struct eCAL::SReceiveCallbackData* data_) 
{
    auto start = std::chrono::system_clock::now();
    inference(static_cast<uint8_t*>(data_->buf));

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end-start;

    std::cout << "Time to process last frame (seconds) - Segmentation: " << diff.count() 
                << " FPS: " << 1.0 / diff.count()  << "\n";
    
    writeMsg();
}

bool SegmentationModule::inference(uint8_t* buffer)
{
    if(!segmentation.get()->runInference(buffer))
        throw std::runtime_error("onProcess failed for segmentation model");
    // segmentation->getOutput(&segmentationData);
    return true;
}

bool SegmentationModule::writeMsg()
{
    // publisher[0]->Send(segmentationData.bufferLane.data(),perception_camera
    //                    (segmentation->modelConfig.outputBlobDims[0])*(segmentation->modelConfig.outputBlobDims[1])*3*sizeof(float)*(segmentation->modelConfig.batchSize));
    // publisher[1]->Send(segmentationData.bufferFreespace.data(),
    //                    (segmentation->modelConfig.outputBlobDims[0])*(segmentation->modelConfig.outputBlobDims[1])*3*sizeof(float)*(segmentation->modelConfig.batchSize));
    publisher[0]->Send(segmentation->maskLaneCPU,
                       (segmentation->modelConfig.outputBlobDims[0])*(segmentation->modelConfig.outputBlobDims[1])*3*sizeof(float)*(segmentation->modelConfig.batchSize));
    publisher[1]->Send(segmentation->maskFreeSpaceCPU,
                       (segmentation->modelConfig.outputBlobDims[0])*(segmentation->modelConfig.outputBlobDims[1])*3*sizeof(float)*(segmentation->modelConfig.batchSize));
    return true;
}

#ifdef RENDER_PERCEPTION_SEGMENTATION
bool SegmentationModule::render()
{
    segmentation->Render();
    return true;
}
#endif

