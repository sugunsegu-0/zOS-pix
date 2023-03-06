#include "perception.hpp"
Perception::Perception()
{
    // objDet = std::make_unique<ObjectDetection>("/home/minuszero/models/object-detection/yolov5m-idd.engine", true, false);
    objDet = std::make_unique<ObjectDetection>("/home/minuszero/models/object-detection/yolov5m8jetson.engine", true, false);
    // objDet = std::make_unique<ObjectDetection>("/home/ade/models/object-detection/yolojetson.engine", true, false);
    // objDet = std::make_unique<ObjectDetection>("/media/ade/24eff3b0-fece-4db2-9904-473f3aaab857/home/minuszero/models/object-detection/yolov5m-idd-simplified-16.engine", true, false);
    // objDet = std::make_unique<ObjectDetection>("/media/ade/24eff3b0-fece-4db2-9904-473f3aaab857/home/minuszero/models/object-detection/yolov5m-idd.engine", true, false);
    // segMen = std::make_unique<Segmentation>("/home/ade/models/segmentation/hardnet.engine", true, false);
    segMen = std::make_unique<Segmentation>("/home/minuszero/models/segmentation/hardnet_idd8jetson.engine", true, false);
    // segMen = std::make_unique<Segmentation>("/home/minuszero/FCHarDNet/hardnet_idd.engine", true, false);
    // segMen = std::make_unique<Segmentation>("/home/minuszero/models/segmentation/hardnet_idd_better.engine", true, false);
    // maskCorrection = std::make_unique<MaskCorrection>("/home/minuszero/pytorch-CycleGAN-and-pix2pix/pix2pix_idd.engine", true, false);
    // laneEst = std::make_unique<LaneEstimation>("/home/minuszero/models/lane-estimation/lstr.engine", true, false);
    // laneEst = std::make_unique<LaneEstimation>("/home/minuszero/models/lane-estimation/lstr-8.engine", true, false);
    laneEst = std::make_unique<LaneEstimation>("/home/minuszero/models/lane-estimation/lstr8jetson.engine", true, false);
    freespace = std::make_unique<FreeSpace>();
    std::cout << "Created Perception module" << std::endl;
}

Perception::~Perception()
{}

bool Perception::onInitialize()
{  
    if(!objDet.get()->loadModel())
        throw std::runtime_error("Unable to load object detection model");

    if(!segMen.get()->loadModel())
        throw std::runtime_error("Unable to load segmentation model");
    
    // if(!maskCorrection.get()->loadModel())
    //     throw std::runtime_error("Unable to load mask correction model");

    if(!laneEst.get()->loadModel())
        throw std::runtime_error("Unable to load lane estimation model");

    if(!freespace.get()->loadModel())
        throw std::runtime_error("Unable to load free space model");

    return true;
}

bool Perception::onProcess(std::vector<cv::Mat> imgBatch)
{ 
    // if(!objDet.get()->runInference(imgBatch))
        // throw std::runtime_error("onProcess failed for object detection model");
    // std::cout << "Object Detection Inference complete" << std::endl;
// 
    if(!segMen.get()->runInference(imgBatch))
        throw std::runtime_error("onProcess failed for segmentation model");
    std::cout << "Segmentation Inference complete" << std::endl;

    // if(!maskCorrection.get()->runInference(imgBatch))
    //     throw std::runtime_error("onProcess failed for mask correction model");
    
    
    std::vector<cv::Mat> blendedFrames, drivableRegionFrames;
    segMen->getBlendedFrames(&blendedFrames); segMen->getDrivableRegionFrames(&drivableRegionFrames);

    // if(!laneEst.get()->runInference(blendedFrames))
    //     throw std::runtime_error("onProcess failed for lane estimation model");
    // std::cout << "Lane Estimation Inference complete" << std::endl;

    // if(!freespace.get()->runInference(drivableRegionFrames))
    //     throw std::runtime_error("onProcess failed for freespace model");
    // std::cout << "Freespace Inference complete" << std::endl;

    return true;
}

bool Perception::getOutput(PerceptionData* perceptionOutputCurrent)
{
    // objDet->getOutput(&(perceptionOutputCurrent->objectDetectionData));
    // laneEst->getOutput(&(perceptionOutputCurrent->laneEstimationData));
    // freespace->getOutput(&(perceptionOutputCurrent->freeSpaceData));
    return true;
}

// bool Perception::onPublishData()
// {
//     // if(!objDet.get()->publishData())
//     //     throw std::runtime_error("Unable to publish object detection output data");

//     // if(!segMen.get()->publishData())
//     //     throw std::runtime_error("Unable to publish segmentation output data");

//     // if(!maskCorrection.get()->publishData())
//     //     throw std::runtime_error("Unable to publish mask correction output data");

//     // if(!laneEst.get()->publishData())
//     //     throw std::runtime_error("Unable to publish lane estimation output data");

//     // if(!freespace.get()->publishData())
//     //     throw std::runtime_error("Unable to load freespace model");

//     return true;
// }

// bool Perception::onPublishDataWithTimeStamp(std::vector<std::string> currTimeStamps)
// {
//     // if(!objDet.get()->publishDataWithTimeStamp(currTimeStamps))
//     //     throw std::runtime_error("Unable to publish object detection output data");

//     // if(!segMen.get()->publishDataWithTimeStamp(currTimeStamps))
//     //     throw std::runtime_error("Unable to publish segmentation output data");

//     // if(!maskCorrection.get()->publishDataWithTimeStamp(currTimeStamps))
//     //     throw std::runtime_error("Unable to publish mask correction output data");

//     // if(!laneEst.get()->publishDataWithTimeStamp(currTimeStamps))
//     //     throw std::runtime_error("Unable to publish lane estimation output data");

//     // if(!freespace.get()->publishDataWithTimeStamp(currTimeStamps))
//     //     throw std::runtime_error("Unable to load freespace model");

//     return true;
// }

bool Perception::onRender(std::vector<cv::Mat> imgBatch)
{
    // if(!objDet.get()->renderResults(&imgBatch))
    //     throw std::runtime_error("Unable to render object detection output");

    if(!segMen.get()->renderResults(&imgBatch))
        throw std::runtime_error("Unable to render segmentation output");
// 
    // if(!maskCorrection.get()->renderResults(&imgBatch))
        // throw std::runtime_error("Unable to render mask correction output");
// 
    // if(!laneEst.get()->renderResults(&imgBatch))
        // throw std::runtime_error("Unable to render lane estimation output");
// 
    // if(!freespace.get()->renderResults(&imgBatch))
        // throw std::runtime_error("Unable to render freespace output");

    return true;
}

// bool Perception::onRelease()
// { 
    
//     return true;
// }