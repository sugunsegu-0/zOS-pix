#include "camera-stack.hpp"

CameraStack::CameraStack(uint32_t cameraCount_m, uint32_t outputFrameWidth_m, uint32_t outputFrameHeight_m,
                         const char** windowName_m, uint32_t* cameraPorts_m)
: cameraCount(cameraCount_m) ,frameWidth(outputFrameWidth_m), frameHeight(outputFrameHeight_m), windowNames(windowName_m), cameraPorts(cameraPorts_m), modeType("Camera") 
{
    stackReserve();
}

void CameraStack::stackReserve()
{
    currFrameStack.reserve(cameraCount);
    camStack.reserve(cameraCount);
    frameReadorNot.reserve(cameraCount);
    for(uint32_t i=0; i<cameraCount; ++i)
    {
        frameReadorNot.push_back(1);
        cv::Mat frameInit;
        currFrameStack.push_back(frameInit);
    }
}

CameraStack::CameraStack(uint32_t cameraCount_m, uint32_t outputFrameWidth_m, uint32_t outputFrameHeight_m,
                         const char** windowName_m, const char** modePath_m, const char* modeType_m)
: cameraCount(cameraCount_m) ,frameWidth(outputFrameWidth_m), frameHeight(outputFrameHeight_m), windowNames(windowName_m), modeType(modeType_m)
{
    CameraStack::stackReserve();
    if (modeType == "Video")
    {
        videoFiles = modePath_m;
    }
    else if (modeType == "Folder")
    {
        imgFolderGlobPaths = modePath_m;
    }
    else
        throw std::runtime_error("Mode type given not supported. Supported Mode Types are 'Camera', 'Video' and 'Folder'.");

}

CameraStack::~CameraStack()
{}

void CameraStack::stackInit()
{
    for(uint32_t i=0; i<cameraCount; ++i)
    {
        if(modeType == "Video")
        {
            camStack.push_back(PerceptionCamera(*(videoFiles + i), frameWidth, frameHeight, *(windowNames + i), "Video"));
        }
        else if(modeType == "Folder")
        {
            // std::cout << "In Init - Folder" << std::endl;
            camStack.push_back(PerceptionCamera(*(imgFolderGlobPaths + i), frameWidth, frameHeight, *(windowNames + i), "Folder"));
        }
        else
        {
            camStack.push_back(PerceptionCamera(*(cameraPorts + i), frameWidth, frameHeight, *(windowNames + i)));
        }
    }
}

// void CameraStack::stackRead(double scaleFactor, cv::Scalar mean, bool swapRB, bool crop, int ddepth)
void CameraStack::stackRead()
{
    for(uint32_t i=0; i<cameraCount; ++i)
    {
        frameReadorNot.push_back(camStack.at(i).readFrame(&(currFrameStack.at(i)))); 
        if (modeType == "Folder")
        {
            currTimeStamps.push_back(camStack.at(i).getCurrTimeStamp());
        }
    }

    // blobPreProcess(currFrameStack, &frameBlob, scaleFactor, cv::Size(frameWidth, frameHeight), mean, swapRB, crop, ddepth);
    // std::cout << "Blob dim in Cam stack: " << frameBlob.size() << std::endl;   
}

void CameraStack::stackRender(bool standAlone=false)
{
    for(uint32_t i=0; i<cameraCount; ++i)
    {
        if(frameReadorNot.at(i))
        {
            camStack.at(i).renderCam(currFrameStack.at(i), standAlone);
        }
        else
        {
            std::cout << "Camera " << i << " not reading any frames!" << std::endl;
        }
    } 
    cv::waitKey(10);  
}