#pragma once
#ifndef FRAME_READER_HH
#define FRAME_READER_HH

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <thread>
#include <future>
#include <string>
#include <bits/stdc++.h>
#include <glob.h>
#include "commons.hpp"

inline std::vector<std::string> glob(const std::string& pat){
    using namespace std;
    glob_t glob_result;
    glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
    std::vector<std::string> ret;
    for(unsigned int i=0;i<glob_result.gl_pathc;++i){
        ret.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return ret;
}

class PerceptionCamera
{
    public:

        PerceptionCamera();

        PerceptionCamera(uint32_t camId_m, uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m);

        PerceptionCamera(const char* modePath, uint32_t outputWidth_m, uint32_t outputHeight_m,
                         const char* windowName_m, const char* modeType_m);

        void setCommonProperties(uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m, const char* modeType_m);

        void setCamProperties(uint32_t camId_m, uint32_t outputWidth_m,
                              uint32_t outputHeight_m, const char* windowName);

        void setCamProperties(const char* modePath_m, uint32_t outputWidth_m, uint32_t outputHeight_m,
                              const char* windowName_m, const char* modeType_m);

        void startCam();

        bool readFrame(cv::Mat* frame);

        ~PerceptionCamera();

        inline const std::string getCurrTimeStamp(){ return currTimeStamp; }

        inline const uint32_t getCamId(){ return camId; }

        inline const char* getGstreamerPipelineConfig(){ return gstreamerPipelineConfig.c_str(); }

        void renderCam(cv::Mat currFrame, bool standAlone);

    private:
        void preprocessFrame(cv::Mat camFrame, cv::Mat* frame);

        uint32_t camId;
        const char* videoFile;
        const char* folderGlobPath;
        std::vector<std::string> imgPaths;
        uint32_t currImgIdx=0;
        std::string currTimeStamp;
        std::string currFilename;
        cv::VideoCapture frameCapturer;
        const char* modeType;
        uint32_t outputWidth;
        uint32_t outputHeight; 
        const char* renderWindowName;
        cv::Mat currFrame;
        std::string gstreamerPipelineConfig;
};

PerceptionCamera::PerceptionCamera()
{}

PerceptionCamera::PerceptionCamera(uint32_t camId_m, uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m)
: camId(camId_m), outputWidth(outputWidth_m), outputHeight(outputHeight_m), renderWindowName(windowName_m), modeType("Camera")
{    
    cv::namedWindow(renderWindowName, cv::WINDOW_AUTOSIZE);
    frameCapturer.open(camId);
}

PerceptionCamera::PerceptionCamera(const char* modePath_m, uint32_t outputWidth_m, uint32_t outputHeight_m,
                                   const char* windowName_m, const char* modeType_m)
: outputWidth(outputWidth_m), outputHeight(outputHeight_m), renderWindowName(windowName_m), modeType(modeType_m)
{    
    cv::namedWindow(renderWindowName, cv::WINDOW_AUTOSIZE);
    if (modeType == "Video")
    {
        videoFile = modePath_m;
        frameCapturer.open(videoFile);
    }
    else if (modeType == "Folder")
    {
        folderGlobPath = modePath_m;
        imgPaths = glob(folderGlobPath);
    }
    else
    {
        throw std::runtime_error("Mode type given not supported. Supported Mode Types are 'Camera', 'Video' and 'Folder'.");
    }
}

void PerceptionCamera::setCommonProperties(uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m, const char* modeType_m)
{
    modeType = modeType_m;
    outputWidth = outputWidth_m;
    outputHeight = outputHeight_m;
    renderWindowName = windowName_m;
    cv::namedWindow(renderWindowName, cv::WINDOW_AUTOSIZE);
}

void PerceptionCamera::setCamProperties(uint32_t camId_m, uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m)
{
    setCommonProperties(outputWidth_m, outputHeight_m, windowName_m, "Camera");
    frameCapturer.open(camId_m);
}

void PerceptionCamera::setCamProperties(const char* modePath_m, uint32_t outputWidth_m, uint32_t outputHeight_m, const char* windowName_m, const char* modeType_m)
{
    
    setCommonProperties(outputWidth_m, outputHeight_m, windowName_m, modeType_m);
    std::cout << modePath_m << std::endl;
    if (modeType_m == "Video")
    {
        videoFile = modePath_m;
        frameCapturer.open(modePath_m);
    }
    else if (modeType_m == "Folder")
    {
        folderGlobPath = modePath_m;
        imgPaths = glob(folderGlobPath);
    }
    else
        throw std::runtime_error("Mode type given not supported. Supported Mode Types are 'Camera', 'Video' and 'Folder'.");
}

void PerceptionCamera::startCam()
{
    if(modeType == "Camera")
    {
        frameCapturer.open(camId);
    }
    else if(modeType == "Video")
    {
        frameCapturer.open(videoFile);
    }
    else if (modeType == "Folder")
    {
        imgPaths = glob(folderGlobPath);
    }
}

bool PerceptionCamera::readFrame(cv::Mat* frame)
{
    if ((modeType == "Camera" || modeType == "Video") && frameCapturer.isOpened())
    {
        cv::Mat camFrame;
        frameCapturer >> camFrame;
        preprocessFrame(camFrame, frame);
        // cv::imshow("read frame output", *frame);
        // cv::waitKey(0);
        return true;
    }
    else if ((modeType == "Folder") && currImgIdx < imgPaths.size())
    {
        cv::Mat camFrame;
        camFrame = cv::imread(imgPaths.at(currImgIdx));
        std::stringstream ss(imgPaths.at(currImgIdx));
        std::string word;
        std::vector<std::string> tokenized_path;
        while (!ss.eof()) {
            getline(ss, word, '/');
            tokenized_path.push_back(word);
        }
        currFilename = tokenized_path.at(tokenized_path.size()-1);
        currTimeStamp = currFilename.substr(0, currFilename.length()-4);
        preprocessFrame(camFrame, frame);
        currImgIdx++;
        return true;
    }
    else
    {
        cv::Mat camFrame;
        camFrame = cv::Mat::zeros(cv::Size(outputWidth, outputHeight), CV_8UC3);
        preprocessFrame(camFrame, frame);
        return false;
    }
}

PerceptionCamera::~PerceptionCamera()
{
    frameCapturer.release();
    cv::destroyWindow(renderWindowName);
}

void PerceptionCamera::preprocessFrame(cv::Mat camFrame, cv::Mat* frame)
{
    cv::resize(camFrame, *frame, cv::Size(outputWidth, outputHeight), cv::INTER_CUBIC);
    // (*frame).convertTo(*frame, CV_32FC3, 1.0/255, 0);
    (*frame).convertTo(*frame, CV_32FC3, 1.0, 0);
}

void PerceptionCamera::renderCam(cv::Mat currFrame, bool standAlone=false)
{
    int alpha = 255;
    if (standAlone)
    {
        alpha = 1;
    }
    currFrame.convertTo(currFrame, CV_8UC3, alpha, 0);
    cv::imshow(renderWindowName, currFrame);
}


class CameraStack
{
    public:
        CameraStack(uint32_t cameraCount_m, uint32_t outputFrameWidth_m, uint32_t outputFrameHeight_m,
                    const char** windowName_m, uint32_t* cameraPorts_m);

        CameraStack(uint32_t cameraCount_m, uint32_t outputFrameWidth_m, uint32_t outputFrameHeight_m,
                    const char** windowName_m, const char** modePath_m, const char* modeType_m);

        ~CameraStack();

        void stackReserve();

        void stackInit();

        void stackRead();

        void stackRender(bool standAlone);

        inline const std::vector<cv::Mat> getCurrFrames() { return currFrameStack; };

        inline const std::vector<std::string> getCurrTimeStamps(){ return currTimeStamps; };

        inline const uint32_t getFrameReadStatus() { return std::accumulate(frameReadorNot.begin(), frameReadorNot.end(), 0);};

        inline const void getCurrFrameBlob(cv::Mat* outFrameBlob) { *outFrameBlob = frameBlob; };

    private:

        void blobPreProcess(std::vector<cv::Mat> imgBatch, cv::Mat* outputBlob, double scaleFactor, cv::Size blobDims, cv::Scalar mean, bool swapRB, bool crop, int ddepth);

        uint32_t cameraCount = 0;
        uint32_t frameWidth = 640;
        uint32_t frameHeight = 480;
        uint32_t* cameraPorts = nullptr;
        const char** videoFiles = nullptr;
        const char** imgFolderGlobPaths = nullptr;
        const char** windowNames = nullptr;
        const char* modeType;
        std::vector<cv::Mat> currFrameStack;
        std::vector<std::string> currTimeStamps;
        std::vector<PerceptionCamera> camStack;
        std::vector<bool> frameReadorNot;
        cv::Mat frameBlob;
        
};

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
                         const char** windowName_m, uint32_t* cameraPorts_m)
: cameraCount(cameraCount_m) ,frameWidth(outputFrameWidth_m), frameHeight(outputFrameHeight_m), windowNames(windowName_m), cameraPorts(cameraPorts_m), modeType("Camera") 
{
    stackReserve();
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

void CameraStack::blobPreProcess(std::vector<cv::Mat> imgBatch, cv::Mat* outputBlob, double scaleFactor, cv::Size blobDims, cv::Scalar mean, bool swapRB, bool crop, int ddepth)
{
    cv::dnn::blobFromImages(imgBatch, *outputBlob, scaleFactor, blobDims, mean, swapRB, crop, ddepth);
}

#endif
