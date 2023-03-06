#pragma once
#ifndef PERCEPTION_CAMERA_HH
#define PERCEPTION_CAMERA_HH

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

#endif
