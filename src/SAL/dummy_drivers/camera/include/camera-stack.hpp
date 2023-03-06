#pragma once
#ifndef PERCEPTION_CAMERA_STACK_HPP
#define PERCEPTION_CAMERA_STACK_HPP

#include "camera.hpp"

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

#endif