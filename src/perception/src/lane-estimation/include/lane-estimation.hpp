#pragma once
#ifndef LANE_ESTIMATION_HPP
#define LANE_ESTIMATION_HPP

#include "dlRF.hpp"
#include "perception/perceptionDS.hpp"

#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"
#include <xtensor/xadapt.hpp>
#include <xtensor/xsort.hpp>
#include <thread>
#include <future>

#include <string>
#include <vector>

#ifdef RENDER_PERCEPTION_LANE_ESTIMATION
#include "render/GLrender.hpp"
#endif

struct LaneEstimationConfig
{
    int batchSize                                       = 1;
    const float scaleFactor                             = 1;
    int32_t inputChannelCount                           = 3;
    const int frameActualDim[2]                         = {640, 480};
    const int inputBlobDims[2]                          = {640, 480};
    const float stdDev[3]                               = {58.395, 57.12, 57.375};
    const float mean[3]                                 = {123.675, 116.28, 103.53};
    bool swapRB                                         = true;
    bool crop                                           = false;
    int ddepth                                          = CV_32F;
    std::vector<const char*> inputBlobNames             = {"input"};
    std::vector<const char*> outputBlobNames            = {"curves", "logits"};
    float PRESENCE_THRESHOLD                            = 0.30f;
    std::vector<int> camIdxToRunOn                      = {0};
    int DLACore                                         = -1;
    int rec_buf_type                                    = REC_BUF_RGB;
};

class LaneEstimation: public PerceptionModel<LaneEstimationConfig>
{
    private:
        static constexpr uint32_t m_maxDetections                           = 2;
        static constexpr uint32_t pointsPerLine                             = 100;      
        typedef std::pair<std::vector<cv::Point>, float> laneConfPairVec;
        typedef std::vector<laneConfPairVec> lanesPerFrameVec;

        cv::Point2f t_detectedLanes[pointsPerLine];
        std::vector<lanesPerFrameVec> detectedLanes;
        std::vector<std::vector<int>> detectedLaneTypes;
        std::unique_ptr<uint32_t> sortedPresence[5];
        std::thread postProcessingThreads[5] = {};

        #ifdef RENDER_PERCEPTION_LANE_ESTIMATION
        std::vector<cv::Mat> tmpMats; 
        std::shared_ptr<cv::Mat> stitchedFrame;
        #endif

    public:

        LaneEstimation(){};

        LaneEstimation(const char* m_modelFile, bool m_isEngine, bool m_isOnnx);

        ~LaneEstimation();

        bool loadModel() override;

        inline const uint32_t getMaxDetectionsCount() { return m_maxDetections; };

        bool preprocess(float* buffer);

        bool runInference(float* buffer);

        bool postprocess() override;

        bool getOutput(LaneEstimationData* lanesEstData);

        #ifdef RENDER_PERCEPTION_LANE_ESTIMATION
        GLFWwindow* window;

        void initialize_render(int window_width,int window_height)
        {
            cout << "Video width: " << window_width << endl;
            cout << "Video height: " << window_height << endl;
            glfwSetErrorCallback(error_callback);

            if (!glfwInit()) {
                exit(EXIT_FAILURE);
            }

            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
            window = glfwCreateWindow(window_width, window_height, "Simple example", NULL, NULL);

            if (!window) {
                glfwTerminate();
                exit(EXIT_FAILURE);
            }

            glfwSetKeyCallback(window, key_callback);
            glfwSetWindowSizeCallback(window, resize_callback);
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);

            //  Initialise glew (must occur AFTER window creation or glew will error)
            GLenum err = glewInit();
            if (GLEW_OK != err)
            {
                cout << "GLEW initialisation error: " << glewGetErrorString(err) << endl;
                exit(-1);
            }
            cout << "GLEW okay - using version: " << glewGetString(GLEW_VERSION) << endl;

            init_opengl(window_width, window_height);
        }

        void Render()
        {
            std::shared_ptr<cv::Mat> tmpFrame = stitchedFrame;
            int height = int(modelConfig.frameActualDim[1]/modelConfig.batchSize);
            int width = int(modelConfig.frameActualDim[0]);
            if(!glfwWindowShouldClose(window))
            {
                draw_frame(*(tmpFrame.get()), height, width);
                glfwSwapBuffers(window);
                glfwPollEvents();
                usleep(100);
            }
        }

        bool renderResults(std::vector<cv::Mat>* frameBatch);
        #else
        bool renderResults(){};
        #endif

        void softmax(float* input, size_t size);

        std::vector<cv::Point> lanePointCalc(cv::Point2f(&lanePoints)[pointsPerLine], int* laneType,
                                    float k, float f, float m, float n, float b1, float b2,
                                    float lowerBoundY, float upperBoundY);

        void interpretOutputPointer(const float* logitsOut, const float* curvesOut);

        void postprocessOutputPerFrame( lanesPerFrameVec* detectedLanesPerFrame,
                                        std::vector<int>* detectedLaneTypesPerFrame,
                                        uint32_t* sortedPresencePerFrame,
                                        const float* logitsOut, uint32_t logitsOutPerFrame,
                                        const float* curvesOut, uint32_t curvesOutPerFrame, 
                                        uint32_t cameraIdx);

};

#endif