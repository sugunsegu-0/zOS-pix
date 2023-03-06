#pragma once
#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include "perception_kernels.hpp"
#include "dlRF.hpp"
#include "perception/perceptionDS.hpp"
#include "npp.h"
#include "nppdefs.h"
#include "nppi_support_functions.h"
#include "nppi_color_conversion.h"

#ifdef RENDER_PERCEPTION_SEGMENTATION
#include "render/GLrender.hpp"
#endif

struct SegmentationConfig // Copy paste of Object detector. Change it when integrating segmentation
{
    int batchSize                                       = 3;
    double scaleFactor                                  = 1;
    int32_t inputChannelCount                           = 3;
    const int frameActualDim[2]                         = {1280, 720};
    const int inputBlobDims[2]                          = {640, 480};
    const int outputBlobDims[2]                         = {640, 480};
    int32_t outputChannelCount                          = 35;
    const float stdDev[3]                               = {58.395, 62.22, 65.025};
    const float mean[3]                                 = {123.675, 116.28, 103.53};
    bool swapRB                                         = false;
    bool crop                                           = false;
    int ddepth                                          = CV_32F;
    std::vector<const char*> inputBlobNames             = {"input"};
    std::vector<const char*> outputBlobNames            = {"output"};
    const float blend_ratio                             = 0.45;
    int DLACore                                         = 1;
    // int rec_buf_type                                    = REC_BUF_UYVY;
    int rec_buf_type                                    = REC_BUF_RGB;
    
    const uint8_t colors[35][3] = {
        {128,  64, 128},
        {250, 170, 160},
        { 81,   0,  81},
        {244,  35, 232},
        {230, 150, 140},
        {152, 251, 152},
        {220,  20,  60},
        {246, 198, 145},
        {255,   0,   0},
        {  0,   0, 230},
        {119,  11,  32},
        {255, 204,  54},
        {  0,   0, 142},
        {  0,   0,  70},
        {  0,  60, 100},
        {  0,   0,  90},
        {  0,   0, 110},
        {  0,  80, 100},
        {136, 143, 153},
        {220, 190,  40},
        {102, 102, 156},
        {190, 153, 153},
        {180, 165, 180},
        {174,  64,  67},
        {220, 220,   0},
        {250, 170,  30},
        {153, 153, 153},
        {153, 153, 153},
        {169, 187, 214},
        { 70,  70,  70},
        {150, 100, 100},
        {150, 120,  90},
        {107, 142,  35},
        { 70, 130, 180},
        {169, 187, 214}
    };
    std::vector<int> camIdxToRunOn                      = {0, 1, 2};
};

class Segmentation: public PerceptionModel<SegmentationConfig>
{
    private:
        float* maskCPU = nullptr;
        
        float* maskGPU = nullptr;
        float* maskLaneGPU = nullptr;
        float* maskFreeSpaceGPU = nullptr;
        float* meansGPU = nullptr;
        float* stdDevsGPU = nullptr;

        float* frameBatchGPU = nullptr;
        float* nchwBatch = nullptr;
        float* nhwcBatch = nullptr;
        float* meanGPU = nullptr;
        float* stdGPU = nullptr;

        void* receivedBufferGpuUYVY = nullptr;
        void* receivedBufferGpuRGB = nullptr;

        std::vector<cv::Mat> blendedFrameBatch;

        std::vector<cv::Mat> driveableRegionBatch;

        cudaStream_t m_cudaStream_datatransfers[3] = {nullptr, nullptr, nullptr};

        std::thread processingThreads[2] = {};

        #ifdef RENDER_PERCEPTION_SEGMENTATION
        void* receivedBufferCpuRGB = nullptr;
        std::shared_ptr<cv::Mat> stitchedFrame;
        #endif

    public:
        float* maskLaneCPU = nullptr;
        float* maskFreeSpaceCPU = nullptr;

        Segmentation(){};

        Segmentation(const char* m_modelFile, bool m_isEngine, bool m_isOnnx);

        ~Segmentation();

        bool loadModel() override;

        bool preprocess(uint8_t* buffer);

        bool runInference(uint8_t* buffer);

        bool allocatePostProcessingBuffers();

        bool transferDataToGPU();

        bool transferDataToCPU();

        bool postprocess() override;

        bool getOutput(SegmentationData* segmentationData);

        #ifdef RENDER_PERCEPTION_SEGMENTATION
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
            int height = int(modelConfig.outputBlobDims[1]);
            int width = int(modelConfig.outputBlobDims[0]);
            if(!glfwWindowShouldClose(window))
            {
                draw_frame(*(tmpFrame.get()), height, width);
                glfwSwapBuffers(window);
                glfwPollEvents();
                usleep(100);
            }
        }

        bool renderResults();
        #else
        bool renderResults(){};
        #endif

        inline bool getBlendedFrames(std::vector<cv::Mat>* blendedFrames){ *(blendedFrames) = blendedFrameBatch; return true; };
        inline bool getDrivableRegionFrames(std::vector<cv::Mat>* driveableRegionFrames){ *(driveableRegionFrames) = driveableRegionBatch; return true; };

};

#endif