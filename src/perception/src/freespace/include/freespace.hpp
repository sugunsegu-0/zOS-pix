#pragma once
#ifndef FREESPACE_HPP
#define FREESPACE_HPP

#include "perception_kernels.hpp"
#include "dlRF.hpp"
#include "perception/perceptionDS.hpp"

#ifdef RENDER_PERCEPTION_FREESPACE
#include "render/GLrender.hpp"
#endif

struct FreeSpaceConfig // Copy paste of Object detector. Change it when integrating segmentation
{
    int batchSize                                       = 3;
    const float scaleFactor                             = 1;
    int32_t inputChannelCount                           = 3;
    const int frameActualDim[2]                         = {640, 480};
    const int inputBlobDims[2]                          = {640, 480};
    const int outputBlobDims[2]                         = {640, 480};
    int32_t outputChannelCount                          = 26;
    const float mean[3]                                 = {123.675, 116.28, 103.53};
    const float stdDev[3]                               = {58.395, 57.12, 57.375};
    bool swapRB                                         = false;
    bool crop                                           = false;
    int ddepth                                          = CV_32F;
    double low_thresh                                   = 50;
    double high_thresh                                  = 100;
    int apperture_size                                  = 3;
    bool L2gradient                                     = false;
    std::vector<const char*> inputBlobNames             = {"None"};
    std::vector<const char*> outputBlobNames            = {"None"};
    std::vector<int> camIdxToRunOn                      = {0, 1, 2};
    float contourAreaFilter                             = 10000;
    int DLACore                                         = -1;
    int rec_buf_type                                    = REC_BUF_RGB;
};

class FreeSpace: public PerceptionModel<FreeSpaceConfig>
{
    private:
        std::vector<cv::Mat> outputBatch;
        std::vector<std::vector<std::vector<cv::Point>>> frameFreeSpacePoints;
        std::vector<std::vector<cv::Vec4i>> frameHierarchy;

        #ifdef RENDER_PERCEPTION_FREESPACE
        std::vector<cv::Mat> tmpMats;
        std::shared_ptr<cv::Mat> stitchedFrame;
        #endif
    public:

        FreeSpace();

        FreeSpace(const char* m_modelFile, bool m_isEngine, bool m_isOnnx){};

        ~FreeSpace();

        bool loadModel() override;

        bool preprocess(float* buffer);

        bool runInference(float* buffer);

        bool postprocess() override;

        bool getOutput(FreeSpaceData* freeSpaceData);

        #ifdef RENDER_PERCEPTION_FREESPACE
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

};

#endif