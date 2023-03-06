#pragma once
#ifndef OBJECT_DETECTION_HPP
#define OBJECT_DETECTION_HPP

#include "dlRF.hpp"
#include "perception/perceptionDS.hpp"
#include "perception_kernels.hpp"
#include <map>
#include <thread>
#include <future>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cmath>
// #include "torchvision/vision.h"
#include "npp.h"
#include "nppdefs.h"
#include "nppi_support_functions.h"
#include "nppi_color_conversion.h"

#ifdef RENDER_PERCEPTION_OBJECT_DETECTION
#include "render/GLrender.hpp"
#endif

using namespace cv;
using namespace cv::dnn;

enum {
    OUT_CCHW=0,
    OUT_XYXY,
};

struct ObjectDetectionConfig
{
    int batchSize                                       = 3;
    static constexpr float_t COVERAGE_THRESHOLD         = 0.6f;
    const uint32_t m_maxDetections                      = 1000U;
    const float_t m_nonMaxSuppressionOverlapThreshold   = 0.5;
    int32_t inputChannelCount                           = 3;
    const int frameActualDim[3]                         = {1280, 720};
    const int inputBlobDims[2]                          = {640, 640};

    // const int outputBlobDims[2]                         = {84, 8400};
    // const int outAnchorBoxes                            = 8400;
    // const int outCols                                   = 84; 
    // const int numOfClasses                              = 80;
    // const int bboxIdxStart                              = 0;
    // const int confidenceIdxInOut                        = -1;
    // const int classPredictionStartIdxInOut              = 4;
    // bool permuteOutput                                  = true;
    // // int outType                                         = OUT_XYXY;
    // int outType                                         = OUT_CCHW;
// 
    const int outputBlobDims[2]                         = {19, 8400};
    const int outAnchorBoxes                            = 8400;
    const int outCols                                   = 19; 
    const int numOfClasses                              = 15;
    const int bboxIdxStart                              = 0;
    const int confidenceIdxInOut                        = -1;
    const int classPredictionStartIdxInOut              = 4;
    bool permuteOutput                                  = true;
    // int outType                                         = OUT_XYXY;
    int outType                                         = OUT_CCHW;

    // const int outputBlobDims[2]                         = {25200, 20};
    // const int outAnchorBoxes                            = 25200;
    // const int outCols                                   = 20;
    // const int numOfClasses                              = 15;
    // const int bboxIdxStart                              = 0;
    // const int confidenceIdxInOut                        = 4;
    // const int classPredictionStartIdxInOut              = 5;
    // bool permuteOutput                                  = false;
    // int outType                                         = OUT_CCHW;
    
    // int rec_buf_type                                    = REC_BUF_UYVY;
    int rec_buf_type                                    = REC_BUF_RGB;
    double scaleFactor                                  = 0.00392156863;
    float stdDev[3]                                     = {1, 1 ,1};
    float mean[3]                                       = {0, 0 ,0};
    bool swapRB                                         = false;
    bool crop                                           = false;
    int ddepth                                          = CV_32F;
    
    float x_factor                                      = 1280 / 640.0;
    float y_factor                                      = 720 / 640.0;
    float NMS_SCORE_THRESHOLD                           = 0.10;
    int topK_nms                                        = 300;
    const float NMS_IOU_THRESHOLD                       = 0.45;
    const float NMS_ETA                                 = 1.f;
    std::vector<const char*> inputBlobNames             = {"images"};
    std::vector<const char*> outputBlobNames            = {"output0"};
    int DLACore                                         = -1;

    // Label Map
    // int lane_n[15] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    // const char* label_n[15] = {
    //     "car","bus","autorickshaw","vehicle fallback","truck","motorcycle",
    //     "rider","person","bicycle","animal","traffic sign","train","trailer",
    //     "traffic light","caravan"
    // };

    int lane_n[80] = {  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 
                        32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 
                        47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 
                        62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76,
                        77, 78, 79
                    };
    const char* label_n[80] = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", 
        "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", 
        "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", 
        "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", 
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", 
        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", 
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", 
        "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", 
        "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", 
        "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
    };

    std::vector<int> camIdxToRunOn                      = {0, 1, 2};
};

class ObjectDetection: public PerceptionModel<ObjectDetectionConfig>
{
    private:

        typedef std::pair<cv::Rect2f, float_t> BBoxConf;
        std::vector<std::vector<cv::Point>> m_left_top;
        std::vector<std::vector<float>> m_width;
        std::vector<std::vector<float>> m_height;
        std::vector<std::vector<cv::Point>> m_detectedCentroids;
        std::vector<std::vector<cv::Rect2i>> m_detectedBoxList;
        std::vector<std::vector<cv::Rect2f>> m_detectedBoxListFloat;
        std::vector<std::vector<int16_t>> classIds;
        std::vector<std::vector<float>> confidences;
        std::vector<std::vector<const char*>> labels;
        std::thread postProcessingThreads[5] = {};
        ObjectBox* filteredObjectsOutput;
        int* filteredObjectRowIds;
        int64_t* mask, keep, parent_object_index, num_to_keep;
        void* receivedBufferGpuUYVY = nullptr;
        void* receivedBufferGpuRGB = nullptr;

        bool m_isRaw = false;
        std::map<int, const char*> labelName;
        std::shared_ptr<cv::Mat> frame;
        
        #ifdef RENDER_PERCEPTION_OBJECT_DETECTION
        void* receivedBufferCpuRGB = nullptr;
        std::shared_ptr<cv::Mat> stitchedFrame;
        std::vector<cv::Mat> tmpMats;
        #endif

        float iou(float x1,float y1,float x2,float y2,float x3,float y3,float x4,float y4);

        std::vector<cv::Rect> update_boxes(std::vector<cv::Rect> m_detectedBoxList_u, std::vector<int16_t> label_l);

        bool interpretOutputPointerwithROI(float* out);

        bool outputPostProcessing(float* out, int frameId);

        void initialize_labelBoxMap();

    public:

        ObjectDetection(){};

        ObjectDetection(const char* m_modelFile, bool m_isEngine, bool m_isOnnx);

        ~ObjectDetection();

        bool loadModel() override;

        bool preprocess(uint8_t* buffer);

        bool runInference(uint8_t* buffer);

        bool postprocess() override;

        bool getOutput(ObjectDetectionData* objData);

        #ifdef RENDER_PERCEPTION_OBJECT_DETECTION
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