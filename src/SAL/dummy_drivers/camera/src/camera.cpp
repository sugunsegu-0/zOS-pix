#include "camera.hpp"

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
}

void PerceptionCamera::renderCam(cv::Mat currFrame, bool standAlone=false)
{
    cv::imshow(renderWindowName, currFrame);
}