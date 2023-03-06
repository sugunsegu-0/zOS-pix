#include <iostream>
#include <thread>
#include <future>
#include "globals.hpp"
#include "frameReader.hpp"
#include "perception.hpp"


int main(int argc, char *argv[])
{
    // CameraStack cameraStack = CameraStack(CAMERA_COUNT, 640, 480, WINDOW_NAMES, CAMERA_PORTS, VIDEOS, true);
    CameraStack cameraStack = CameraStack(CAMERA_COUNT, 640, 480, WINDOW_NAMES, VIDEOS, "Video");
    // CameraStack cameraStack = CameraStack(CAMERA_COUNT, 640, 480, WINDOW_NAMES, VIDEOS, "Folder");
    Perception perceptionStack;

    // Initializing Cameras
    {
        cameraStack.stackInit();
        perceptionStack.onInitialize(); 
    }

    // Running Cameras
    while(cameraStack.getFrameReadStatus())
    {
        
        cameraStack.stackRead();
        auto currFrames = cameraStack.getCurrFrames();
        auto currTimeStamps = cameraStack.getCurrTimeStamps();
        
        auto start = std::chrono::system_clock::now();
        perceptionStack.onProcess(currFrames);
        // perceptionStack.onPublishData();
        // perceptionStack.onRender(currFrames);
        // cameraStack.stackRender(); 
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end-start;
        std::cout << "Time to process last frame (seconds): " << diff.count() 
                  << " FPS: " << 1.0 / diff.count() << "\n";

        perceptionStack.onRender(currFrames);
        cameraStack.stackRender(); 
    }

    // perceptionStack.onRelease();
    return 0;
}