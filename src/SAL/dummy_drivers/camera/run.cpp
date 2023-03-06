#include "camera-stack.hpp"
#include "globals.hpp"
#include <ecal/ecal.h>

int main(int argc, char *argv[])
{
    std::cout << "Inside Main" << std::endl;
    eCAL::Initialize(argc, argv, "multicam");
    eCAL::CPublisher publisher("multicam");
    auto var = FRAME_WIDTH * FRAME_HEIGHT * 3 * sizeof(uint8_t);
    auto ptr = (uint8_t*)malloc(CAMERA_COUNT * FRAME_WIDTH * FRAME_HEIGHT * 3 * sizeof(uint8_t));
    CameraStack cameraStack = CameraStack(CAMERA_COUNT, FRAME_WIDTH, FRAME_HEIGHT, WINDOW_NAMES, VIDEOS, "Video");
    
    cameraStack.stackInit();

    while(cameraStack.getFrameReadStatus())
    {
        cameraStack.stackRead();
        auto currFrames = cameraStack.getCurrFrames();
        auto currTimeStamps = cameraStack.getCurrTimeStamps();
        for(int i=0; i<currFrames.size(); i++){
            memcpy(ptr + (i * var), currFrames[i].data, var);
        }
        publisher.Send(ptr, CAMERA_COUNT * FRAME_WIDTH * FRAME_HEIGHT * 3 * sizeof(uint8_t));
        // cameraStack.stackRender(true);
    }

    return 0;
}