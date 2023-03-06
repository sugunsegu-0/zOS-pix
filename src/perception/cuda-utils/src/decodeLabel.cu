#include "utils.hpp"
#include "decodeLabel.hpp"
#include <cuda.h>
#include <stdexcept>
#include <iostream>
#include <stdio.h>
#include <limits>

// clang-format off
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// clang-format on

__global__ void decodeLabelKernel(float* mask, float* maskLane, float* maskFreeSpace, float* image, const float* label,
                                const uint32_t width, const uint32_t height, float maxVal, const float* mean,
                                const float* stdDev)
{
    
    const uint8_t colors[36][3] = {
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
        {169, 187, 214},
        {0, 0, 0},
    };

    
    const uint32_t tidx = blockDim.x * blockIdx.x + threadIdx.x;
    const uint32_t tidy = blockDim.y * blockIdx.y + threadIdx.y;

    if (tidx >= width || tidy >= height)
        return;

    int maxIdx = -1;
    uint32_t startPos = (tidy * width) + tidx;
    for(int i=0; i<35; ++i)
    {
        // // (35, 480, 640)
        if (label[startPos + (i * height * width)] > maxVal)
        {
            maxIdx = i;
            maxVal = label[startPos + (i * height * width)];
            // printf(" %f ", maxVal);
        }
    }

    mask[tidy * width * 3 + 3 * tidx + 0] = colors[maxIdx][0];
    mask[tidy * width * 3 + 3 * tidx + 1] = colors[maxIdx][1];
    mask[tidy * width * 3 + 3 * tidx + 2] = colors[maxIdx][2];

    // preparing blended image for lane estimation
    float blend_ratio = 0.45;
    maskLane[tidy * width * 3 + 3 * tidx + 0] = (stdDev[0]*image[startPos + 0 * height * width] + mean[0]) * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 0] * (1 - blend_ratio);
    maskLane[tidy * width * 3 + 3 * tidx + 1] = (stdDev[1]*image[startPos + 1 * height * width] + mean[1]) * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 1] * (1 - blend_ratio);
    maskLane[tidy * width * 3 + 3 * tidx + 2] = (stdDev[2]*image[startPos + 2 * height * width] + mean[2]) * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 2] * (1 - blend_ratio);

    // maskLane[tidy * width * 3 + 3 * tidx + 0] = image[startPos + 0 * height * width] * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 0] * (1 - blend_ratio);
    // maskLane[tidy * width * 3 + 3 * tidx + 1] = image[startPos + 1 * height * width] * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 1] * (1 - blend_ratio);
    // maskLane[tidy * width * 3 + 3 * tidx + 2] = image[startPos + 2 * height * width] * blend_ratio + mask[tidy * width * 3 + 3 * tidx + 2] * (1 - blend_ratio);

    // preparing color mask for free space
    if (maxIdx != 0)
        maxIdx = 35;

    maskFreeSpace[tidy * width * 3 + 3 * tidx + 0] = colors[maxIdx][0];
    maskFreeSpace[tidy * width * 3 + 3 * tidx + 1] = colors[maxIdx][1];
    maskFreeSpace[tidy * width * 3 + 3 * tidx + 2] = colors[maxIdx][2];
    
}

void batchedDecodeLabel(float* mask,  float* maskLane, float* maskFreeSpace, float* image, 
                        const float* gpuBuffer, const int batchSize, 
                        const int width, const int height, const float* mean,
                        const float* stdDev)
{
    for(int i=0; i < batchSize; ++i)
    {
        float maxVal = std::numeric_limits<float>::min();
        auto maskCUDA_ptr = mask + i*width*height*3;
        auto maskCUDA_Lane_ptr = maskLane + i*width*height*3;
        auto maskCUDA_FreeSpace_ptr = maskFreeSpace + i*width*height*3;
        auto image_ptr = image + i*width*height*3;
        auto gpuBuffer_ptr = gpuBuffer + i*width*height*35;

        dim3 numThreads = dim3(32, 32);

        // std::cout << "IN" << std::endl;

        decodeLabelKernel<<<dim3(iDivUp(width, numThreads.x),
                                 iDivUp(height, numThreads.y)),
                            numThreads>>>(maskCUDA_ptr,
                                        maskCUDA_Lane_ptr,
                                        maskCUDA_FreeSpace_ptr,
                                        image_ptr,
                                        gpuBuffer_ptr,
                                        width,
                                        height, 
                                        maxVal,
                                        mean,
                                        stdDev);
    }
}

// clang-format on
#pragma GCC diagnostic pop
// clang-format off