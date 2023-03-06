#pragma once
#ifndef DECODE_LABELS_HH
#define DECODE_LABELS_HH

#include <opencv2/core/cuda.hpp>

void batchedDecodeLabel(float* mask,  float* maskLane, float* maskFreeSapce,
                        float* image, const float* gpuBuffer, 
                        const int batchSize, const int width, 
                        const int height, const float* mean,
                        const float* stdDev);

#endif