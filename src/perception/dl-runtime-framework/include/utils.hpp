#pragma once
#ifndef DLRF_UTILS_HPP
#define DLRF_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc/types_c.h>
#include <ATen/ATen.h>
#include <torch/torch.h>
#include "npp.h"
#include "nppdefs.h"
#include "nppi_support_functions.h"
#include "nppi_color_conversion.h"

enum {
    BUF_UINT8=0,
    BUF_INT8,
    BUF_INT16,
    BUF_INT32,
    BUF_INT64,
    BUF_FLOAT16,
    BUF_FLOAT32,
    BUF_FLOAT64,
};

template<class pModelConfig>
class DLRFUtils
{
    private:
    public:
        pModelConfig utilsConfig;
        DLRFUtils();
        bool convertBufferUYVY2RGB(Npp8u* uyvyBuf, Npp8u* rgbBuf);
        bool ResizeAndNormalizeImgTensor(at::Tensor& imgTensor);
        bool BufferToGPUTensor(void* buffer, at::Tensor& imgTensor, int batchSize, int width, int height, int channelCount, std::vector<int> idxs,
                            int buffer_type=BUF_UINT8, bool ownership=true, bool onDevice=false);
        bool BufferToNormalizedGPUTensor(void* buffer, at::Tensor& imgTensor, int batchSize, int width, int height, int channelCount, std::vector<int> idxs, 
                            int buffer_type=BUF_UINT8, bool ownership=true, bool onDevice=false);
        bool MatToGPUTensor(cv::Mat& img, at::Tensor& imgTensor, int batchSize, bool ownership=true);
        bool MatToNormalizedGPUTensor(cv::Mat& img, at::Tensor& imgTensor);
        bool SelectImagesFromNds(at::Tensor& imgTensor, std::vector<int> idxs);
};

template<class pModelConfig>
DLRFUtils<pModelConfig>::DLRFUtils()
{}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::convertBufferUYVY2RGB(Npp8u* uyvyBuf, Npp8u* rgbBuf)
{
    NppiSize roiSize = { utilsConfig.frameActualDim[0], utilsConfig.frameActualDim[1] };
    nppiCbYCr422ToRGB_8u_C2C3R(
        uyvyBuf,
        utilsConfig.frameActualDim[0] * 2 *sizeof(Npp8u),
        rgbBuf,
        utilsConfig.frameActualDim[0]* 3 * sizeof(Npp8u), 
        roiSize);
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::ResizeAndNormalizeImgTensor(at::Tensor& imgTensor)
{
    imgTensor = torch::nn::functional::interpolate(
        imgTensor,
        torch::nn::functional::InterpolateFuncOptions()
                .mode(torch::kBilinear)
                .size(std::vector<int64_t>({utilsConfig.inputBlobDims[1], utilsConfig.inputBlobDims[0]}))
                .align_corners(true)
    );
    imgTensor.index({at::indexing::Slice(), 0}).sub_(utilsConfig.mean[0]).div_(utilsConfig.stdDev[0]).mul_(utilsConfig.scaleFactor);
    imgTensor.index({at::indexing::Slice(), 1}).sub_(utilsConfig.mean[1]).div_(utilsConfig.stdDev[1]).mul_(utilsConfig.scaleFactor);
    imgTensor.index({at::indexing::Slice(), 2}).sub_(utilsConfig.mean[2]).div_(utilsConfig.stdDev[2]).mul_(utilsConfig.scaleFactor);
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::BufferToGPUTensor(
    void* buffer, at::Tensor& imgTensor, int batchSize,
    int width, int height, int channelCount, std::vector<int> idxs, 
    int buffer_type, bool ownership, bool onDevice)
{
    torch::Device device(torch::kCUDA, 0);
    torch::Device host(torch::kCPU);
    bool non_blocking=true;
    auto options = torch::TensorOptions();

    switch (buffer_type)
    {
        case BUF_UINT8:
            options = options.dtype(torch::kUInt8);
            break;
        case BUF_INT8:
            options = options.dtype(torch::kInt8);
            break;
        case BUF_INT16:
            options = options.dtype(torch::kInt16);
            break;
        case BUF_INT32:
            options = options.dtype(torch::kInt32);
            break;
        case BUF_INT64:
            options = options.dtype(torch::kInt64);
            break;
        case BUF_FLOAT16:
            options = options.dtype(torch::kFloat16);
            break;
        case BUF_FLOAT32:
            options = options.dtype(torch::kFloat32);
            break;
        case BUF_FLOAT64:
            options = options.dtype(torch::kFloat64);
            break;
        default:
            break;
    }

    if(onDevice)
        options = options.device(device);

    imgTensor = torch::from_blob(buffer, { batchSize, height, width, channelCount }, options);
    
    if (ownership)
        imgTensor = imgTensor.clone();

    if(!onDevice)
        imgTensor = imgTensor.to(device, non_blocking);

    imgTensor = imgTensor.to(at::kFloat);
    if (utilsConfig.swapRB)
    {
        imgTensor = imgTensor.flip(3);
    }
    imgTensor = imgTensor.permute({ 0, 3, 1, 2 });
    SelectImagesFromNds(imgTensor, idxs);
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::SelectImagesFromNds(at::Tensor& imgTensor, std::vector<int> idxs)
{
    auto idxsTensor = torch::tensor(idxs).to(torch::kCUDA);
    imgTensor = imgTensor.index_select(0, idxsTensor);
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::BufferToNormalizedGPUTensor(
    void* buffer, at::Tensor& imgTensor, int batchSize,
    int width, int height, int channelCount, std::vector<int> idxs,
    int buffer_type, bool ownership, bool onDevice)
{
    BufferToGPUTensor(buffer, imgTensor, batchSize, width, height,
                      channelCount, idxs, buffer_type, ownership, onDevice);
    ResizeAndNormalizeImgTensor(imgTensor);
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::MatToGPUTensor(cv::Mat& img, at::Tensor& imgTensor, int batchSize, bool ownership)
{
    torch::Device device(torch::kCUDA, 0);
    bool non_blocking=true;
    imgTensor = torch::from_blob(img.data, { 1, img.rows, img.cols, 3 }, at::kByte).to(device, non_blocking);

    if (ownership)
        imgTensor = imgTensor.clone();

    imgTensor = imgTensor.to(at::kFloat);
    if (utilsConfig.swapRB)
    {
        imgTensor = imgTensor.flip(3);
    }
    imgTensor = imgTensor.permute({ 0, 3, 1, 2 });
    return true;
}

template<class pModelConfig>
bool DLRFUtils<pModelConfig>::MatToNormalizedGPUTensor(cv::Mat& img, at::Tensor& imgTensor)
{
    MatToGPUTensor(img, imgTensor);
    ResizeAndNormalizeImgTensor(imgTensor);
    return true;
}

#endif