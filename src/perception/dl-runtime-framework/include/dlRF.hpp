#pragma once
#ifndef CORE_HPP
#define CORE_HPP

#include "cuda.h"
#include "cuda_runtime_api.h"
#include "NvInfer.h"
#include "NvInferRuntime.h"
#include "NvOnnxParser.h"
 
#include <iostream>
#include <vector>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <fstream>
#include <math.h>
#include <typeinfo>
#include <assert.h>
#include "perception_kernels.hpp"
#include <torch/torch.h>
#include <ATen/ATen.h>
#include "utils.hpp"
#include <thread>
#include <future>

using namespace cv;
using namespace cv::dnn;

#define NV_CUDA_CHECK(status)                                                                  \
{                                                                                              \
    if (status != 0)                                                                           \
    {                                                                                          \
        std::cout << "Cuda failure: " << cudaGetErrorString(status) << " in file " << __FILE__ \
                    << " at line " << __LINE__ << std::endl;                                   \
        abort();                                                                               \
    }                                                                                          \
}

// destroy TensorRT objects if something goes wrong
struct TRTDestroy
{
    template< class T >
    void operator()(T* obj) const
    {
        if (obj)
        {
            delete obj;
        }
    }
};

template< class T >
using TRTUniquePtr = std::unique_ptr< T, TRTDestroy >;

class Logger : public nvinfer1::ILogger
{
    public:
        void log(Severity severity, const char *msg) noexcept override {
            using namespace std;
            std::string s;
            switch (severity) {
                case Severity::kINTERNAL_ERROR:
                    s = "INTERNAL_ERROR";
                    break;
                case Severity::kERROR:
                    s = "ERROR";
                    break;
                case Severity::kWARNING:
                    s = "WARNING";
                    break;
                case Severity::kINFO:
                    s = "INFO";
                    break;
                case Severity::kVERBOSE:
                    s = "VERBOSE";
                    break;
            }
            cerr << s << ": " << msg << endl;
        }
};

enum {
    REC_BUF_UYVY=0,
    REC_BUF_RGB,
};


template<class pModelConfig>
class PerceptionModel : public DLRFUtils<pModelConfig>
{
    
    private:

    public:
        
        const char* modelEngineFile;
        const char* onnxModelFile;
        bool isEngine;
        bool isOnnx;;
        std::vector< uint32_t > inputIdxs;
        std::vector< uint32_t > outputIdxs;
        std::vector< nvinfer1::Dims > ioDims;
        std::vector<void*> m_DeviceBuffers;
        std::vector<void*> m_HostBuffers;
        TRTUniquePtr< nvinfer1::IBuilder > m_builder{nullptr};
        TRTUniquePtr< nvinfer1::INetworkDefinition > m_network{nullptr};
        TRTUniquePtr< nvonnxparser::IParser > m_parser{nullptr};
        TRTUniquePtr< nvinfer1::ICudaEngine > m_engine{nullptr};
        TRTUniquePtr< nvinfer1::IExecutionContext > m_context{nullptr};
        cudaStream_t m_model_cudaStream = nullptr;
        std::vector<cudaStream_t> internalDT_cudaStreams;
        std::vector<std::thread> preprocessThreads;
        pModelConfig modelConfig;
        at::Tensor imgBatchInPipeline;

        float* frameBatchGPU = nullptr;
        float* nchwBatch = nullptr;
        float* nhwcBatch = nullptr;
        float* meanGPU = nullptr;
        float* stdGPU = nullptr;
        
        PerceptionModel(){};

        PerceptionModel(const char* m_modelFile, bool m_isEngine, bool m_isOnnx);

        virtual ~PerceptionModel();

        inline const char* getModelEngineFile() { return modelEngineFile; };

        inline const char* getOnnxModelFile() { return onnxModelFile; };

        inline const std::vector< uint32_t > getInputIdxs() {return inputIdxs; };

        inline const std::vector< uint32_t > getOutputIdxs() { return outputIdxs; };

        inline const std::vector< nvinfer1::Dims > getDims() { return ioDims; };

        virtual bool loadModel();

        virtual bool preprocess(){};

        virtual bool postprocess() = 0;

        virtual bool runInference(){};

        virtual bool runInference(std::vector<void*>* inputs);

        virtual bool renderResults(){ return true; };

        virtual bool publishData(){ return true; };

        virtual bool layerPlugin(){ return true; };

        virtual bool builderCreation(){ return true; };

        virtual bool engineCreation(){ return true; };

        virtual bool modelCreation(){ return true; };

        virtual bool loggerCreation(){ return true; };

        virtual bool parseOnnxModel(){ return true; };

        virtual bool calculateMetrics(){ return true; };

        size_t getSizeByDim(const nvinfer1::Dims& dims);

        bool allocateBuffers();

};

template<class pModelConfig>
PerceptionModel<pModelConfig>::PerceptionModel(const char* m_modelFile, bool m_isEngine, bool m_isOnnx)
: isEngine(m_isEngine), isOnnx(m_isOnnx)
{
    if(isEngine) {
        modelEngineFile = m_modelFile;
    }
    else if(isOnnx) {
        onnxModelFile = m_modelFile;
    }
}

template<class pModelConfig>
PerceptionModel<pModelConfig>::~PerceptionModel()
{
    if (m_model_cudaStream) {
        cudaStreamDestroy(m_model_cudaStream);
    }
    for(auto internalDT_cs : internalDT_cudaStreams)
    {
        if (internalDT_cs) {
            cudaStreamDestroy(internalDT_cs);
        }
    }
}

template<class pModelConfig>
bool PerceptionModel<pModelConfig>::loadModel()
{
    std::ifstream file(modelEngineFile, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size))
        throw std::runtime_error("Unable to read engine file");
    std::cout << modelEngineFile << std::endl;
    Logger m_logger;
    std::unique_ptr<nvinfer1::IRuntime> m_runtime{nvinfer1::createInferRuntime(m_logger)};
    if (!m_runtime)
        throw std::runtime_error("Unable to create runtime");

    if (modelConfig.DLACore != -1)
    {
        m_runtime->setDLACore(modelConfig.DLACore);
        std::cout << "Runtime configured to use DLA Core - " << m_runtime->getDLACore() << std::endl;
    }
    else
    {
        std::cout << "Runtime not configured to use DLA Core. DLACore is given - " << modelConfig.DLACore << std::endl;
    }

    m_engine.reset(m_runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
    if (!m_engine)
        throw std::runtime_error("Unable to create runtime engine from input buffer");

    m_context.reset(m_engine->createExecutionContext());
    if (!m_context)
        throw std::runtime_error("Unable to create context from the created runtime engine");
    
    auto cudaRet = cudaStreamCreate(&m_model_cudaStream);
    if (cudaRet != 0)
        throw std::runtime_error("Unable to create model cuda stream");

    std::cout << "Loaded model from: " << modelEngineFile << std::endl;

    allocateBuffers();
    std::cout << "Allocated buffers for: " << modelEngineFile << std::endl << std::endl;

    for(int i=0; i<ioDims.size(); ++i)
    {
        cudaStream_t cs;
        internalDT_cudaStreams.push_back(cs);
        cudaStreamCreate(&(internalDT_cudaStreams.at(i)));
    }

    return true;
}

template<class pModelConfig>
bool PerceptionModel<pModelConfig>::runInference(std::vector<void*>* inputs)
{
    assert(inputs.size() == inputIdxs.size() && "Number of inputs passed and input indices present differ");
    auto start = std::chrono::system_clock::now();
    for(auto id : inputIdxs)
    {
        NV_CUDA_CHECK(cudaMemcpyAsync(m_DeviceBuffers.at(id),
                                    inputs->at(id),
                                    getSizeByDim(ioDims.at(id)) * sizeof(float),
                                    cudaMemcpyDeviceToDevice,
                                    internalDT_cudaStreams.at(id)));   
    }
    // std::cout << "Copied data" << std::endl;
    m_context->enqueueV2(m_DeviceBuffers.data(), m_model_cudaStream, nullptr );
    // std::cout << "Inference complete" << std::endl;
    // for(auto id : inputIdxs)
    // {
    //     NV_CUDA_CHECK(cudaMemcpyAsync(m_HostBuffers.at(id),
    //                                 inputs->at(id),
    //                                 getSizeByDim(ioDims.at(id)) * sizeof(float),
    //                                 cudaMemcpyHostToHost,
    //                                 internalDT_cudaStreams.at(id)));
    // }
    return true;
}

template<class pModelConfig>
bool PerceptionModel<pModelConfig>::allocateBuffers()
{
    auto numOfIO = m_engine->getNbBindings();
    m_DeviceBuffers.resize(numOfIO, nullptr);
    m_HostBuffers.resize(numOfIO, nullptr);
    ioDims.resize(numOfIO, nvinfer1::Dims{0, 0, 0});

    for(auto inputName : modelConfig.inputBlobNames)
    {
        auto idx = m_engine->getBindingIndex(inputName);
        inputIdxs.push_back(idx);
        ioDims.at(idx) = m_context->getBindingDimensions(idx);
    }
    for(auto outputName : modelConfig.outputBlobNames)
    {
        auto idx = m_engine->getBindingIndex(outputName);
        outputIdxs.push_back(idx);
        ioDims.at(idx) = m_context->getBindingDimensions(idx);
    }

    for(int i=0; i<inputIdxs.size(); ++i)
    {
        NV_CUDA_CHECK(cudaMalloc(&m_DeviceBuffers.at(inputIdxs.at(i)),
                                 getSizeByDim(ioDims.at(inputIdxs.at(i))) * sizeof(float)));

        NV_CUDA_CHECK(cudaMallocHost(&m_HostBuffers.at(inputIdxs.at(i)),
                                     getSizeByDim(ioDims.at(inputIdxs.at(i))) * sizeof(float)));
    }

    for(int j=0;j<outputIdxs.size(); ++j)
    {
        NV_CUDA_CHECK(cudaMalloc(&m_DeviceBuffers.at(outputIdxs.at(j)),
                                 getSizeByDim(ioDims.at(outputIdxs.at(j))) * sizeof(float)));

        NV_CUDA_CHECK(cudaMallocHost(&m_HostBuffers.at(outputIdxs.at(j)),
                                     getSizeByDim(ioDims.at(outputIdxs.at(j))) * sizeof(float)));
    }
    return true;
}

template<class pModelConfig>
size_t PerceptionModel<pModelConfig>::getSizeByDim(const nvinfer1::Dims& dims)
{
    int32_t size = 1;
    // std::cout << dims.d[0] << " ";
    for (int32_t i = 1; i < dims.nbDims; ++i)
    {
        size *= dims.d[i];
        // std::cout << dims.d[i] << " ";
    }
    // std::cout << std::endl;
    return size * modelConfig.batchSize;
}

#endif