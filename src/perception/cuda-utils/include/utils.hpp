#pragma once
#ifndef CUDA_INTERNAL_UTILS_HH
#define CUDA_INTERNAL_UTILS_HH

int iDivUp(const int a, const int b);

struct ObjectBox
{
    float x,y,width,height,score,classIdx;

};

#endif