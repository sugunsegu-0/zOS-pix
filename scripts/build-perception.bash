#!/bin/bash

mkdir -p ${ROOTDIR}/BUILD/perception
cmake -B ${ROOTDIR}/BUILD/perception \
    -S ${ROOTDIR}/src/perception/ \
    -DDEPLOYMENT=True \
    -DROOTDIR=$ROOTDIR \
    -DTensorRT_DIR=$TensorRT_DIR \
    -Dxtensor_DIR=$xtensor_DIR \
    -Dxtl_DIR=$xtl_DIR \
    -DTorch_DIR=$Torch_DIR \
    -DRENDER_PERCEPTION_OBJECT_DETECTION=True \
    -DRENDER_PERCEPTION_SEGMENTATION=False \
    -DRENDER_PERCEPTION_LANE_ESTIMATION=True \
    -DRENDER_PERCEPTION_FREESPACE=True

make -C ${ROOTDIR}/BUILD/perception -j`nproc`
