export ROOTDIR=$PWD
# export TensorRT_DIR=/home/ade/minuszero/env/TensorRT-8.4.3.1
# export TensorRT_DIR=/home/minuszero/TensorRT/install
export xtl_DIR=~/work/ENV/xtl/INSTALL/usr/local/share/cmake/
export xtensor_DIR=~/work/ENV/xtensor/INSTALL/usr/local/share/cmake/
# export Thrust_DIR=/home/minuszero/env/thrust/install/thrust/cmake
# export Torch_DIR=/home/minuszero/libtorch/install/share/cmake/Torch
export Torch_DIR=`python -c 'import torch;print(torch.utils.cmake_prefix_path)'`
# export TorchVision_DIR=/home/minuszero/vision/latest/install/share/cmake/TorchVision
# export OpenCV_DIR=/home/minuszero/opencv-4.5.2/build
