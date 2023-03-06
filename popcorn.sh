#!/bin/bash

. ./scripts/color.sh
sudo apt-get install figlet > /dev/null
figlet -c -t -f block Minus Zero

echo -e "${Green}"
cat >&2 << EOL
+--------------------------------------------------------+
|           Creation of Minuszero Environment            |
|                  & Building Packages                   |
+--------------------------------------------------------+
EOL
echo -e "${co}"

sudo apt -y update && sudo apt -y upgrade
echo
ROOTDIR=$(cd `dirname $BASH_SOURCE[0]`; pwd)
echo ${ROOTDIR}
ENV="${ROOTDIR}/env"
if [ -d "$ENV" ] 
then
    echo OK
else
    mkdir ${ROOTDIR}/env
    echo "Created Dir env"
fi

#-------------- xtensor dependencies -------------

cd $ENV
git clone https://github.com/xtensor-stack/xtl.git
cd xtl
mkdir build; cd build
cmake ..
echo "done ====== xtl"
# sudo make install
cd $ENV

git clone https://github.com/xtensor-stack/xtensor.git
cd xtensor; mkdir build;cd build
cmake -DCMAKE_PREFIX_PATH=$ENV/xtl/build/ ..
# sudo make install
cd $ENV

#---------------- x - x - x ---------------------

#********************************** GDRIVE **********************************
# cd $ENV
# gdown --folder https://drive.google.com/drive/folders/1NE8Hy4Zg8I1PtefJA6CQzUv7SK2MN65y

# gdown --id 1gRkq7bB0U4iSNEfpzHLAWjqeJTwsQj0x
# gdown --id 1QFMGpv4LL7TCxGRoFGOtv-hz6KlQVDbL
# gdown --id 1JS0weDYRC3YTJJ8i4wy-ad8P4RA8fg-2

# sudo chmod +x cuda_11.6.2_510.47.03_linux.run

# tar -xvf cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz
# tar -xvf TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

# rm cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

# echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:$ENV/TensorRT-8.4.3.1/lib/" >> ~/.bashrc
# echo "export PATH=$PATH:/usr/local/cuda/bin:$ENV/TensorRT-8.4.3.1/bin/" >> ~/.bashrc

#---------------- x - x - x ---------------------

#---------------- Boost Libs --------------------

cd $ENV
wget https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.bz2
tar --bzip2 -xf boost*.bz2
cd boost*/
pwd
./bootstrap.sh
sudo ./b2 install

#---------------- x - x - x ---------------------

##---------------- OpenCV --------------------

# sudo apt -y install build-essential cmake pkg-config unzip yasm git checkinstall \
# 		    libjpeg-dev libpng-dev libtiff-dev \
# 		    libavcodec-dev libavformat-dev libswscale-dev libavresample-dev \
# 		    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
# 		    libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev \
# 		    libfaac-dev libmp3lame-dev libvorbis-dev \
# 		    libopencore-amrnb-dev libopencore-amrwb-dev \
# 		    libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils \
# 		    libgtk-3-dev libtbb-dev libatlas-base-dev gfortran

# cd /usr/include/linux
# sudo ln -s -f ../libv4l1-videodev.h videodev.h

# cd $ENV
# wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip
# wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip
# unzip opencv.zip
# unzip opencv_contrib.zip
# cd opencv-4.5.2

# if [ -d "build" ]; then
# 	cd build
#     rm -rf *
# else
#     mkdir build
# fi
# cd build
# cmake -D CMAKE_BUILD_TYPE=RELEASE \
# -D CMAKE_INSTALL_PREFIX=$ENV \
# -D WITH_TBB=ON \
# -D ENABLE_FAST_MATH=1 \
# -D CUDA_FAST_MATH=1 \
# -D WITH_CUBLAS=1 \
# -D WITH_CUDA=ON \
# -D BUILD_opencv_cudacodec=OFF \
# -D WITH_CUDNN=ON \
# -D OPENCV_DNN_CUDA=ON \
# -D CUDA_ARCH_BIN="5.0 5.2 6.0 6.1 7.0 7.5 8.6" \
# -D WITH_V4L=ON \
# -D WITH_QT=ON \
# -D WITH_OPENGL=OFF \
# -D WITH_GSTREAMER=ON \
# -D OPENCV_GENERATE_PKGCONFIG=ON \
# -D OPENCV_PC_FILE_NAME=opencv.pc \
# -D OPENCV_ENABLE_NONFREE=ON \
# -D BUILD_PROTOBUF=ON \
# -D OPENCV_PYTHON3_INSTALL_PATH=/usr/lib/python3/dist-packages \
# -D PYTHON_EXECUTABLE=/usr/bin/python3 \
# -D OPENCV_EXTRA_MODULES_PATH=$ENV/opencv_contrib-4.5.2/modules/ \
# -D INSTALL_C_EXAMPLES=OFF \
# -D BUILD_EXAMPLES=OFF \
# -D OpenGL_GL_PREFERENCE=GLVND ..

# make -j`nproc`
# rm opencv*.zip

##---------------- x - x - x ---------------------

##---------------- PERCEPTION --------------------

# SRC_MAIN="${ROOTDIR}/src"
# PERC_DIR="${SRC_MAIN}/perception"
# echo $PERC_DIR
# echo
# cd $C_DIR
# echo $C_DIR

## ****************** COMMON BUILD ******************

# for f in "cuda-utils" "commons" "data-structures" "dl-runtime-framework"
# do
#     if [ -d "$f" ]; then
#         echo $f
#         cd $f; 
#         if [ -d "build" ]; then
#             rm -rf build
#         fi
#         mkdir build; cd build; cmake ../ -DCMAKE_PREFIX_PATH=$ENV/opencv-4.5.2/build/install \
#                                          -DROOTDIR=$ROOTDIR -DTensorRT_DIR=$ENV/TensorRT-8.4.3.1;
#         make -j24
#         cd ../../
#     fi
# done

## ****************** PERC_DEPS BUILD ******************

# cd ${PERC_DIR}/src
# pwd
# for f in *
# do
#     if [ -d "$f" ]
#     then
#         echo $f;
        
#         cd $f; 
#         if [ -d "build" ]; then
#             rm -rf build
#         fi
        cmake -B build . -DCMAKE_PREFIX_PATH=$ENV/opencv-4.5.2/build/install \
                                         -DROOTDIR=$ROOTDIR -DTensorRT_DIR=$ENV/TensorRT-8.4.3.1;
#         make -C build
#         cd ..
#     fi
# done

## ****************** PERCEPTION BUILD ******************

# cd $PERC_DIR
# pwd
# cd $C_DIR

# if [ -d "build" ]; then
#     rm -rf build
# fi
# cmake -B build . -DCMAKE_PREFIX_PATH="/home/minuszero/work/minuszero/env/opencv-4.5.2/build;/home/minuszero/work/_minus_cyber/minuszero/cyber/build/install/share/cmake/cyber" -DROOTDIR=$rootD -DTensorRT_DIR=$tRT;
# # cmake  -DCMAKE_PREFIX_PATH="/home/minuszero/work/minuszero/env/opencv-4.5.2/build" -DROOTDIR=$rootD -DTensorRT_DIR=$tRT;
# make -j24

##---------------- x - x - x ---------------------

##---------------- PROTO_ALL --------------------

# cd ${ROOTDIR}/proto
# cmake -B build .
# make -C build

##---------------- x - x - x ---------------------


##---------------- MAPPING --------------------

# cd ${SRC_MAIN}/mapping
# cmake -B build . -DCMAKE_PREFIX_PATH=${ROOTDIR}/env/opencv-4.5.2/
# make -C build -j24

##---------------- x - x - x ---------------------


##---------------- MOTION_PLANNING --------------------

# cd ${SRC_MAIN}/planning
# cmake -B build . -DCMAKE_PREFIX_PATH=${ROOTDIR}/env/opencv-4.5.2/
# make -C build -j24

##---------------- x - x - x ---------------------






















#--------------------Required current ------------------

# cmake -B build -S . -DROOTDIR=~/Desktop/pri_perc/minuszero -DTensorRT_DIR=~/TensorRT-8.4.3.1/ \
# -DCMAKE_PREFIX_PATH="/home/minuszero/work/mz_orig/env/opencv-4.5.2;/home/minuszero/xtensor/xtensor;/home/minuszero/work/_minus_cyber/minuszero/cyber/build/install/share/cmake/" \
# -DBUILD_ALL=True

# bash perc_build.sh ~/Desktop/pri_perc/minuszero/ ~/TensorRT-8.4.3.1/ 
# "/home/minuszero/work/mz_orig/env/opencv-4.5.2;
# /home/minuszero/xtensor/xtensor;
# /home/minuszero/work/_minus_cyber/minuszero/cyber/build/install/share/cmake/"
