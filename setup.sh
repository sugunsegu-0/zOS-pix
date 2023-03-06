#!/bin/bash

shopt -s extdebug
arg_count=$#

. ./scripts/color.sh
command -v figlet >/dev/null 2>&1 || { sudo apt install figlet > /dev/null; }

echo -e "${BCyan}"
figlet -ctf block Minus Zero
echo -e "${co}"

echo -e "${BIPurple}"
figlet -ctf standard Setting Up Your Environment
echo -e "${co}"

echo -en "${On_Red}"
cat >&2 << EOL
If you changed your mind just CANCEL it NOW. Please Run $BASH_SOURCE only Once.
If anything Goes wrong Just remove the ENV dir which is out of this dir.
EOL
echo -e "${co}"

# Intro
T=3
wait_s () {
    while [ $T -ne 0 ]
    do
        echo -en "${IBlue}\r"
        echo -en "Starting in $T \r"
        echo -en "${co}\r"
        sleep 1
        let "T-=1"
    done
}
wait_s

echo -e "${Green}"
cat >&2 << EOL
    +--------------------------------------------------------+
    |           Creation of Minuszero Environment            |
    |                  & Building Packages                   |
    +--------------------------------------------------------+
EOL
echo -e "${co}"

ROOTDIR=$(cd `dirname $BASH_SOURCE[0]`; pwd)
E="${ROOTDIR}/../ENV"

# Update & Upgrade
echo -e "${Yellow}"
cat >&2 << EOL
    +--------------------------------------------------------+
    |             Updating, Upgrading & Installing           |
    |               Required Dependencies.                   |
    +--------------------------------------------------------+
EOL
echo -e "${co}"
sudo apt -y update && sudo apt -y upgrade
sudo apt -y install build-essential cmake pkg-config unzip yasm git checkinstall \
		    libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev \
		    libswscale-dev libavresample-dev libgstreamer1.0-dev \
		    libgstreamer-plugins-base1.0-dev libxvidcore-dev x264 libx264-dev \
		    libfaac-dev libmp3lame-dev libtheora-dev libvorbis-dev \
		    libopencore-amrnb-dev libopencore-amrwb-dev \
		    libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils \
		    libgtk-3-dev libtbb-dev libatlas-base-dev gfortran freeglut3-dev \
            libglfw3-dev libglew-dev protobuf-compiler libprotobuf-dev \
            libeigen3-dev tree python3-pip figlet tmux \
            libc6 libcurl4 libgcc-s1 \
            libhdf5-103 libprotobuf17 \
            libqt5core5a libqt5gui5 \
            libqt5widgets5 libqt5svg5 \
            libstdc++6 sysstat ifstat \
            libqwt-qt5-6 libyaml-cpp0.6
pip3 install gdown

sudo add-apt-repository -y ppa:ecal/ecal-5.11
sudo apt-get -y install ecal

if [[ `!command tmux &> /dev/null` ]] 
then
	sudo apt -y install tmux &>/dev/null
	cd 
	if [[ -f ".tmux.conf" ]]
	then
		echo ".tmux.conf exists"
	else 
		cat <<- EOF > .tmux.conf
		unbind C-b
		set -g prefix C-Space
		bind Space send-prefix
		unbind '"'
		unbind %
		bind | split-window -h
		bind - split-window -v
		set -g mouse on
		EOF
	fi
    cd -
fi

echo -e "${BGreen}"
cat >&2 << EOL
    +--------------------------------------------------------+
    |             Updating, Upgrading & Installing           |
    |               Required Dependencies.                   |
    +--------------------------------------------------------+
EOL
echo -e "${co}"

# Env dir creation
echo -e "${Blue}"
cat >&2 << EOL
    +--------------------------------------------------------+
    |              Checking for 'ENV' Dir                    |
    +--------------------------------------------------------+
                                |
                                |
                                v
EOL

if [ -d "$E" ] 
then
    echo -e "${Green}"
    cat >&2 << EOL
    +--------------------------------------------------------+
    |                 'ENV' Directory present                |
    +--------------------------------------------------------+
EOL
    echo -e "${co}"
else
    echo -e "${Purple}"
    cat >&2 << EOL
    +--------------------------------------------------------+
    |  Creating a New 'ENV' directory & every dependency     |
    |                will be added there.                    |
    +--------------------------------------------------------+
EOL
    echo -e "${co}"
    mkdir $E
fi

ENV=${PWD%/*}/ENV
rm -rf ${ENV}/*

# --------------------* Functions Here *--------------------

# cmake function
cmake_f () {
    cmake -B BUILD . $*
    make -C BUILD -j`nproc` DESTDIR=../INSTALL install
}

success_build () {
echo -e "${Green}"
echo "+--------------------------------------------------------+"
echo "           $* Built & installed in INSTALL dir         "
echo "+--------------------------------------------------------+"
echo -e "${co}"
}

rm_all () {
    rm -rf $*
}

check_dir () {
    if [ -d "$*" ];then
        rm -rf $*
    fi
}

downloading () {
    echo -e "${Green}"
    echo "+--------------------------------------------------------+"
    echo "           $* is being Downloaded         "
    echo "+--------------------------------------------------------+"
    echo -e "${co}" 
}
# ------------------------- x - x - x ------------------------

# ------------------- xtensor dependencies -------------------
# xtl
function xtl {
    cd $ENV
    check_dir xtl
    git clone https://github.com/xtensor-stack/xtl.git
    cd xtl
    cmake_f
    if [ $? -eq 0 ] 
    then
        success_build xtl
    else
        echo -e "${On_Red}"
        cat >&2 << EOL
    +--------------------------------------------------------+
    |            * EXITING SOMETHING WENT WRONG *            |
    +--------------------------------------------------------+
EOL
        echo -e "${co}"

        exit 0
    fi
}

# xtensor
function xtensor {
    cd $ENV
    check_dir xtensor
    git clone https://github.com/xtensor-stack/xtensor.git
    cd xtensor
    cmake_f -DCMAKE_PREFIX_PATH=$ENV/xtl/BUILD
    if [ $? -eq 0 ] 
    then
        success_build xtensor
    else
        echo -e "${On_Red}"
        cat >&2 << EOL
    +--------------------------------------------------------+
    |            * EXITING SOMETHING WENT WRONG *            |
    +--------------------------------------------------------+
EOL
        echo -e "${co}"

        exit 0
    fi
}

# ------------------------- x - x - x ------------------------


# ------------------- CUDA TENSOR-RT CUDNN-11 -------------------
function cud_ten {
    cd $ENV

    downloading cuda
    gdown 1gRkq7bB0U4iSNEfpzHLAWjqeJTwsQj0x #cuda run file
    downloading TensorRT
    gdown 1QFMGpv4LL7TCxGRoFGOtv-hz6KlQVDbL #TensorRT
    downloading cudnn
    gdown 1JS0weDYRC3YTJJ8i4wy-ad8P4RA8fg-2 #cudnn-11

    echo -e "${Purple}"
        cat >&2 << EOL
    +--------------------------------------------------------+
    |  Following is the cuda run file, Accept the LICENSE    |
    |              & Install requires                        |
    +--------------------------------------------------------+
EOL
    echo -e "${co}"
    sudo sh cuda_11.6.2_510.47.03_linux.run

    tar -xvf cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz
    tar -xvf TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz
    rm cudnn-linux-x86_64-8.6.0.163_cuda11-archive.tar.xz TensorRT-8.4.3.1.Linux.x86_64-gnu.cuda-11.6.cudnn8.4.tar.gz

    #---------------- CUDA_DNN --------------------
    cd cudnn-linux-x86_64-8.6.0.163_cuda11-archive
    sudo cp -r ./include/* /usr/local/cuda-11.6/include/
    sudo cp -r ./lib/* /usr/local/cuda-11.6/lib64/

    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:$ENV/TensorRT-8.4.3.1/lib/" >> ~/.bashrc
    echo "export PATH=$PATH:/usr/local/cuda/bin:$ENV/TensorRT-8.4.3.1/bin/" >> ~/.bashrc
}

#---------------- x - x - x ---------------------

#---------------- Boost Libs --------------------
function boost {
    cd $ENV
    wget https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.bz2
    tar --bzip2 -xf boost*.bz2
    cd boost*/
    pwd
    ./bootstrap.sh
    sudo ./b2 install
}

#---------------- x - x - x ---------------------

#---------------- OpenCV --------------------

function opencv {
    cd /usr/include/linux
    sudo ln -s -f ../libv4l1-videodev.h videodev.h

    cd $ENV
    wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.2.zip
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.2.zip
    unzip opencv.zip
    unzip opencv_contrib.zip
    cd opencv-4.5.2

    cmake -B build . -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=$ENV \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D CUDA_ARCH_BIN="5.0 5.2 6.0 6.1 7.0 7.5 8.6" \
    -D WITH_V4L=ON \
    -D WITH_QT=ON \
    -D WITH_OPENGL=OFF \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D BUILD_PROTOBUF=ON \
    -D OPENCV_PYTHON3_INSTALL_PATH=/usr/lib/python3/dist-packages \
    -D PYTHON_EXECUTABLE=/usr/bin/python3 \
    -D OPENCV_EXTRA_MODULES_PATH=$ENV/opencv_contrib-4.5.2/modules/ \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D OpenGL_GL_PREFERENCE=GLVND ..

    make -C build -j`nproc`
    cd $ENV
    rm opencv*.zip
}

#---------------- x - x - x ---------------------

function aarch_64 {
    cd $ENV
    xtl
    xtensor
    boost
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh                  
    bash Miniconda3-latest-Linux-aarch64.sh
    conda activate
    cd $ENV
    git clone https://github.com/pytorch/pytorch.git
    cd pytorch/
    git checkout tags/v1.12.0 -b v1.12.0
    conda install astunparse numpy ninja pyyaml setuptools cmake cffi typing_extensions future six requests dataclasses
    export CMAKE_PREFIX_PATH=${CONDA_PREFIX:-"$(dirname $(which conda))/../"}
    export TORCH_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=1"
    python setup.py install
}

function x86 {
    xtl
    xtensor
    cud_ten
    boost 
    opencv
}

if [ `uname -m` == "x86_64" ]; then
    x86
    cat <<- EOF > ${ROOTDIR}/scripts/perception-env.sh
export ROOTDIR=$ROOTDIR
export xtl_DIR=$ENV/xtl/INSTALL/
export xtensor_DIR=$ENV/xtensor/INSTALL/
export TensorRT_DIR=$ENV/TensorRT-8.4.3.1/
export OpenCV_DIR=$ENV/opencv-4.5.2/build
EOF
elif [ `uname -m` == "aarch64" ]; then
    aarch_64
    cat <<- EOF > ${ROOTDIR}/scripts/perception-env.sh
export ROOTDIR=${ROOTDIR}
export xtl_DIR=$ENV/xtl/INSTALL/usr/local/share/cmake/
export xtensor_DIR=$ENV/xtensor/INSTALL/usr/local/share/cmake/
export Torch_DIR=\`python -c 'import torch;print(torch.utils.cmake_prefix_path)'\`
EOF
fi