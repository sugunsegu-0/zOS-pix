cmake -B build \
    -S . \
    -DCMAKE_PREFIX_PATH=`python -c 'import torch;print(torch.utils.cmake_prefix_path)'` \
    -DROOTDIR=$ROOTDIR \
    -DTensorRT_DIR=$TensorRT_DIR \
    -Dxtensor_DIR=$xtensor_DIR \
    -Dxtl_DIR=$xtl_DIR \
    -DTorch_DIR=$Torch_DIR \
    -DRENDER_PERCEPTION_OBJECT_DETECTION=True \
    -DRENDER_PERCEPTION_SEGMENTATION=True \
    -DRENDER_PERCEPTION_LANE_ESTIMATION=True \
    -DRENDER_PERCEPTION_FREESPACE=True

make -C build -j24