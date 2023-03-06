rootD=$1
tRT=$2
cPrePath=$3 
if [ $# -lt 2 ]; then
    echo "+-------------------------------------------------------------------+"
    echo "| ***** Please provide the ROOTDIR & TensorRT as follows *****      |"
    echo "|                                                                   |"
    echo "|     $0 /abs/path/to/rootdir/ /abs/path/to/TensorRT/    |"
    echo "|                                                                   |"
    echo "|                ********** EXITING **********                      |"
    echo "+-------------------------------------------------------------------+"

    exit 0
fi
C_DIR="$PWD"
cd $C_DIR/../
for f in "data-structures" "cuda-utils" "dl-runtime-framework"
# for f in "commons" "cuda-utils" "dl-runtime-framework"
do
    if [ -d "$f" ]; then
        echo $f
        cd $f; 
        if [ -d "build" ]; then
            rm -rf build
        fi
        mkdir build; cd build; cmake ../ -DCMAKE_CUDA_COMPILER:PATH=/usr/local/cuda/bin/nvcc \
                                         -DCMAKE_PREFIX_PATH=$cPrePath \
                                         -DROOTDIR=$rootD -DTensorRT_DIR=$tRT;
        make -j24
        cd ../../
    fi
done
cd $C_DIR
for f in *
do
    if [ -d "$f" ]
    then
        echo $f;
        
        cd $f; 
        if [ -d "build" ]; then
            rm -rf build
        fi
        mkdir build; cd build; cmake ../ -DCMAKE_CUDA_COMPILER:PATH=/usr/local/cuda/bin/nvcc \
                                         -DCMAKE_PREFIX_PATH=$cPrePath \
                                         -DROOTDIR=$rootD -DTensorRT_DIR=$tRT;
        make -j24
        cd ../../
    fi
done