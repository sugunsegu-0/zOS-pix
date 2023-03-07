#!/bin/bash
export ROOTDIR=$PWD
cmake -B src/control/build \
    -S ${ROOTDIR}/src/control/ \
    -DROOTDIR=$ROOTDIR \

make -C ${ROOTDIR}/src/control/build
