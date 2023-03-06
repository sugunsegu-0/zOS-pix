#!/bin/bash

. ./scripts/color.sh
source ./scripts/perception-env.bash
echo $ROOTDIR

echo -e "${BCyan}"
figlet -ctf block Minus Zero
echo -e "${co}"

shopt -s extdebug
arg_count=$#

ROOTDIR=$(cd `dirname $BASH_SOURCE[0]`; pwd)
SRC_MAIN="${ROOTDIR}/src"

echo -e "${BPurple}"
figlet -ctf standard zOS Build
echo -e "${co}"

wait_s() {
    T=3
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
build_start() {
    echo -e "${BPurple}"
    echo
    echo "------------------------------------------"
    echo "          STARTED Building dir $*"
    echo "------------------------------------------"
    echo 
    echo -e "${co}"
}

build_done() {
    echo -e "${Green}"
    echo 
    echo "------------------------------------------"
    echo "          Done Building dir $*"
    echo "------------------------------------------"
    echo 
    echo -e "${co}"
}

DIR_BUILD="commons perception control localization planning mapping SAL/camera vehicleio"
function all {
    for f in ${DIR_BUILD}
    do
        if [ "$f" = "perception" ];then
            build_start $f
            bash ${HOME}/work/zOS/scripts/build-perception.bash    
            build_done $f
        elif [ "$f" = "vehicleio" ];then
            build_start $f
            cd $SRC_MAIN/$f;
            mkdir -p ${ROOTDIR}/BUILD/$f
            cmake -B ${ROOTDIR}/BUILD/$f .  \
            -DROOTDIR=${ROOTDIR} \
            -DCMAKE_PREFIX_PATH=${SRC_MAIN}/vehicleio/socketcan-cpp/install/lib/cmake/socketcan_cpp
            make -C ${ROOTDIR}/BUILD/$f -j`nproc`
            build_done $f
        else
            build_start $f
            cd $SRC_MAIN/$f;
            mkdir -p ${ROOTDIR}/BUILD/$f
            cmake -B ${ROOTDIR}/BUILD/$f . -DROOTDIR=${ROOTDIR}
            make -C ${ROOTDIR}/BUILD/$f -j`nproc`
            build_done $f
        fi
    done
}

check_make() {
	if [[ $arg_count -lt 1 ]]; then
			all
	elif [[ $arg_count -ge 1 ]]; then
		for f in $@
		do
            if [ "$f" = "perception" ];then
                build_start $f
                bash ${HOME}/work/zOS/scripts/build-perception.bash    
                build_done $f
            elif [ "$f" = "vehicleio" ];then
                build_start $f
                cd $SRC_MAIN/$f;
                mkdir -p ${ROOTDIR}/BUILD/$f
                cmake -B ${ROOTDIR}/BUILD/$f .  \
                -DROOTDIR=${ROOTDIR} \
                -DCMAKE_PREFIX_PATH=${SRC_MAIN}/vehicleio/socketcan-cpp/install/lib/cmake/socketcan_cpp
                make -C ${ROOTDIR}/BUILD/$f -j`nproc`
                build_done $f
            else
                build_start $f
                cd $SRC_MAIN/$f;
                mkdir -p ${ROOTDIR}/BUILD/$f
                cmake -B ${ROOTDIR}/BUILD/$f . -DROOTDIR=${ROOTDIR}
                make -C ${ROOTDIR}/BUILD/$f -j`nproc`
                build_done $f
            fi
		done
	fi
}

check_make $*