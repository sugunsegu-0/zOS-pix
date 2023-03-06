#!/bin/bash

. ./scripts/color.sh

BUILD_DIR="`pwd`"
MAKE_ALL=make

CHECK_DIR="${BUILD_DIR##*/}"
shopt -s extdebug
arg_count=$#

if [[ `!command tmux &> /dev/null` ]] 
then
cd
	echo "-----------* Installing tmux *---------"
	sudo apt -y install tmux &>/dev/null
	echo "-------------* done * ------------"
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
else 
	echo "Checked tmux & Good to go"
fi

usage() {
	cat > &2 << EOL
	
EOL
}

#command -v tmux >/dev/null 2>&1 || { echo >&2 "tmux is not installed but required. Trying to install it..."; sudo apt install tmux; }

if [[ ( $@ == "--help") || $@ == "-h" ]];
then
	usage
	exit 0
fi

function check_cmd {
    if [ $* == 0 ]; then
        echo good
    else 
        exit 0
    fi
}

function run {
    S=mz
    tmux="tmux -2 -q"
    $tmux kill-server
    $tmux has-session -t $S
    if [ $? -eq 0 ]; then
            echo "Session $S already exists. Attaching to session."
            $tmux attach -t $S
            exit 0;
    fi

    # while true; do 

    # monitor
    $tmux new-session -d -s $S 'atop' # replace with jtop
    check_cmd $?

    # Perception
    # $tmux new-window -a -t $S 'taskset -c 0 ./a'     # segmentation
    # $tmux split-window -t 0 -h 'taskset -c 0 ./a'
    # $tmux split-window -t 1 -v 'taskset -c 0 ./a'
    # $tmux split-window -t 0 -v 'taskset -c 0 ./a'

    # # Mapping & Motion Planning
    # $tmux new-window -a -t $S 'taskset -c 0 ./a'
    # $tmux split-window -t 0 -h 'taskset -c 0 ./a'


    # # Localization 
    # $tmux new-window -a -t $S 'taskset -c 0 ./a'
    # $tmux split-window -t 0 -h 'taskset -c 0 ./a'
    $tmux attach -t $S
}

run