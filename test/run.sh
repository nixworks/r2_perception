#!/usr/bin/env bash

set -e

SCRIPTDIR=~/hansonrobotics/release/scripts

export NAME=robot
export SESSION=$(date +%H%M%S)
export ROSCONSOLE_FORMAT='[${logger}][${severity}] [${time}]: ${message}'
export OC_LOG_LEVEL=info        # error, warn, info, debug and fine

export OGRE_RTT_MODE=Copy # to maybe help against rviz crashes...
export LIBGL_ALWAYS_SOFTWARE=1

source ~/hansonrobotics/release/devel/setup.bash

# kill already running session
if [[ $(tmux ls 2>/dev/null) == ${NAME}* ]]; then
    tmux kill-session -t $NAME
    echo "Killed session $NAME"
fi

# prepare ROS arguments
ros_args="basedir:=$SCRIPTDIR name:=$NAME session:=$SESSION"

# start ROS master and wait for it
tmux new-session -n 'roscore' -d -s $NAME "roslaunch $SCRIPTDIR/robot.launch $ros_args; $SHELL"
until rostopic list >/dev/null 2>&1; do sleep 1; done

# start Blender
tmux new-window -n 'blender' "cd ~/hansonrobotics/sophia_blender_api && ./run.sh Sophia6.blend; $SHELL"

# start bash shell
tmux new-window -n 'bash' "cd $SCRIPTDIR; source ../devel/setup.bash; $SHELL"

tmux attach;
