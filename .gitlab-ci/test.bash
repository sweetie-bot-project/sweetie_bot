#!/bin/bash

log() {
    echo
    echo
    echo
    echo
    echo '#########################################################################################'
    echo "$3 (exit code: $1)"
    echo '#########################################################################################'
    cat "$2"
}

export USER=dev
source devel/setup.bash

# Launch ROS Core
roslaunch sweetie_bot_deploy load_param.launch 2>&1 > ros_core.log &
ROS_CORE=$!
sleep 40

# Create virtual display
export DISPLAY=:99
/usr/bin/Xvfb $DISPLAY -screen 0 1920x1080x24 &
VIRT_DISPLAY=$!

# Launch actual application
CMD_APP="roslaunch sweetie_bot_deploy flexbe_control.launch run_flexbe:=false"
$CMD_APP 2>&1 > app.log & APP=$!; sleep 100; scrot screenshot.png
kill -SIGINT $APP; wait $APP; EXIT_APP=$?; log $EXIT_APP app.log "$CMD_APP"

CMD_APP="roslaunch sweetie_bot_deploy joint_space_control.launch"
$CMD_APP 2>&1 > app.log & APP=$!; sleep 100; scrot screenshot-towr.png
kill -SIGINT $APP; wait $APP; EXIT_APP_TOWR=$?; log $EXIT_APP_TOWR app.log "$CMD_APP"

kill -SIGINT $ROS_CORE
wait $ROS_CORE
EXIT_CORE=$?

kill -SIGINT $VIRT_DISPLAY
wait $VIRT_DISPLAY
EXIT_VIRT_DISPLAY=$?

log $EXIT_CORE ros_core.log "roslaunch sweetie_bot_deploy load_param.launch"

EXIT="$(( $EXIT_APP + $EXIT_APP_TOWR + $EXIT_CORE + $EXIT_VIRT_DISPLAY ))"
if [[ "$EXIT" > 0 ]]; then
    exit "$EXIT"
fi

convert screenshot.png -crop 214x298+825+219 +repage cropped_screenshot.png
ERROR=$( compare -metric MSE reference.png cropped_screenshot.png diff.png 2>&1 | cut -f1 -d' ' | cut -f1 -d. )
echo "Image comparison score: $ERROR"
if (( "$ERROR" > 400 )); then
    exit 4
fi

convert screenshot-towr.png -crop 214x298+825+219 +repage cropped_screenshot.png
ERROR=$( compare -metric MSE reference.png cropped_screenshot.png diff-towr.png 2>&1 | cut -f1 -d' ' | cut -f1 -d. )
echo "Image comparison score: $ERROR"
if (( "$ERROR" > 400 )); then
    exit 4
fi
