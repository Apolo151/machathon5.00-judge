#!/bin/bash
USER_ID="$(id -u)"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
VOLUMES="--volume=$XSOCK:$XSOCK:rw
         --volume=$XAUTH:$XAUTH:rw"

xhost +
docker run \
    -it --rm \
    $VOLUMES \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --gpus all \
    --name="car_demo" \
    osrf/car_demo:$(git rev-parse --abbrev-ref HEAD)
xhost -
