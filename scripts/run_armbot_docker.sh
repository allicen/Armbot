#!/bin/bash

xhost +local:docker || true
docker run  -it --rm \
        -e "DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e XAUTHORITY \
        -v /dev:/dev \
        -v "$ARMBOT_PATH":/workspace \
       --net=host \
       --privileged \
       --name armbot armbot-img