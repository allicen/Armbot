xhost +local:docker || true
docker run  -ti --rm \
        -e "DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e XAUTHORITY \
        -v /dev:/dev \
        -v /home/e/ROS/Armbot:/workspace \
       --net=host \
       --privileged \
       --name armbot armbot-img