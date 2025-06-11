#!/bin/bash

docker rm -f pi_container 2>/dev/null || true
xhost +local:docker

docker run -it \
  --privileged \
  --net=host \
  --name="pi_container" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device /dev:/dev \
  --env=NVIDIA_VISIBLE_DEVICES=all \
  --env=NVIDIA_DRIVER_CAPABILITIES=all \
  ros_pi_docker \
  /bin/bash
