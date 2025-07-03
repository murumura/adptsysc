
#!/bin/bash
container_id=$(docker ps -aq --filter name=sysc_container)
if [ ! -z "$container_id" ]
then
    docker rm ${container_id}
fi
# Expose the X server on the host.
sudo xhost +local:root
# --rm: Make the container ephemeral (delete on exit).
# -it: Interactive TTY.
# --gpus all: Expose all GPUs to the container.
docker run \
  --rm \
  -it \
  -v $(pwd):/sysc \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --name sysc_container \
  --privileged \
  sysc-dev \
  bash
