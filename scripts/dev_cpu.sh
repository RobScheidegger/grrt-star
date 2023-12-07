docker build . -t grrt_container
docker run -it --rm \
 --gpus all \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
 -v $(pwd):/workplace \
 grrt_container bash