docker build . -t grrt_container
xhost local:root & 
docker run -it --rm \
 --gpus all \
 --runtime=nvidia \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
 -v $(pwd):/workplace \
 grrt_container bash