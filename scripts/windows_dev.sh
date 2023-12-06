docker build . -t grrt_container
docker run -it -v "$(pwd)":/workplace grrt_container bash