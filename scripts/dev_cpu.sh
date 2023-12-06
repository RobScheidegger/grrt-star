docker build . -t grrt_container
docker run -it --rm -v "$(pwd)":/workplace grrt_container bash