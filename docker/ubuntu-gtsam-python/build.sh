# Build command for Docker image
# TODO(dellaert): use docker compose and/or cmake
docker build --no-cache -t borglab/ubuntu-gtsam-python:bionic .
