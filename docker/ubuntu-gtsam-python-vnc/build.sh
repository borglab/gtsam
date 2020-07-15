# Build command for Docker image
# TODO(dellaert): use docker compose and/or cmake
# Needs to be run in docker/ubuntu-gtsam-python-vnc directory
docker build -t dellaert/ubuntu-gtsam-python-vnc:bionic .
