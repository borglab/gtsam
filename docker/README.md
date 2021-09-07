# Instructions

# Images on Docker Hub

There are 4 images available on https://hub.docker.com/orgs/borglab/repositories:

- `borglab/ubuntu-boost-tbb`: 18.06 Linux (nicknamed `bionic`) base image, with Boost and TBB installed.
- `borglab/ubuntu-gtsam`: GTSAM Release version installed in `/usr/local`.
- `borglab/ubuntu-gtsam-python`: installed GTSAM with python wrapper.
- `borglab/ubuntu-gtsam-python-vnc`: image with GTSAM+python wrapper that will run a VNC server to connect to.

# Using the images

## Just GTSAM

To start the Docker image, execute
```bash
docker run -it borglab/ubuntu-gtsam:bionic 
```
after you will find yourself in a bash shell, in the directory `/usr/src/gtsam/build`.
## GTSAM with Python wrapper

To use GTSAM via the python wrapper, similarly execute
```bash
docker run -it borglab/ubuntu-gtsam-python:bionic 
```
and then launch `python3`:
```bash
python3
>>> import gtsam
>>> gtsam.Pose2(1,2,3)
(1, 2, 3)
```

## GTSAM with Python wrapper and VNC

First, start the docker image, which will run a VNC server on port 5900:
```bash
docker run -p 5900:5900 borglab/ubuntu-gtsam-python-vnc:bionic
```

Then open a remote VNC X client, for example:

### Linux
```bash
sudo apt-get install tigervnc-viewer
xtigervncviewer :5900
```
### Mac
The Finder's "Connect to Server..." with `vnc://127.0.0.1` does not work, for some reason. Using the free [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/), enter `0.0.0.0:5900` as the server.

# Re-building the images locally

To build all docker images, in order:

```bash
(cd ubuntu-boost-tbb && ./build.sh)
(cd ubuntu-gtsam && ./build.sh)
(cd ubuntu-gtsam-python && ./build.sh)
(cd ubuntu-gtsam-python-vnc && ./build.sh)
```

Note: building GTSAM can take a lot of memory because of the heavy templating. It is advisable to give Docker enough resources, e.g., 8GB, to avoid OOM errors while compiling.