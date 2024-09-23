# GTSAM Containers

- container files to build images
- script to push images to a registry
- instructions to pull images and run containers

## Dependencies

- a container engine such as [`Docker Engine`](https://docs.docker.com/engine/install/)

## Pull from Docker Hub

Various GTSAM image configurations are available at [`docker.io/borglab/gtsam`](https://hub.docker.com/r/borglab/gtsam). Determine which [tag](https://hub.docker.com/r/borglab/gtsam/tags) you want and pull the image.

Example for pulling an image with GTSAM compiled with TBB and Python support on top of a base Ubuntu 22.04 image.

```bash
docker pull docker.io/borglab/gtsam:4.2.0-tbb-ON-python-ON_22.04
```

[`docker.io/borglab/gtsam-vnc`](https://hub.docker.com/r/borglab/gtsam-vnc) is also provided as an image with GTSAM that will run a VNC server to connect to.

## Using the images

### Just GTSAM

To start the image, execute

```bash
docker run -it borglab/gtsam:4.2.0-tbb-ON-python-OFF_22.04
```

after you will find yourself in a bash shell.

### GTSAM with Python wrapper

To use GTSAM via the python wrapper, similarly execute

```bash
docker run -it borglab/gtsam:4.2.0-tbb-ON-python-ON_22.04
```

and then launch `python3`:

```bash
python3
>>> import gtsam
>>> gtsam.Pose2(1,2,3)
(1, 2, 3)
```

### GTSAM with Python wrapper and VNC

First, start the image, which will run a VNC server on port 5900:

```bash
docker run -p 5900:5900 borglab/gtsam-vnc:4.2.0-tbb-ON-python-ON_22.04
```

Then open a remote VNC X client, for example:

#### Linux

```bash
sudo apt-get install tigervnc-viewer
xtigervncviewer :5900
```

#### Mac

The Finder's "Connect to Server..." with `vnc://127.0.0.1` does not work, for some reason. Using the free [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/), enter `0.0.0.0:5900` as the server.

## Build images locally

### Build Dependencies

- a [Compose Spec](https://compose-spec.io/) implementation such as [docker-compose](https://docs.docker.com/compose/install/)

### `gtsam` image

#### `.env` file

- `GTSAM_GIT_TAG`: [git tag from the gtsam repo](https://github.com/borglab/gtsam/tags)
- `UBUNTU_TAG`: image tag provided by [ubuntu](https://hub.docker.com/_/ubuntu/tags) to base the image off of
- `GTSAM_WITH_TBB`: to build GTSAM with TBB, set to `ON`
- `GTSAM_BUILD_PYTHON`: to build python bindings, set to `ON`
- `CORES`: number of cores to compile with

#### Build `gtsam` image

```bash
docker compose build
```

### `gtsam-vnc` image

#### `gtsam-vnc/.env` file

- `GTSAM_TAG`: image tag provided by [gtsam](https://hub.docker.com/r/borglab/gtsam/tags)

#### Build `gtsam-vnc` image

```bash
docker compose --file gtsam-vnc/compose.yaml build
```

## Push to Docker Hub

Make sure you are logged in via: `docker login docker.io`.

### `gtsam` images

Specify the variables described in the `.env` file in the `hub_push.sh` script.
To push images to Docker Hub, run as follows:

```bash
./hub_push.sh
```

### `gtsam-vnc` images

Specify the variables described in the `gtsam-vnc/.env` file in the `gtsam-vnc/hub_push.sh` script.
To push images to Docker Hub, run as follows:

```bash
./gtsam-vnc/hub_push.sh
```
