# GTSAM Docker Images

The official Docker images for GTSAM are maintained in the [borglab/docker-images](https://github.com/borglab/docker-images) repository.

## Available Images

The following images are available on Docker Hub, primarily under the `borglab` namespace:

-   **[borglab/gtsam](https://hub.docker.com/r/borglab/gtsam)**:
    A pre-compiled environment containing the latest `develop` branch of GTSAM. Useful for quick testing or as a base for downstream applications.
    -   *Source:* [`docker-images/gtsam`](https://github.com/borglab/docker-images/tree/main/gtsam)

-   **[borglab/gtsam-manylinux](https://hub.docker.com/r/borglab/gtsam-manylinux)**:
    An environment based on `manylinux2014` tailored for building Python wheels for GTSAM.
    -   *Source:* [`docker-images/gtsam-manylinux`](https://github.com/borglab/docker-images/tree/main/gtsam-manylinux)

-   **[borglab/ubuntu-boost-tbb](https://hub.docker.com/r/borglab/ubuntu-boost-tbb)**:
    Base image (Ubuntu 24.04) with Boost and TBB libraries pre-installed.
    -   *Source:* [`docker-images/ubuntu-boost-tbb`](https://github.com/borglab/docker-images/tree/main/ubuntu-boost-tbb)

-   **CI Images**:
    Various images used for Continuous Integration, covering different Ubuntu versions (22.04, 24.04) and compilers (Clang, GCC).
    -   *Source:* [`docker-images/gtsam-ci`](https://github.com/borglab/docker-images/tree/main/gtsam-ci)

## Usage

### Running GTSAM

To start an interactive shell in a container with GTSAM pre-installed:

```bash
docker run -it borglab/gtsam:latest
```

### Using the Python Wrapper

The `borglab/gtsam` image typically includes Python bindings. To use them:

1.  Start the container:
    ```bash
    docker run -it borglab/gtsam:latest
    ```
2.  Launch Python:
    ```bash
    python3
    ```
3.  Import GTSAM:
    ```python
    import gtsam
    print(gtsam.Pose3())
    ```

## Building Images

To build these images locally or contribute changes, please refer to the **[borglab/docker-images](https://github.com/borglab/docker-images)** repository. It contains the Dockerfiles and build scripts for all the images listed above.

### Legacy Configuration



The following files in this directory are legacy artifacts and are **no longer actively maintained**:



-   **`Containerfile`**: Build instructions for a standalone GTSAM image (cloning from git and building from source).

-   **`compose.yaml`**: A Docker Compose wrapper used for configurable builds (via `.env` variables like `GTSAM_WITH_TBB`, `GTSAM_BUILD_PYTHON`) and standardized image tagging.

-   **`hub_push.sh`**: A utility script to iterate through configuration matrices and push multiple image variants to Docker Hub.



For official builds and the most up-to-date configurations, please refer to the **[borglab/docker-images](https://github.com/borglab/docker-images)** repository.



> **TODO**: Consider migrating the configurable build and matrix-pushing functionality from these legacy files into the `docker-images` repository to support more flexible local builds.



## VNC Support (`gtsam-vnc`)



The **gtsam-vnc** image configuration is available locally in the [`gtsam-vnc`](./gtsam-vnc) subdirectory. This image extends the official `borglab/gtsam` image by adding a VNC server, allowing you to view GUI applications (like Matplotlib plots) running inside the container.



### Building and Running VNC Image



1.  **Navigate to the directory:**

    ```bash

    cd gtsam-vnc

    ```



2.  **Build the image:**

    You can build it using Docker Compose or directly with Docker.

    ```bash

    # Example using docker build

    docker build -t gtsam-vnc .

    ```



3.  **Run with Port Forwarding:**

    Map port 5900 to access the VNC server.

    ```bash

    docker run -p 5900:5900 gtsam-vnc

    ```



4.  **Connect:**

    Use a VNC client to connect to `localhost:5900`.
