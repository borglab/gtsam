# GTSAM GitHub Configuration

This directory contains GitHub-specific configuration, including issue templates and CI workflows.

## Workflows Overview

The `.github/workflows` directory contains definitions for the various CI/CD processes:

-   **`build-linux.yml`**: Main Linux CI. Runs inside the pre-built Docker containers mentioned above to compile and test GTSAM with various compilers (GCC, Clang) and configurations.
-   **`build-macos.yml`**: Compiles and tests GTSAM on macOS runners.
-   **`build-windows.yml`**: Compiles and tests GTSAM on Windows runners using MSVC.
-   **`build-python.yml`**: Verifies the Python wrapper compilation across Linux, macOS, and Windows.
-   **`build-cibw.yml`**: Builds distributable Python wheels using `cibuildwheel`. It builds dependencies (like Boost) from source to ensure ABI compatibility.
-   **`prod-cibw.yml`**: Used for production wheel builds (often triggered on release).
-   **`vcpkg.yml`**: Compiles and tests GTSAM on Linux, macOS, and Windows using `vcpkg` to install all dependencies.
-   **`deploy.yml`**: Handles deployment tasks (e.g., docs, release artifacts).

## Updating vcpkg dependencies

`vcpkg` supports a manifest in the form of `vcpkg.json`, which specifies all the dependencies to install, and a "baseline", which is just a commit hash from https://github.com/microsoft/vcpkg. Instead of upgrading individual library versions which might be incompatible, you upgrade baselines, which are a set of library versions that have been tested to work together. This also means library versions are pinned to the baseline, so the baseline needs to periodically updated to get the latest libraries. Updating is not *strictly* necessary, as old library versions should work for a while, but it can be good to ensure compatibility with the latest libraies. To update the baseline, simply go to https://github.com/microsoft/vcpkg and copy the full commit hash for the version you want (could be HEAD for the day, or if you prefer tags, you can use the commit hash for a tag instead) and update the `builtin-baseline` field in `vcpkg.json` at the root of this repo.

## Docker CI Images (Linux Only)

The **Linux CI** workflow (`build-linux.yml`) relies on pre-built Docker images to ensure consistency and speed up build times. These images are hosted in the [borglab/docker-images](https://github.com/borglab/docker-images) repository.

*Note: macOS and Windows workflows use standard GitHub Actions runners and install dependencies (like Boost) via package managers (Homebrew, Chocolatey) or from source.*

### Building and Updating Images

If you need to update an existing CI image or add a new one (e.g., for a new Ubuntu version or compiler):

1.  **Navigate to the `docker-images` repository:**
    ```bash
    cd ../docker-images/gtsam-ci
    ```
2.  **Add or modify a Dockerfile:**
    -   Follow the naming convention: `ubuntu-<version>-<compiler>-<version>.Dockerfile`.
    -   Base images are defined in `*-base.Dockerfile`.
3.  **Build and Push:**
    Use the provided script to build and push to Docker Hub (requires `borglab` permissions):
    ```bash
    ./build_and_push.sh <dockerhub-username>
    ```

For more details, see the [README in the docker-images repository](https://github.com/borglab/docker-images/blob/main/gtsam-ci/README.md).
