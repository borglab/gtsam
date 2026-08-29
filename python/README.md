# README

# Python Wrapper

This is the Python wrapper around the GTSAM C++ library. We use our custom [wrap library](https://github.com/borglab/wrap) to generate the bindings to the underlying C++ code.

For instructions on updating the version of the [wrap library](https://github.com/borglab/wrap) included in GTSAM to the latest version, please refer to the [wrap README](https://github.com/borglab/wrap/blob/master/README.md#git-subtree-and-contributing)

## Requirements

- Cmake >= 3.15
- If you want to build the GTSAM python library for a specific python version (eg 3.6),
  use the `-DGTSAM_PYTHON_VERSION=3.6` option when running `cmake` otherwise the default interpreter will be used.
- This wrapper needs [pyparsing(>=2.4.2)](https://github.com/pyparsing/pyparsing), [pybind11-stubgen>=2.5.1](https://github.com/sizmailov/pybind11-stubgen) and [numpy(>=1.11.0)](https://numpy.org/).

  > **Note:** On systems that enforce [PEP 668](https://peps.python.org/pep-0668/) (Homebrew Python on macOS, and the system Python on Ubuntu 23.04+, Fedora, Arch, and other modern distros), bare `pip install` is blocked. Create and activate a virtual environment first:
  >
  > ```bash
  > python3 -m venv .venv
  > source .venv/bin/activate   # on Windows: .venv\Scripts\activate
  > ```

  Then install the requirements:

  ```bash
  pip install -r <gtsam_folder>/python/dev_requirements.txt
  ```

  When configuring cmake, point `PYTHON_EXECUTABLE` at the venv interpreter so the build and install use the same environment:

  ```bash
  cmake .. -DGTSAM_BUILD_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
  ```

## Install

- Run cmake with the `GTSAM_BUILD_PYTHON` cmake flag enabled to configure building the wrapper. The wrapped module will be built and copied to the directory `<PROJECT_BINARY_DIR>/python`. For example, if your local Python version is 3.6.10, then you should run:
  ```bash
  cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.6.10
  ```
  If you do not have TBB installed, you should also provide the argument `-DGTSAM_WITH_TBB=OFF`.
- Build GTSAM and the wrapper with `make` (or `ninja` if you use `-GNinja`).

- To install, simply run `make python-install` (`ninja python-install`).
  - This installs into the Python interpreter selected by `cmake` when GTSAM was configured, which is the interpreter printed in the cmake configure summary. It does not change based on whichever virtual environment is active when `make python-install` runs.
  - To install into a virtual environment, activate the environment before running `cmake`, then either let cmake discover that interpreter or pass `-DGTSAM_PYTHON_VERSION=<version>` matching the environment. To verify the target environment, run `<full/path/to/python> -m pip list` with the Python path reported by cmake.
  - **NOTE**: if you don't want GTSAM to install to a system directory such as `/usr/local`, pass `-DCMAKE_INSTALL_PREFIX="./install"` to cmake to install GTSAM to a subdirectory of the build directory.

- You can also directly run `make python-install` without running `make`, and it will compile all the dependencies accordingly.

## CUDA Bindings

The optional CUDA optimizers are exposed under `gtsam.cuda` only when the
Python wrapper is built from source with CUDA enabled. Configure and build the
module with:

```bash
cmake -S . -B build-cuda -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_ENABLE_CUDA=ON
cmake --build build-cuda --target gtsam_py -j6
```

This is sufficient for the matrix-free PCG backend and CUDA SFM dense
Cholesky. Add `-DGTSAM_ENABLE_CUDSS=ON` to enable the cuDSS sparse direct
backend; cuDSS must be installed separately.

When CUDA is disabled, `gtsam.cuda` is intentionally absent. See the
[CUDA linear solver guide](../docs/CUDA_LINEAR_SOLVERS.md) for the General LM
and SFM Python APIs, backend selection, and examples.

## Windows Installation

See Windows Installation in INSTALL.md in the root directory.

## Generate Docstrings

The wrap library provides for building the Python wrapper with docstrings included, sourced from the C++ Doxygen comments. To build the Python wrapper with docstrings, follow these instructions:

1. Build GTSAM with the flag `-DGTSAM_GENERATE_DOC_XML=1`. This will compile the `doc/Doxyfile.in` into a `Doxyfile` with `GENERATE_XML` set to `ON`.
2. From the project root directory, run `doxygen build/<build_name>/doc/Doxyfile`. This will generate the Doxygen XML documentation in `xml/`.
3. Build the Python wrapper with the CMake option `GTWRAP_ADD_DOCSTRINGS` enabled.

## Unit Tests

The Python toolbox also has a small set of unit tests located in the
test directory.
To run them, use `make python-test`.

## Utils

TODO

## Examples

TODO

## Writing Your Own Scripts

See the tests for examples.

### Some Important Notes:

- Vector/Matrix:

  - GTSAM expects double-precision floating point vectors and matrices.
    Hence, you should pass numpy matrices with `dtype=float`, or `float64`, to avoid any conversion needed.
  - Also, GTSAM expects _column-major_ matrices, unlike the default storage
    scheme in numpy. But this is only performance-related as `pybind11` should translate them when needed. However, this will result a copy if your matrix is not in the expected type
    and storage order.

## Wrapping Custom GTSAM-based Project

Please refer to the template project and the corresponding tutorial available [here](https://github.com/borglab/GTSAM-project-python).

## Wheels

GTSAM Python wheels are built in CI through two cibuildwheel workflows that share the same matrix of Python 3.11--3.14 targets on Linux x86_64, Linux aarch64, macOS x86_64, and macOS arm64. Both scripts first configure the wrapper with `cmake -DGTSAM_BUILD_PYTHON=1` so that `setup.py` exists for cibuildwheel, invoke `.github/scripts/python_wheels/cibw_before_all.sh`, then run `.github/scripts/python_wheels/build_wheels.sh` before storing the artifacts and publishing them with `pypa/gh-action-pypi-publish`.

1. **Develop wheels** (`.github/workflows/build-cibw.yml`) run on every push to `develop` (and by manual dispatch). The workflow injects `DEVELOP=1` and a timestamp so the generated version string becomes a `gtsam-develop` build, and it continues to publish the built wheels via the publish action at the end of the job. Use this workflow as a staging pipeline for the most recent development snapshots.

2. **Release wheels** (`.github/workflows/prod-cibw.yml`) trigger when a GitHub release is published (and can also be run manually). The job is otherwise identical but omits the `DEVELOP` flag and publishes the wheels to PyPI, making it the production-quality artifact build tied to a release tag.

### Cleaning develop wheels

After a successful develop-wheel upload, `.github/workflows/build-cibw.yml` can automatically prune `gtsam-develop` on PyPI. Configure the repository secrets `PYPI_CLEANUP_USERNAME` and `PYPI_CLEANUP_PASSWORD` for a PyPI owner account on `gtsam-develop`; if either secret is missing, the workflow skips cleanup. If that account requires two-factor authentication, also configure the optional base32 TOTP seed as `PYPI_CLEANUP_TOTP_SECRET`. The cleanup keeps the five most recent PyPI releases by upload time and deletes older release versions.

For a manual dry run, install `pypi-cleanup` and run `bash .github/scripts/python_wheels/cleanup_gtsam_develop.sh --dry-run`. To delete manually, run `bash .github/scripts/python_wheels/cleanup_gtsam_develop.sh --keep 5 <username>` and provide the PyPI password when prompted, or set `PYPI_CLEANUP_PASSWORD` in the environment. If needed, set `PYPI_CLEANUP_TOTP_SECRET` to let the script generate the PyPI TOTP code. Treat deletion as permanent cleanup for staying below PyPI's project size limit.
