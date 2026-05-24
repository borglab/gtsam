# README

# Python Wrapper

This is the Python wrapper around the GTSAM C++ library. We use our custom [wrap library](https://github.com/borglab/wrap) to generate the bindings to the underlying C++ code.

For instructions on updating the version of the [wrap library](https://github.com/borglab/wrap) included in GTSAM to the latest version, please refer to the [wrap README](https://github.com/borglab/wrap/blob/master/README.md#git-subtree-and-contributing)

## Requirements

- Cmake >= 3.15
- If you want to build the GTSAM python library for a specific python version (eg 3.6),
  use the `-DGTSAM_PYTHON_VERSION=3.6` option when running `cmake` otherwise the default interpreter will be used.
- If the interpreter is inside an environment (such as an anaconda environment or virtualenv environment),
  then the environment should be active while building GTSAM.
- This wrapper needs [pyparsing(>=2.4.2)](https://github.com/pyparsing/pyparsing), [pybind11-stubgen>=2.5.1](https://github.com/sizmailov/pybind11-stubgen) and [numpy(>=1.11.0)](https://numpy.org/). These can all be installed as follows:

  ```bash
  pip install -r <gtsam_folder>/python/dev_requirements.txt
  ```

## Install

- Run cmake with the `GTSAM_BUILD_PYTHON` cmake flag enabled to configure building the wrapper. The wrapped module will be built and copied to the directory `<PROJECT_BINARY_DIR>/python`. For example, if your local Python version is 3.6.10, then you should run:
  ```bash
  cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.6.10
  ```
  If you do not have TBB installed, you should also provide the argument `-DGTSAM_WITH_TBB=OFF`.
- Build GTSAM and the wrapper with `make` (or `ninja` if you use `-GNinja`).

- To install, simply run `make python-install` (`ninja python-install`).
  - The same command can be used to install into a virtual environment if it is active.
  - **NOTE**: if you don't want GTSAM to install to a system directory such as `/usr/local`, pass `-DCMAKE_INSTALL_PREFIX="./install"` to cmake to install GTSAM to a subdirectory of the build directory.

- You can also directly run `make python-install` without running `make`, and it will compile all the dependencies accordingly.

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

GTSAM Python wheels are built in CI through two cibuildwheel workflows that share the same matrix of Python 3.10--3.13 targets on Linux x86_64, Linux aarch64, macOS x86_64, and macOS arm64. Both scripts first configure the wrapper with `cmake -DGTSAM_BUILD_PYTHON=1` so that `setup.py` exists for cibuildwheel, invoke `.github/scripts/python_wheels/cibw_before_all.sh`, then run `.github/scripts/python_wheels/build_wheels.sh` before storing the artifacts and publishing them with `pypa/gh-action-pypi-publish`.

1. **Develop wheels** (`.github/workflows/build-cibw.yml`) run on every push to `develop` (and by manual dispatch). The workflow injects `DEVELOP=1` and a timestamp so the generated version string becomes a `gtsam-develop` build, and it continues to publish the built wheels via the publish action at the end of the job. Use this workflow as a staging pipeline for the most recent development snapshots.

2. **Release wheels** (`.github/workflows/prod-cibw.yml`) trigger when a GitHub release is published (and can also be run manually). The job is otherwise identical but omits the `DEVELOP` flag and publishes the wheels to `https://test.pypi.org/legacy/`, making it the production-quality artifact build tied to a release tag.

### Cleaning develop wheels

If the `gtsam-develop` project on PyPI grows too large (PyPI enforces a 10 GB quota for each package), run `.github/scripts/python_wheels/cleanup_gtsam_develop.sh` to drop every release except the most recent one. You can pass your PyPI username (`bash .github/scripts/python_wheels/cleanup_gtsam_develop.sh <username>`) or let the script prompt for it, but the account must be an owner or maintainer of `gtsam-develop`. The script always confirms before deleting, then calls `python3 -m pypi_cleanup` with `--leave-most-recent-only --do-it`, so treat this as a permanent cleanup that should only be used when you are about to exceed PyPI's size limit.
