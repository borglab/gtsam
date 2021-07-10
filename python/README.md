# README

# Python Wrapper

This is the Python wrapper around the GTSAM C++ library. We use our custom [wrap library](https://github.com/borglab/wrap) to generate the bindings to the underlying C++ code.

For instructions on updating the version of the [wrap library](https://github.com/borglab/wrap) included in GTSAM to the latest version, please refer to the [wrap README](https://github.com/borglab/wrap/blob/master/README.md#git-subtree-and-contributing)

## Requirements

- If you want to build the GTSAM python library for a specific python version (eg 3.6),
  use the `-DGTSAM_PYTHON_VERSION=3.6` option when running `cmake` otherwise the default interpreter will be used.
- If the interpreter is inside an environment (such as an anaconda environment or virtualenv environment),
  then the environment should be active while building GTSAM.
- This wrapper needs `pyparsing(>=2.4.2)`, and `numpy(>=1.11.0)`. These can be installed as follows:

  ```bash
  pip install -r <gtsam_folder>/python/requirements.txt
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
