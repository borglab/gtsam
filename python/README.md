# README

# Python Wrapper

This is the Python wrapper around the GTSAM C++ library. We use Cython to generate the bindings to the underlying C++ code.

## Requirements

- If you want to build the GTSAM python library for a specific python version (eg 3.6),
  use the `-DGTSAM_PYTHON_VERSION=3.6` option when running `cmake` otherwise the default interpreter will be used.
- If the interpreter is inside an environment (such as an anaconda environment or virtualenv environment),
  then the environment should be active while building GTSAM.
- This wrapper needs `Cython(>=0.25.2)`, `backports_abc(>=0.5)`, and `numpy(>=1.11.0)`. These can be installed as follows:

  ```bash
  pip install -r <gtsam_folder>/cython/requirements.txt
  ```

- For compatibility with GTSAM's Eigen version, it contains its own cloned version of [Eigency](https://github.com/wouterboomsma/eigency.git),
  named `gtsam_eigency`, to interface between C++'s Eigen and Python's numpy.

## Install

- Run cmake with the `GTSAM_INSTALL_CYTHON_TOOLBOX` cmake flag enabled to configure building the wrapper. The wrapped module will be built and copied to the directory defined by `GTSAM_CYTHON_INSTALL_PATH`, which is by default `<PROJECT_BINARY_DIR>/cython` in Release mode and `<PROJECT_BINARY_DIR>/cython<CMAKE_BUILD_TYPE>` for other modes.

- Build GTSAM and the wrapper with `make`.

- To install, simply run `make python-install`.
  - The same command can be used to install into a virtual environment if it is active.
  - **NOTE**: if you don't want GTSAM to install to a system directory such as `/usr/local`, pass `-DCMAKE_INSTALL_PREFIX="./install"` to cmake to install GTSAM to a subdirectory of the build directory.

- You can also directly run `make python-install` without running `make`, and it will compile all the dependencies accordingly.

## Unit Tests

The Cython toolbox also has a small set of unit tests located in the
test directory. To run them:

  ```bash
  cd <GTSAM_CYTHON_INSTALL_PATH>
  python -m unittest discover
  ```

## Utils

TODO

## Examples

TODO

## Writing Your Own Scripts

See the tests for examples.

### Some Important Notes:

- Vector/Matrix:

  - GTSAM expects double-precision floating point vectors and matrices.
    Hence, you should pass numpy matrices with `dtype=float`, or `float64`.
  - Also, GTSAM expects _column-major_ matrices, unlike the default storage
    scheme in numpy. Hence, you should pass column-major matrices to GTSAM using
    the flag order='F'. And you always get column-major matrices back.
    For more details, see [this link](https://github.com/wouterboomsma/eigency#storage-layout---why-arrays-are-sometimes-transposed).
  - Passing row-major matrices of different dtype, e.g. `int`, will also work
    as the wrapper converts them to column-major and dtype float for you,
    using numpy.array.astype(float, order='F', copy=False).
    However, this will result a copy if your matrix is not in the expected type
    and storage order.

- Inner namespace: Classes in inner namespace will be prefixed by <innerNamespace>\_ in Python.

  Examples: `noiseModel_Gaussian`, `noiseModel_mEstimator_Tukey`

- Casting from a base class to a derive class must be done explicitly.

  Examples:

  ```python
  noiseBase = factor.noiseModel()
  noiseGaussian = dynamic_cast_noiseModel_Gaussian_noiseModel_Base(noiseBase)
  ```

## Wrapping Custom GTSAM-based Project

Please refer to the template project and the corresponding tutorial available [here](https://github.com/borglab/GTSAM-project-python).
