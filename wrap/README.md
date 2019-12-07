# WRAP

The wrap library wraps the GTSAM library into a Python library or MATLAB toolbox.
It was designed to be more general than just wrapping GTSAM. For notes on creating a wrap interface, see `gtsam.h` for what features can be wrapped into a toolbox, as well as the current state of the toolbox for gtsam.

## Presequisites: Pybind11 and pyparsing

1. This library uses `pybind11`, which is included as a submodule in GTSAM. If you didn't clone gtsam with the `--recursive` option, you will need to initialize and update the submodule first as follows:
    ```
    git submodule init
    git submodule update
    ```

2. The `interface_parser.py` in this library uses `pyparsing` to parse the interface file `gtsam.h`. Please install it first in your current Python environment before attempting the build.
    ```
    python3 -m pip install pyparsing
    ```

## GTSAM Python wrapper

1. Set `GTSAM_BUILD_PYTHON=ON` while configuring the build with `cmake`.
1. What you can do in the `build` folder:
    1. Just run python then import gtsam and play around:
        ```
        from gtsam_py import gtsam
        gtsam.__dir__()
        ```

    1. Run the unittests:
        ```
        python -m unittest discover
        ```
    1. Edit the unittests in `wrap/python/gtsam_py/*.py` and simply rerun the test.
    They were symlinked to `<build_folder>/gtsam_py/*.py` to facilitate fast development.
        ```
        python -m unittest gtsam_py/tests/test_Pose3.py
        ```
        - NOTE: You might need to re-run `cmake ..` if files are deleted or added.
1. Do `make install` and `cd <gtsam_install_folder>/python`. Here, you can:
    1. Run the unittests:
        ```
        python setup.py test
        ```
    1. Install `gtsam_py` to your current Python environment.
        ```
        python setup.py install
        ```
        - NOTE: It's a good idea to create a virtual environment otherwise it will be installed in your system Python's site-packages.


## GTSAM Matlab wrapper
*Outdated note from the original wrap.*

TODO: Update this.

It was designed to be more general than just wrapping GTSAM, but a small amount of GTSAM specific code exists in matlab.h, the include file that is included by the mex files. The GTSAM-specific functionality consists primarily of handling of Eigen Matrix and Vector classes.

Some good things to know:

OBJECT CREATION

- Classes are created by special constructors, e.g., new_GaussianFactorGraph_.cpp.
	These constructors are called from the MATLAB class @GaussianFactorGraph.
	new_GaussianFactorGraph_ calls wrap_constructed in matlab.h, see documentation there

METHOD (AND CONSTRUCTOR) ARGUMENTS

- Simple argument types of methods, such as "double", will be converted in the
  mex wrappers by calling unwrap<double>, defined in matlab.h
- Vector and Matrix arguments are normally passed by reference in GTSAM, but
  in gtsam.h you need to pretend they are passed by value, to trigger the
  generation of the correct conversion routines unwrap<Vector> and unwrap<Matrix>
- passing classes as arguments works, provided they are passed by reference.
	This triggers a call to unwrap_shared_ptr
