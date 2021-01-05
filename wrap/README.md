
# WRAP

The wrap library wraps the GTSAM library into a Python library or MATLAB toolbox.
It was designed to be more general than just wrapping GTSAM. For notes on creating a wrap interface, see `gtsam.h` for what features can be wrapped into a toolbox, as well as the current state of the toolbox for GTSAM.

## Prerequisites: Pybind11 and pyparsing

1. This library uses `pybind11`, which is included as a subdirectory in GTSAM.
2. The `interface_parser.py` in this library uses `pyparsing` to parse the interface file `gtsam.h`. Please install it first in your current Python environment before attempting the build.

```
python3 -m pip install pyparsing
```

## Getting Started

Clone this repository to your local machine and perform the standard CMake install:

```sh
mkdir build && cd build
cmake ..
make install # use sudo if needed
```

Using `wrap` in your project is straightforward from here. In you `CMakeLists.txt` file, you just need to add the following:

```cmake
include(PybindWrap)

pybind_wrap(${PROJECT_NAME}_py # target
            ${PROJECT_SOURCE_DIR}/cpp/${PROJECT_NAME}.h # interface header file
            "${PROJECT_NAME}.cpp" # the generated cpp
            "${PROJECT_NAME}" # module_name
            "gtsam" # top namespace in the cpp file
            "${ignore}" # ignore classes
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl
            ${PROJECT_NAME} # libs
            "${PROJECT_NAME}" # dependencies
            ON # use boost
            )
```

For more information, please follow our [tutorial](https://github.com/borglab/gtsam-project-python).

## GTSAM Python wrapper

**WARNING: On macOS, you have to statically build GTSAM to use the wrapper.**

1. Set `GTSAM_BUILD_PYTHON=ON` while configuring the build with `cmake`.
1. What you can do in the `build` folder:
    1. Just run python then import GTSAM and play around:
        ```

        import gtsam
        gtsam.__dir__()
        ```

    1. Run the unittests:
        ```
        python -m unittest discover
        ```
    1. Edit the unittests in `python/gtsam/*.py` and simply rerun the test.
    They were symlinked to `<build_folder>/gtsam/*.py` to facilitate fast development.
        ```
        python -m unittest gtsam/tests/test_Pose3.py
        ```
        - NOTE: You might need to re-run `cmake ..` if files are deleted or added.
1. Do `make install` and `cd <gtsam_install_folder>/python`. Here, you can:
    1. Run the unittests:
        ```
        python setup.py test
        ```
    2. Install `gtsam` to your current Python environment.
        ```
        python setup.py install
        ```
        - NOTE: It's a good idea to create a virtual environment otherwise it will be installed in your system Python's site-packages.
