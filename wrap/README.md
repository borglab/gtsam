# WRAP

The wrap library wraps the GTSAM library into a Python library or MATLAB toolbox.
It was designed to be more general than just wrapping GTSAM. For notes on creating a wrap interface, see `gtsam.h` for what features can be wrapped into a toolbox, as well as the current state of the toolbox for GTSAM.

## Prerequisites

`Pybind11` and `pyparsing`

1. This library uses `pybind11`, which is included as a subdirectory in GTSAM.
2. The `interface_parser.py` in this library uses `pyparsing` to parse the interface file `gtsam.h`. Please install it first in your current Python environment before attempting the build.

```sh
python3 -m pip install pyparsing
```

## Getting Started

Clone this repository to your local machine and perform the standard CMake install:

```sh
mkdir build && cd build
cmake ..
make install # use sudo if needed
```

Using `wrap` in your project is straightforward from here. In your `CMakeLists.txt` file, you just need to add the following:

```cmake
find_package(gtwrap)

set(interface_files ${PROJECT_SOURCE_DIR}/cpp/${PROJECT_NAME}.h)

pybind_wrap(${PROJECT_NAME}_py # target
            "${interface_files}" # list of interface header files
            "${PROJECT_NAME}.cpp" # the generated cpp
            "${PROJECT_NAME}" # module_name
            "${PROJECT_MODULE_NAME}" # top namespace in the cpp file e.g. gtsam
            "${ignore}" # ignore classes
            ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.tpl # the wrapping template file
            ${PROJECT_NAME} # libs
            "${PROJECT_NAME}" # dependencies
            ON # use boost serialization
            )
```

For more information, please follow our [tutorial](https://github.com/borglab/gtsam-project-python).

## Documentation

Documentation for wrapping C++ code can be found [here](https://github.com/borglab/wrap/blob/master/DOCS.md).

## Python Wrapper

**WARNING: On macOS, you have to statically build GTSAM to use the wrapper.**

1. Set `GTSAM_BUILD_PYTHON=ON` while configuring the build with `cmake`.
1. What you can do in the `build` folder:

   1. Just run python then import GTSAM and play around:

      ```python
      import gtsam
      gtsam.__dir__()
      ```

   1. Run the unittests:
      ```sh
      python -m unittest discover
      ```
   1. Edit the unittests in `python/gtsam/*.py` and simply rerun the test.
      They were symlinked to `<build_folder>/gtsam/*.py` to facilitate fast development.
      `python -m unittest gtsam/tests/test_Pose3.py` - NOTE: You might need to re-run `cmake ..` if files are deleted or added.

1. Do `make install` and `cd <gtsam_install_folder>/python`. Here, you can:
   1. Run the unittests:
      ```sh
      python setup.py test
      ```
   2. Install `gtsam` to your current Python environment.
      ```sh
      python setup.py install
      ```
      - NOTE: It's a good idea to create a virtual environment otherwise it will be installed in your system Python's site-packages.

## Matlab Wrapper

In the CMake, simply include the `MatlabWrap.cmake` file.

```cmake
include(MatlabWrap)
```

This cmake file defines functions for generating MATLAB wrappers.

- `wrap_and_install_library(interfaceHeader linkLibraries extraIncludeDirs extraMexFlags)` Generates wrap code and compiles the wrapper.

Usage example:

    `wrap_and_install_library("lba.h" "" "" "")`

Arguments:

- `interfaceHeader`: The relative or absolute path to the wrapper interface definition file.
- `linkLibraries`: Any _additional_ libraries to link. Your project library
  (e.g. `lba`), libraries it depends on, and any necessary
  MATLAB libraries will be linked automatically. So normally,
  leave this empty.
- `extraIncludeDirs`: Any _additional_ include paths required by dependent
  libraries that have not already been added by
  include_directories. Again, normally, leave this empty.
- `extraMexFlags`: Any _additional_ flags to pass to the compiler when building
  the wrap code. Normally, leave this empty.

## Git subtree and Contributing

**\*WARNING\*: Running the ./update_wrap.sh script from the GTSAM repo creates 2 new commits in GTSAM.  Be sure to _NOT_ push these directly to master/develop.  Preferably, open up a new PR with these updates (see below).**

The [wrap library](https://github.com/borglab/wrap) is included in GTSAM as a git subtree.  This means that sometimes the wrap library can have new features or changes that are not yet reflected in GTSAM.  There are two options to get the most up-to-date versions of wrap:
  1. Clone and install the [wrap repository](https://github.com/borglab/wrap).  For external projects, make sure cmake is using the external `wrap` rather than the one pre-packaged with GTSAM.
  2. Run `./update_wrap.sh` from the root of GTSAM's repository to pull in the newest version of wrap to your local GTSAM installation.  See the warning above about this script automatically creating commits.

To make a PR on GTSAM with the most recent wrap updates, create a new branch/fork then pull in the most recent wrap changes using `./update_wrap.sh`.  You should find that two new commits have been made: a squash and a merge from master.  You can push these (to the non-develop branch) and open a PR.

For any code contributions to the wrap project, please make them on the [wrap repository](https://github.com/borglab/wrap).
