# Installation Guide

## Quickstart

In the root library folder execute:

```sh
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build . --target check # (optional, runs unit tests)
$ cmake --build . --target install
```

## Important Installation Notes

1. GTSAM requires the following libraries to be installed on your system:
    - BOOST version 1.70 or greater (install through Linux repositories or MacPorts). Please see [Boost Notes](#boost-notes) for version recommendations based on your compiler.

    - CMake version 3.10 or higher

    Optional dependent libraries:
     - If TBB is installed and detectable by CMake GTSAM will use it automatically.
       Ensure that CMake prints "Use Intel TBB : Yes".  To disable the use of TBB,
       disable the CMake flag `GTSAM_WITH_TBB` (enabled by default) by providing
       the argument `-DGTSAM_WITH_TBB=OFF` to `cmake`.  On Ubuntu, TBB may be
       installed from the Ubuntu repositories, and for other platforms it may be
       downloaded from https://www.threadingbuildingblocks.org/
     - GTSAM may be configured to use MKL by toggling `GTSAM_WITH_EIGEN_MKL` and
       `GTSAM_WITH_EIGEN_MKL_OPENMP` to `ON`; however, best performance is usually
       achieved with MKL disabled. We therefore advise you to benchmark your problem
       before using MKL.

    Tested compilers:

    - GCC 4.2-7.3
    - OS X Clang 2.9-10.0
    - OS X GCC 4.2
    - MSVC 2010, 2012, 2017

    Tested systems:

    - Ubuntu 16.04 - 18.04
    - MacOS 10.6 - 10.14
    - Windows 7, 8, 8.1, 10

2. GTSAM makes extensive use of debug assertions, and we highly recommend you work
in Debug mode while developing (enabled by default). Likewise, it is imperative
that you switch to release mode when running finished code and for timing. GTSAM
will run up to 10x faster in Release mode! See the end of this document for
additional debugging tips.

3. GTSAM has Doxygen documentation. To generate, run 'make doc' from your
build directory after setting the `GTSAM_BUILD_DOCS` and `GTSAM_BUILD_[HTML|LATEX]` cmake flags.

4. The instructions below install the library to the default system install path and
build all components. From a terminal, starting in the root library folder,
execute commands as follows for an out-of-source build:

  ```sh
  $ mkdir build
  $ cd build
  $ cmake ..
  $ cmake --build . --target check # (optional, runs unit tests)
  $ cmake --build . --target install
  ```

  This will build the library and unit tests, run all of the unit tests,
  and then install the library itself.

## Boost Notes

Versions of Boost prior to 1.65 have a known bug that prevents proper "deep" serialization of objects, which means that objects encapsulated inside other objects don't get serialized.
This is particularly seen when using `clang` as the C++ compiler.

For this reason we recommend Boost>=1.65, and recommend installing it through alternative channels when it is not available through your operating system's primary package manager.

## Known Issues

- MSVC 2013 is not yet supported because it cannot build the serialization module of Boost 1.55 (or earlier).

# Windows Installation

There are two ways to build GTSAM on Windows: the traditional way with Visual Studio and the modern way with CMake + Ninja. The CMake + Ninja way is preferred because the Ninja generator is much faster than Visual Studio.


**Important**: Regardless of how you build, GTSAM requires compiling with `/permissive-` and for all projects to also compile with `/permissive-` (due to lots of code being in headers) and sets the list of public compiler flags accordingly. If your project does not currently build with `/permissive-`, make sure it does and fix whatever is needed to make it work. Failure to compile with `/permissive-` can cause various runtime or build errors.

### Prerequisites

- Visual Studio with Desktop development with C++
  - You need MSVC and the Windows SDK to build GTSAM.
  - This also includes the C++ CMake tools for Windows component, which includes Ninja and CMake.
  - CMake >= 3.21 is required for Visual Studio installation because custom templates used in the build were added in 3.21. Use `cmake --version` in the VS Developer Command Prompt to ensure you meet this requirement.
- All the other pre-requisites listed above.

## Building with CMake and Ninja

This section details how to use CMake with the Ninja generator. You must be in a Developer shell for this to work.

In the root library folder execute:

```powershell
$ mkdir build
$ cd build
$ cmake .. -G Ninja
$ cmake --build . --target check # (optional, runs unit tests)
$ cmake --build . --target install
```

Note: if you are used to using the Visual Studio generators, you do not need to pass --config here for Ninja. This is because the Visual Studio generator is a multi-config generator, so you need --config to select the build type. Ninja is not a multi-config generator, so you just need to set CMAKE_BUILD_TYPE when configuring and it will use that build type to compile. If you want the multi-config behavior, try using `-G Ninja Multi-Config`.

## Building with Visual Studio

This section details how to build a GTSAM `.sln` file using Visual Studio.

### Steps

1. Open Visual Studio.
2. Select `Open a local folder` and select the GTSAM source directory.
3. Go to `Project -> CMake Settings`.
  - (Optional) Set `Configuration name`.
  - (Optional) Set `Configuration type`.
  - Set the `Toolset` to `msvc_x64_x64`. If you know what toolset you require, then skip this step.
  - Update the `Build root` to `${projectDir}\build\${name}`.
  - You can optionally create a new configuration for a `Release` build.
  - Set the necessary CMake variables for your use case. If you are not using Boost, uncheck `GTSAM_ENABLE_BOOST_SERIALIZATION` and `GTSAM_USE_BOOST_FEATURES`. 
  - Click on `Show advanced settings`.
  - For `CMake generator`, select a version which matches `Visual Studio <Version> <Year> Win64`, e.g. `Visual Studio 17 2022 Win64`.
  - Save the settings (Ctrl + S).
4. Saving the CMake settings should automatically generate the cache. Otherwise, click on `Project -> Configure Cache`. This will generate the CMake build files (as seen in the Output window).
  - If generating the cache yields `CMake Error ... (ADD_CUSTOM_COMMAND)` errors, you have an old CMake. Verify that your CMake is >= 3.21. If Visual Studio says that it is but you're still getting the error, install the latest CMake (tested with 3.31.4) and point to its executable in `CMakeSettings > Advanced settings > CMake executable`.
5. The last step will generate a `GTSAM.sln` file in the `build` directory. At this point, GTSAM can be used as a regular Visual Studio project.

### Python Installation

To install the Python bindings on Windows:

Install [pyparsing(>=2.4.2)](https://github.com/pyparsing/pyparsing), [pybind-stubgen>=2.5.1](https://github.com/sizmailov/pybind11-stubgen) and [numpy(>=1.11.0)](https://numpy.org/) with the Python environment you wish to develop in. These can all be installed as follows:

  ```bash
  pip install -r <gtsam_folder>/python/dev_requirements.txt
  ```

1. Follow the above steps for GTSAM general installation.
  - In the CMake settings variables, set `GTSAM_BUILD_PYTHON` to be true and specify the path to your desired Python environment executable in the "CMake command arguments" field using `-DPYTHON_EXECUTABLE="<path to your python.exe>"`
  - Once the cache is generated, ensure it's using your desired Python executable and the version is correct. It may cause errors later otherwise, such as missing Python packages required to build. If the version is not the one you specified in `DPYTHON_EXECUTABLE`, change the version in all 3 Python version specifiers in the CMake variables (`GTSAM_PYTHON_VERSION`, `PYBIND11_PYTHON_VERSION`, `WRAP_PYTHON_VERSION`) to the exact version of your desired executable. CMake might look for executables in the standard locations first, so ensure it's using the one you want before continuing.
2. Build the project (Build > Build All).
  - If you encounter the error `C1083	Cannot open include file: 'boost/serialization/export.hpp': No such file or directory`, you need to make changes to the template files that include Boost since your build is not using Boost. Locate `python/gtsam/gtsam.tpl` (and `python/gtsam_unstable/gtsam_unstable.tpl` if you are building unstable) and comment out the line `#include <boost/serialization/export.hpp>` near the top of each. Delete the generated build directory (e.g. if you made `build/x64-Debug`, delete the `x64-Debug` folder), regenerate the cache, and build again.
  - If you encounter an error involving copying `.pyd` files, find the files mentioned (`gtsam_py.pyd` and `gtsam_unstable_py.pyd`, probably in the `Debug`/`Release`/etc. folder inside `build/<your build>/python/gtsam`) and copy them to where they are supposed to be (the source of the copy error, probably `build/<your build>/python/gtsam`) then rebuild.
3. At this point, `gtsam` in `build/<your build>/python` is available to be used as a Python package. You can use `pip install .` in that directory to install the package.



# CMake Configuration Options and Details

GTSAM has a number of options that can be configured, which is best done with
one of the following:

  - ccmake      the curses GUI for cmake
  - cmake-gui   a real GUI for cmake

## Important Options:

#### CMAKE_BUILD_TYPE
We support several build configurations for GTSAM (case insensitive)

```cmake -DCMAKE_BUILD_TYPE=[Option] ..```

  - Debug (default)  All error checking options on, no optimization. Use for development.
  - Release          Optimizations turned on, no debug symbols.
  - Timing           Adds ENABLE_TIMING flag to provide statistics on operation
  - Profiling        Standard configuration for use during profiling
  - RelWithDebInfo   Same as Release, but with the -g flag for debug symbols

#### CMAKE_INSTALL_PREFIX

The install folder. The default is typically `/usr/local/`.
To configure to install to your home directory, you could execute:

```cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME ..```

#### GTSAM_TOOLBOX_INSTALL_PATH

The Matlab toolbox will be installed in a subdirectory
of this folder, called 'gtsam'.

```cmake -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=$HOME/toolbox ..```

#### GTSAM_BUILD_CONVENIENCE_LIBRARIES

This is a build option to allow for tests in subfolders to be linked against convenience libraries rather than the full libgtsam.
Set with the command line as follows:

```cmake -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON ..```
  - ON (Default): This builds convenience libraries and links tests against them. This   				 option is suggested for gtsam developers, as it is possible to build and run tests without first building the rest of the library, and speeds up compilation for a single test. The downside of this option is that it will build the entire library again to build the full libgtsam library, so build/install will be slower.
  - OFF: This will build all of libgtsam before any of the tests, and then link all of the tests at once. This option is best for users of GTSAM, as it avoids rebuilding the entirety of gtsam an extra time.

#### GTSAM_BUILD_UNSTABLE

Enable build and install for libgtsam_unstable library.
Set with the command line as follows:

```cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON ..```

  ON:             When enabled, libgtsam_unstable will be built and installed with the same options as libgtsam.  In addition, if tests are enabled, the unit tests will be built as well.  The Matlab toolbox will also be generated if the matlab toolbox is enabled, installing into a folder called `gtsam_unstable`.
  OFF (Default)  If disabled, no `gtsam_unstable` code will be included in build or install.

## Convenience Options:

#### GTSAM_BUILD_EXAMPLES_ALWAYS

Whether or not to force building examples, can be true or false.

#### GTSAM_BUILD_TESTS

Whether or not to build tests, can be true or false.

## Check

`make check` will build and run all of the tests. Note that the tests will only be
built when using the "check" targets, to prevent `make install` from building the tests
unnecessarily. You can also run `make timing` to build all of the timing scripts.
To run check on a particular module only, run `make check.[subfolder]`, so to run
just the geometry tests, run `make check.geometry`. Individual tests can be run by
appending `.run` to the name of the test, for example, to run testMatrix, run
`make testMatrix.run`.

MEX_COMMAND: Path to the mex compiler. Defaults to assume the path is included in your shell's PATH environment variable. mex is installed with matlab at `$MATLABROOT/bin/mex`

$MATLABROOT can be found by executing the command `matlabroot` in MATLAB

## Performance

Here are some tips to get the best possible performance out of GTSAM.

1. Build in `Release` mode. GTSAM will run up to 10x faster compared to `Debug` mode.
2. Enable TBB. On modern processors with multiple cores, this can easily speed up
    optimization by 30-50%. Please note that this may not be true for very small
    problems where the overhead of dispatching work to multiple threads outweighs
    the benefit. We recommend that you benchmark your problem with/without TBB.
3. Use `GTSAM_BUILD_WITH_MARCH_NATIVE`. A performance gain of
    25-30% can be expected on modern processors. Note that this affects the portability
    of your executable. It may not run when copied to another system with older/different
    processor architecture.
    Also note that all dependent projects *must* be compiled with the same flag, or
    seg-faults and other undefined behavior may result.
4. Possibly enable MKL. Please note that our benchmarks have shown that this helps only
    in very limited cases, and actually hurts performance in the usual case. We therefore
    recommend that you do *not* enable MKL, unless you have benchmarked it on
    your problem and have verified that it improves performance.


## Debugging tips

Another useful debugging symbol is _GLIBCXX_DEBUG, which enables debug checks and safe containers in the standard C++ library and makes problems much easier to find.

NOTE:  The native Snow Leopard g++ compiler/library contains a bug that makes it impossible to use _GLIBCXX_DEBUG.  MacPorts g++ compilers do work with it though.

NOTE:  If _GLIBCXX_DEBUG is used to compile gtsam, anything that links against gtsam will need to be compiled with _GLIBCXX_DEBUG as well, due to the use of header-only Eigen.


## Installing MKL on Linux

Intel has a guide for installing MKL on Linux through APT repositories at <https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo>.

After following the instructions, add the following to your `~/.bashrc` (and afterwards, open a new terminal before compiling GTSAM):
`LD_PRELOAD` need only be set if you are building the python wrapper to use GTSAM from python.
```sh
source /opt/intel/mkl/bin/mklvars.sh intel64
export LD_PRELOAD="$LD_PRELOAD:/opt/intel/mkl/lib/intel64/libmkl_core.so:/opt/intel/mkl/lib/intel64/libmkl_sequential.so"
```
To use MKL in GTSAM pass the flag `-DGTSAM_WITH_EIGEN_MKL=ON` to cmake.


The `LD_PRELOAD` fix seems to be related to a well known problem with MKL which leads to lots of undefined symbol errors, for example:
- <https://software.intel.com/en-us/forums/intel-math-kernel-library/topic/300857>
- <https://software.intel.com/en-us/forums/intel-distribution-for-python/topic/628976>
- <https://groups.google.com/a/continuum.io/forum/#!topic/anaconda/J3YGoef64z8>

Failing to specify `LD_PRELOAD` may lead to errors such as:
`ImportError: /opt/intel/mkl/lib/intel64/libmkl_vml_avx2.so: undefined symbol: mkl_serv_getenv`
or
`Intel MKL FATAL ERROR: Cannot load libmkl_avx2.so or libmkl_def.so.`
when importing GTSAM using the python wrapper.


## Compile gtsam with vcpkg. (For linux/wsl)
Install python dependencies + ninja + build essential in linux:   
```
sudo apt update
sudo apt-get install autoconf automake autoconf-archive ninja-build build-essential -y
```

On your home dir near gtsam dir:   
Setup vcpkg   
```bash
git clone https://github.com/microsoft/vcpkg
cd vcpkg
./bootstrap-vcpkg.sh
```

vcpkg install dependencies   
on vcpkg dir:   
```bash
./vcpkg x-set-installed         \
          boost-assign          \
          boost-bimap           \
          boost-chrono          \
          boost-date-time       \
          boost-filesystem      \
          boost-format          \
          boost-graph           \
          boost-math            \
          boost-program-options \
          boost-regex           \
          boost-serialization   \
          boost-system          \
          boost-thread          \
          boost-timer           \
          tbb                   \
          pybind11
```

Setup python dependencies   
Go to your gtsam folder `cd gtsam`:   
```bash
../vcpkg/installed/x64-linux/tools/python3/python3 -m ensurepip --upgrade
../vcpkg/installed/x64-linux/tools/python3/python3 -m pip install -r python/dev_requirements.txt
```

Cmake config:   
In gtsam folder   
```bash
cmake . -B build -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=../scripts/buildsystems/vcpkg.cmake \
    -DVCPKG_INSTALLED_DIR=../installed \
    -DVCPKG_TARGET_TRIPLET=x64-linux \
    -DVCPKG_HOST_TRIPLET=x64-linux \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_BUILD_TESTS=ON \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=OFF \
    -DGTSAM_USE_SYSTEM_METIS=OFF \
    -DGTSAM_USE_SYSTEM_PYBIND=ON \
    -DGTSAM_SUPPORT_NESTED_DISSECTION=ON
```

cmake compile:   
In gtsam folder:   
```bash
cmake --build build
```

Run python tests:   
```
VCPKG_INSTALLATION_ROOT=<abs path to vcpkg root dir>
export PATH="$PATH:$VCPKG_INSTALLATION_ROOT/installed/x64-linux/bin"
cmake --build build --target python-install
cmake --build build --target python-test
cmake --build build --target python-test-unstable
```

Run gtsam tests:   
```bash
cmake --build build --target check
```