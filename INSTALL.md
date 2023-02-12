# Quickstart

In the root library folder execute:

```sh
$ mkdir build
$ cd build
$ cmake ..
$ make check # (optional, runs unit tests)
$ make install
```

## Important Installation Notes

1. GTSAM requires the following libraries to be installed on your system:
    - BOOST version 1.67 or greater (install through Linux repositories or MacPorts). Please see [Boost Notes](#boost-notes) for version recommendations based on your compiler.

    - Cmake version 3.0 or higher
    - Support for XCode 4.3 command line tools on Mac requires CMake 2.8.8 or higher

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
  $ make check (optional, runs unit tests)
  $ make install
  ```

  This will build the library and unit tests, run all of the unit tests,
  and then install the library itself.

# Windows Installation

This section details how to build a GTSAM `.sln` file using Visual Studio.

### Prerequisites

- Visual Studio with C++ CMake tools for Windows
- All the other pre-requisites listed above.

### Steps

1. Open Visual Studio.
2. Select `Open a local folder` and select the GTSAM source directory.
3. Go to `Project -> CMake Settings`.
  - (Optional) Set `Configuration name`.
  - (Optional) Set `Configuration type`.
  - Set the `Toolset` to `msvc_x64_x64`. If you know what toolset you require, then skip this step.
  - Update the `Build root` to `${projectDir}\build\${name}`.
  - You can optionally create a new configuration for a `Release` build.
  - Set the necessary CMake variables for your use case.
  - Click on `Show advanced settings`.
  - For `CMake generator`, select a version which matches `Visual Studio <Version> <Year> Win64`, e.g. `Visual Studio 16 2019 Win64`.
  - Save the settings (Ctrl + S).
4. Click on `Project -> Generate Cache`. This will generate the CMake build files (as seen in the Output window).
5. The last step will generate a `GTSAM.sln` file in the `build` directory. At this point, GTSAM can be used as a regular Visual Studio project.


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
3. Add `-march=native` to `GTSAM_CMAKE_CXX_FLAGS`. A performance gain of
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


