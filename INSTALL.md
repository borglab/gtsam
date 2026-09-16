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
    - CMake version 3.16 or higher
    - A compiler with C++17 support. The continuously tested toolchains are:
      - Linux: GCC 11, 13, 14, or 15 and Clang 11, 14, or 16
      - macOS: Xcode 16
      - Windows: MSVC toolset 14.40

    Older C++17-capable toolchains may work but are not continuously tested.

    Boost version 1.70 or greater is required when either
    `GTSAM_USE_BOOST_FEATURES` or `GTSAM_ENABLE_BOOST_SERIALIZATION` is
    enabled. Both options are enabled by default outside ROS 2 `colcon` builds.
    To build without Boost, disable both:

    ```sh
    $ cmake .. \
        -DGTSAM_USE_BOOST_FEATURES=OFF \
        -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF
    ```

    ROS 2 `colcon` builds also use the system Eigen package by default. This
    keeps GTSAM and downstream ROS packages on the same Eigen version. Plain
    CMake builds continue to use GTSAM's bundled Eigen by default. Either build
    mode can override the choice explicitly with `GTSAM_USE_SYSTEM_EIGEN`.

    Optional dependent libraries:
     - If TBB is installed and detectable by CMake GTSAM will use it automatically.
       Ensure that CMake prints "Use Intel TBB : Yes".  To disable the use of TBB,
       disable the CMake flag `GTSAM_WITH_TBB` (enabled by default) by providing
       the argument `-DGTSAM_WITH_TBB=OFF` to `cmake`.  On Ubuntu, TBB may be
       installed from the Ubuntu repositories, and for other platforms it may be
       downloaded from the [oneTBB project](https://github.com/uxlfoundation/oneTBB).
     - GTSAM may be configured to use MKL by toggling `GTSAM_WITH_EIGEN_MKL` and
       `GTSAM_WITH_EIGEN_MKL_OPENMP` to `ON`; however, best performance is usually
       achieved with MKL disabled. We therefore advise you to benchmark your problem
       before using MKL.
     - The CUDA optimizers (`GTSAM_ENABLE_CUDA=OFF` by default) need the CUDA
       toolkit and a GPU of compute capability 6.0 or newer. Their cuDSS backend
       (`GTSAM_ENABLE_CUDSS=OFF` by default) additionally needs cuDSS 0.8.0 or
       newer installed separately, since it is neither part of the CUDA toolkit
       nor bundled with GTSAM. See
       [doc/CUDA_LINEAR_SOLVERS.md](doc/CUDA_LINEAR_SOLVERS.md).

2. GTSAM makes extensive use of debug assertions, and we highly recommend you
explicitly select Debug mode while developing. Single-configuration builds
default to Release mode. Use Release mode when running finished code and for
timing; GTSAM can run substantially faster than in Debug mode. See the end of this
document for additional debugging tips.

3. GTSAM has Doxygen documentation. To generate, run 'make doc' from your
build directory after setting the `GTSAM_BUILD_DOCS` and
`GTSAM_BUILD_DOC_[HTML|LATEX]` cmake flags.

4. The instructions below install the library to the default system install path and
build the default components. From a terminal, starting in the root library
folder, execute commands as follows for an out-of-source build:

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

GTSAM's minimum supported Boost version, 1.70, already includes the fix. We
recommend installing it through alternative channels when it is not available
through your operating system's primary package manager.

## Installing prebuilt packages

### Ubuntu PPA

GTSAM can also be installed on Ubuntu using the
[BorgLab PPA repositories](https://launchpad.net/~borglab).

For the current GTSAM 4.2 release:

```sh
sudo add-apt-repository ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

For nightly builds from the `develop` branch:

```sh
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

Package availability depends on the Ubuntu release. Consult the linked PPA
pages for the currently published packages.

### Arch Linux AUR

GTSAM is available in the Arch User Repository as
[`gtsam`](https://aur.archlinux.org/packages/gtsam/). Installing GTSAM on
Arch Linux is not tested by the GTSAM developers.

Install it manually by following the
[Arch Wiki instructions](https://wiki.archlinux.org/title/Arch_User_Repository)
or use an AUR helper such as `yay`:

```sh
yay -S gtsam
```

An Intel MKL-enabled package is also available:

```sh
yay -S gtsam-mkl
```

## Using GTSAM from a CMake project

After installing GTSAM, downstream CMake projects can use its exported target:

```cmake
find_package(GTSAM REQUIRED)

add_executable(my_program main.cpp)
target_link_libraries(my_program PRIVATE gtsam)
```

Linking the `gtsam` target supplies the required include directories and
transitive build requirements. If GTSAM was installed to a nonstandard prefix,
point CMake at it when configuring the downstream project:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/gtsam
```

See the complete
[`cmake/example_cmake_find_gtsam`](cmake/example_cmake_find_gtsam)
consumer example.

# Windows Installation

There are two ways to build GTSAM on Windows: the traditional way with Visual Studio and the modern way with CMake + Ninja. The CMake + Ninja way is preferred because the Ninja generator is much faster than Visual Studio.


**Important**: Regardless of how you build, GTSAM requires compiling with `/permissive-` and for all projects to also compile with `/permissive-` (due to lots of code being in headers) and sets the list of public compiler flags accordingly. If your project does not currently build with `/permissive-`, make sure it does and fix whatever is needed to make it work. Failure to compile with `/permissive-` can cause various runtime or build errors.

### Prerequisites

- Visual Studio with Desktop development with C++
  - You need MSVC and the Windows SDK to build GTSAM.
  - This also includes the C++ CMake tools for Windows component, which includes Ninja and CMake.
  - CMake 3.21 or newer is required when generating Visual Studio 2022 project files because that generator was added in CMake 3.21. GTSAM's minimum remains CMake 3.16 when using another supported generator such as Ninja. Use `cmake --version` in the VS Developer Command Prompt to check the selected CMake installation.
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
  - For `CMake generator`, select `Visual Studio 17 2022` and select `x64` as the target platform.
  - Save the settings (Ctrl + S).
4. Saving the CMake settings should automatically generate the cache. Otherwise, click on `Project -> Configure Cache`. This will generate the CMake build files (as seen in the Output window).
  - If `Visual Studio 17 2022` is not an available generator, install CMake 3.21 or newer and select that executable in `CMakeSettings > Advanced settings > CMake executable`.
5. The last step will generate a `GTSAM.sln` file in the `build` directory. At this point, GTSAM can be used as a regular Visual Studio project.

### Python Installation

To install the Python bindings on Windows:

Install [pyparsing>=3.2.5](https://github.com/pyparsing/pyparsing), [pybind-stubgen>=2.5.1](https://github.com/sizmailov/pybind11-stubgen), and [numpy>=1.11.0](https://numpy.org/) in the Python environment you wish to use. The tested development dependencies can all be installed as follows:

  ```bash
  pip install -r <gtsam_folder>/python/dev_requirements.txt
  ```

1. Follow the above steps for GTSAM general installation.
  - In the CMake settings variables, set `GTSAM_BUILD_PYTHON` to true and specify the desired environment's interpreter in the "CMake command arguments" field using `-DPYTHON_EXECUTABLE="<path to your python.exe>"`.
  - Confirm that the configure summary reports the intended interpreter. If an exact version must be requested, set `GTSAM_PYTHON_VERSION`; the wrapper and pybind11 version settings are derived from it.
2. Build the project (Build > Build All).
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

  - Debug            All error checking options on, no optimization. Use for development.
  - Release (default for single-configuration generators) Optimizations turned
    on, no debug symbols.
  - Timing           Adds ENABLE_TIMING flag to provide statistics on operation
  - Profiling        Standard configuration for use during profiling
  - RelWithDebInfo   Same as Release, but with debug symbols.
  - MinSizeRel       Optimize for binary size.
  - None             Do not apply configuration-specific build flags.

#### CMAKE_INSTALL_PREFIX

The install folder. The default is typically `/usr/local/`.
To configure to install to your home directory, you could execute:

```cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME ..```

#### GTSAM_TOOLBOX_INSTALL_PATH

The final destination for the installed MATLAB toolbox. If unset, it defaults
to `${CMAKE_INSTALL_PREFIX}/gtsam_toolbox`.

```cmake -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=$HOME/toolbox ..```

#### GTSAM_BUILD_UNSTABLE

Enable build and install for libgtsam_unstable library.
Set with the command line as follows:

```cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON ..```

  ON (Default for source checkouts): When enabled, `libgtsam_unstable` is built
  and installed with the same options as `libgtsam`. If tests are enabled, its
  unit tests are built as well. Its MATLAB toolbox is also generated when the
  MATLAB toolbox is enabled, under `gtsam_unstable`.
  OFF: If disabled, no `gtsam_unstable` code will be included in build or install.

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

1. Build in `Release` mode. It can be substantially faster than `Debug` mode.
2. Enable TBB for workloads that benefit from parallel execution. Small problems
    may instead be slower because task-dispatch overhead outweighs the benefit, so
    benchmark your workload with and without TBB. TBB's parallel tree traversal
    can also significantly increase memory usage. If memory is a concern, set
    `-DGTSAM_TBB_BOUNDED_MEMORY_GROWTH=ON` to disable parallel tree traversal
    while retaining other TBB benefits.
3. Try `GTSAM_BUILD_WITH_MARCH_NATIVE` and benchmark the result. This can improve
    performance but affects executable portability; the binary may not run on a
    system with an older or different processor architecture.
    Also note that all dependent projects *must* be compiled with the same flag, or
    seg-faults and other undefined behavior may result.
4. Possibly enable MKL. Please note that our benchmarks have shown that this helps only
    in very limited cases, and actually hurts performance in the usual case. We therefore
    recommend that you do *not* enable MKL, unless you have benchmarked it on
    your problem and have verified that it improves performance.


## Debugging tips

Another useful debugging symbol is _GLIBCXX_DEBUG, which enables debug checks and safe containers in the standard C++ library and makes problems much easier to find.

NOTE:  If _GLIBCXX_DEBUG is used to compile gtsam, anything that links against gtsam will need to be compiled with _GLIBCXX_DEBUG as well, due to the use of header-only Eigen.


## Installing oneMKL on Linux

Follow Intel's current [oneMKL APT installation instructions](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html) and install the development package:

```sh
sudo apt install intel-oneapi-mkl-devel
```

Initialize the oneAPI environment before configuring GTSAM. The standard
component-layout installation provides:

```sh
source /opt/intel/oneapi/setvars.sh
```

Installations using Intel's unified layout provide `oneapi-vars.sh` instead.
Then configure GTSAM with `-DGTSAM_WITH_EIGEN_MKL=ON`; add
`-DGTSAM_WITH_EIGEN_MKL_OPENMP=ON` when OpenMP-backed MKL threading is desired.
No manual `LD_PRELOAD` setting is required for the supported oneAPI layout.


## Compile gtsam with vcpkg

vcpkg is an easy, cross-platform way to install all the dependencies gtsam uses, including Boost, MKL, and pybind11. It will calculate the proper [triplet for your system](https://learn.microsoft.com/en-us/vcpkg/concepts/triplets), like x64-linux, x64-windows, or arm64-osx, and install dependencies accordingly. That triplet will be referred to as `<triplet>` in this guide.

To get started, install some base dependencies.

On Linux, install Python dependencies + ninja + build-essential:

```bash
sudo apt update
sudo apt-get install autoconf automake autoconf-archive ninja-build build-essential -y
```

On Windows, see [the Prerequisites section earlier](#Prerequisites).

On Mac, install Python dependencies + ninja:

```bash
brew install autoconf autoconf-archive automake libtool
```

Go to your gtsam folder `cd gtsam`, and set up vcpkg:

```bash
git clone https://github.com/microsoft/vcpkg
./vcpkg/bootstrap-vcpkg.sh # or ./vcpkg/bootstrap-vcpkg.bat on Windows
```

Setup vcpkg and Python dependencies

```bash
./vcpkg/vcpkg install

# Linux and macOS
./vcpkg_installed/<triplet>/tools/python3/python3 -m ensurepip --upgrade
./vcpkg_installed/<triplet>/tools/python3/python3 -m pip install -r python/dev_requirements.txt

# Windows (PowerShell)
./vcpkg_installed/<triplet>/tools/python3/python.exe -m ensurepip --upgrade
./vcpkg_installed/<triplet>/tools/python3/python.exe -m pip install -r python/dev_requirements.txt
```

Configure CMake build:
```bash
cmake -B build -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=vcpkg/scripts/buildsystems/vcpkg.cmake \
    -DVCPKG_INSTALLED_DIR=vcpkg_installed \
    -DVCPKG_TARGET_TRIPLET=<triplet> \
    -DVCPKG_HOST_TRIPLET=<triplet> \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_BUILD_TESTS=ON \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_USE_SYSTEM_METIS=OFF \
    -DGTSAM_USE_SYSTEM_PYBIND=ON \
    -DGTSAM_ENABLE_GEOGRAPHICLIB=ON \
    -DGTSAM_SUPPORT_NESTED_DISSECTION=ON \
    -DGTSAM_WITH_EIGEN_MKL=ON \
    -DGTSAM_WITH_EIGEN_MKL_OPENMP=ON
```

Build gtsam:

```bash
cmake --build build
```

Add vcpkg libraries to PATH:

Linux/Mac:
```bash
export PATH="$PATH:/path/to/gtsam/vcpkg_installed/<triplet>/bin"
```

Windows (PowerShell):
```pwsh
$env:Path = "$env:Path;\path\to\gtsam\vcpkg_installed\<triplet>\bin"
```

Run Python tests:
```bash
cmake --build build --target python-install
cmake --build build --target python-test
cmake --build build --target python-test-unstable
```

Run gtsam tests:
```bash
cmake --build build --target check
```
