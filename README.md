README - Georgia Tech Smoothing and Mapping library
===================================================

What is GTSAM?
--------------

GTSAM is a library of C++ classes that implement smoothing and
mapping (SAM) in robotics and vision, using factor graphs and Bayes
networks as the underlying computing paradigm rather than sparse
matrices.

On top of the C++ library, GTSAM includes a MATLAB interface (enable
GTSAM_INSTALL_MATLAB_TOOLBOX in CMake to build it). A Python interface
is under development.

Quickstart
----------

In the root library folder execute:

```
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```

Prerequisites:

- [Boost](http://www.boost.org/users/download/) >= 1.43 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 2.6 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl)

Additional Information
----------------------

Read about important [`GTSAM-Concepts`](GTSAM-Concepts.md) here.

See the [`INSTALL`](INSTALL) file for more detailed installation instructions.

GTSAM is open source under the BSD license, see the [`LICENSE`](LICENSE) and [`LICENSE.BSD`](LICENSE.BSD) files.

Please see the [`examples/`](examples) directory and the [`USAGE`](USAGE.md) file for examples on how to use GTSAM.

GTSAM was developed in the lab of [Frank Dellaert](http://www.cc.gatech.edu/~dellaert) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS).