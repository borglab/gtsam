README - Georgia Tech Smoothing and Mapping library
===================================================

Quickstart

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

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl)

Tested compilers

- GCC 4.2-4.7
- OSX Clang 2.9-5.0
- OSX GCC 4.2
- MSVC 2010, 2012

Tested systems:

- Ubuntu 11.04 - 13.10
- MacOS 10.6 - 10.9
- Windows 7, 8

See the `INSTALL` file for more detailed installation instructions.

What is GTSAM?
==============

GTSAM is a library of C++ classes that implement smoothing and
mapping (SAM) in robotics and vision, using factor graphs and Bayes
networks as the underlying computing paradigm rather than sparse
matrices. 

GTSAM is open source under the BSD license, see the `LICENSE` file.

Please see the `examples/` directory and the `USAGE` file for examples on how to use GTSAM.

The library is organized according to the following directory structure:

    3rdparty      local copies of third party libraries - Eigen3 and CCOLAMD
    base          provides some base Math and data structures, as well as test-related utilities
    geometry      points, poses, tensors, etc
    inference     core graphical model inference such as factor graphs, junction trees, Bayes nets, Bayes trees 
    linear        inference specialized to Gaussian linear case, GaussianFactorGraph etc...
    nonlinear     non-linear factor graphs and non-linear optimization
    slam          SLAM and visual SLAM application code

This library contains unchanged copies of two third party libraries, with documentation 
of licensing as follows:

- CCOLAMD 2.73: Tim Davis' constrained column approximate minimum degree ordering library
    - http://www.cise.ufl.edu/research/sparse
    - Licenced under LGPL v2.1, provided in gtsam/3rdparty/CCOLAMD/Doc/lesser.txt
- Eigen 3.2:  General C++ matrix and linear algebra library
    - Licenced under MPL2, provided in gtsam/3rdparty/Eigen/COPYING.README (some code that is 3rd-party to Eigen is BSD and LGPL)
