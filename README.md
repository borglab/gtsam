[![Build Status](https://travis-ci.com/borglab/gtsam.svg?branch=develop)](https://travis-ci.com/borglab/gtsam/)

# README - Georgia Tech Smoothing and Mapping library

## What is GTSAM?

GTSAM is a library of C++ classes that implement smoothing and
mapping (SAM) in robotics and vision, using factor graphs and Bayes
networks as the underlying computing paradigm rather than sparse
matrices.

On top of the C++ library, GTSAM includes a MATLAB interface (enable
GTSAM_INSTALL_MATLAB_TOOLBOX in CMake to build it). A Python interface
is under development.

## Quickstart

In the root library folder execute:

```sh
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make check (optional, runs unit tests)
$ make install
```

Prerequisites:

- [Boost](http://www.boost.org/users/download/) >= 1.43 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl) (Ubuntu: [installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo))
    - See [INSTALL.md](INSTALL.md) for more installation information
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.

## GTSAM 4 Compatibility

GTSAM 4 will introduce several new features, most notably Expressions and a python toolbox. We will also deprecate some legacy functionality and wrongly named methods, but by default the flag GTSAM_ALLOW_DEPRECATED_SINCE_V4 is enabled, allowing anyone to just pull V4 and compile. To build the python toolbox, however, you will have to explicitly disable that flag.

Also, GTSAM 4 introduces traits, a C++ technique that allows optimizing with non-GTSAM types. That opens the door to retiring geometric types such as Point2 and Point3 to pure Eigen types, which we will also do. A significant change which will not trigger a compile error is that zero-initializing of Point2 and Point3 will be deprecated, so please be aware that this might render functions using their default constructor incorrect.

## The Preintegrated IMU Factor

GTSAM includes a state of the art IMU handling scheme based on

- Todd Lupton and Salah Sukkarieh, "Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions", TRO, 28(1):61-76, 2012. [[link]](https://ieeexplore.ieee.org/document/6092505)

Our implementation improves on this using integration on the manifold, as detailed in

- Luca Carlone, Zsolt Kira, Chris Beall, Vadim Indelman, and Frank Dellaert, "Eliminating conditionally independent sets in factor graphs: a unifying perspective based on smart factors", Int. Conf. on Robotics and Automation (ICRA), 2014. [[link]](https://ieeexplore.ieee.org/abstract/document/6907483)
- Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", Robotics: Science and Systems (RSS), 2015. [[link]](http://www.roboticsproceedings.org/rss11/p06.pdf)

If you are using the factor in academic work, please cite the publications above.

In GTSAM 4 a new and more efficient implementation, based on integrating on the NavState tangent space and detailed in docs/ImuFactor.pdf, is enabled by default. To switch to the RSS 2015 version, set the flag **GTSAM_TANGENT_PREINTEGRATION** to OFF.


## Additional Information

There is a [`GTSAM users Google group`](https://groups.google.com/forum/#!forum/gtsam-users) for general discussion.

Read about important [`GTSAM-Concepts`](GTSAM-Concepts.md) here. A primer on GTSAM Expressions,
which support (superfast) automatic differentiation,
can be found on the [GTSAM wiki on BitBucket](https://bitbucket.org/gtborg/gtsam/wiki/Home).

See the [`INSTALL`](INSTALL.md) file for more detailed installation instructions.

GTSAM is open source under the BSD license, see the [`LICENSE`](LICENSE) and [`LICENSE.BSD`](LICENSE.BSD) files.

Please see the [`examples/`](examples) directory and the [`USAGE`](USAGE.md) file for examples on how to use GTSAM.

GTSAM was developed in the lab of [Frank Dellaert](http://www.cc.gatech.edu/~dellaert) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS).
