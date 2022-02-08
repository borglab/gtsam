MH-iSAM2 library 
================

What is MH-iSAM2?
-----------------

MH-iSAM2, or Multi-hypothesis Incremental Smoothing and Mapping (iSAM) Using Bayes Tree and Hypo-tree, is a library of C++ classes based on GTSAM library that implements multi-hypothesis inference and optimization for robotics and vision applications, especially for online simultaneous localization and mapping (SLAM) problems.

Different from most other optimizers such as g2o, Ceres, iSAM, or iSAM2, MH-iSAM2 models ambiguous measurements or data associations specifically as multi-mode factors (MMF), and solves for multiple possible solutions of the estimated states when the ambiguities cannot be resolved, or converges back to one single solution when sufficienct measurements/constraints are given.

Current version: 1.0 (based on [GTSAM 4](https://bitbucket.org/gtborg/gtsam/src/develop/))

Link to the paper: [MH-iSAM2]()


Quickstart
----------

```
#!bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Prerequisites (same as GTSAM 4):

- [Boost](http://www.boost.org/users/download/) >= 1.43 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 2.6 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl)

Notice: Most classes and functions in GTSAM are preserved in the MH-iSAM2 library, so you do not have to install GTSAM if you have MH-iSAM2 installed already. However new updates in GTSAM after 2018 are not included.

Warning: It is highly suggested to only use the Eigen library that has the exact same version as the one in gtsam/3rdparty/Eigen to avoid conflict when building your own project. E.g.: install Point Cloud Library (PCL) from source with the same version of Eigen.


Examples
--------

The examples of using MH-iSAM2 in a SLAM system can be found in: 

[examples/MH_ISAM2_TEST_city10000.cpp](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/examples/MH_ISAM2_TEST_city10000.cpp)

and 

[examples/MH_ISAM2_TEST_victoriaPark.cpp](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/examples/MH_ISAM2_TEST_victoriaPark.cpp)

More information of the examples are in [MH_example_tutorial.md](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/MH_example_tutorial.md).


Usage
-----

Usage guidance for building your own multi-hypothesis SLAM system is in [USAGE.md](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/USAGE.md).

Since MH-iSAM2 is based on (and still contains a large part of) GTSAM, most information in [USAGE_GTSAM.md](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/USAGE_GTSAM.md) can still be apllied to MH-iSAM2. The differences are that now Values can contain multiple hypotheses and Factors can contain multiple modes.


Additional Information
----------------------

MH-iSAM2 was developed in the [Robot Perception Lab](http://rpl.ri.cmu.edu/) of [Michael Kaess](http://www.cs.cmu.edu/~kaess/) at the [Robotics Institute](https://www.ri.cmu.edu/) of [Carnegie Mellon University](https://www.cmu.edu/).


