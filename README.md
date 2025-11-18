# GTSAM: Georgia Tech Smoothing and Mapping Library
[![C++ API](https://img.shields.io/badge/API-C%2B%2B-blue.svg)](https://gtsam.org/doxygen/)
[![Docs](https://img.shields.io/badge/Docs-Python%20%7C%20C%2B%2B-green.svg)](https://borglab.github.io/gtsam/)

**Important Note**

**As of January 2023, the `develop` branch is officially in "Pre 4.3" mode. We envision several API-breaking changes as we switch to C++17 and away from boost.**

In addition, features deprecated in 4.2 will be removed. Please use the stable [4.2 release](https://github.com/borglab/gtsam/releases/tag/4.2) if you need those features. However, most are easily converted and can be tracked down (in 4.2) by disabling the cmake flag `GTSAM_ALLOW_DEPRECATED_SINCE_V42`.

## What is GTSAM?

GTSAM is a C++ library that implements smoothing and
mapping (SAM) in robotics and vision, using Factor Graphs and Bayes
Networks as the underlying computing paradigm rather than sparse
matrices.



<!-- Main CI Badges (develop branch) -->
| CI Status | Platform | Compiler |
|:----------|:---------|:---------|
| [![Python CI](https://github.com/borglab/gtsam/actions/workflows/build-python.yml/badge.svg?branch=develop)](https://github.com/borglab/gtsam/actions/workflows/build-python.yml?query=branch%3Adevelop) | Ubuntu 22.04, MacOS 13-14, Windows | gcc/clang,MSVC |
| [![vcpkg](https://github.com/borglab/gtsam/actions/workflows/vcpkg.yml/badge.svg?branch=develop)](https://github.com/borglab/gtsam/actions/workflows/vcpkg.yml?query=branch%3Adevelop) | Latest Windows/Ubuntu/Mac | - |
| [![Build Wheels for Develop](https://github.com/borglab/gtsam/actions/workflows/build-cibw.yml/badge.svg?branch=develop)](https://github.com/borglab/gtsam/actions/workflows/build-cibw.yml?query=branch%3Adevelop) | See [pypi files](https://pypi.org/project/gtsam-develop/#files); no Windows| - |

On top of the C++ library, GTSAM includes [wrappers for MATLAB & Python](#wrappers).


## Documentation

- **C++ API Docs:** [https://gtsam.org/doxygen/](https://gtsam.org/doxygen/)
- **Python API Docs:** [https://borglab.github.io/gtsam/](https://borglab.github.io/gtsam/)
<!-- TODO: Perhaps include links to source code as well? But the wrappers doesn't really help too much understanding the source code. 
C++: https://github.com/borglab/gtsam/tree/develop/gtsam
Matlab wrapper: https://github.com/borglab/gtsam/blob/develop/matlab/README.md
Python wrapper https://github.com/borglab/gtsam/blob/develop/python/README.md
-->


## Quickstart

In the root library folder execute:

```sh
#!bash
mkdir build
cd build
cmake ..
make check  # optional, runs all unit tests
make install
```

Prerequisites:

- A modern compiler:
    - Mac: at least xcode-14.2
    - Linux: at least clang-11 or gcc-9
    - Windows: at least msvc-14.2
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.9
    - Ubuntu: `sudo apt-get install cmake`

Optional Boost prerequisite:

Boost is now *optional*. Two cmake flags govern its behavior:
 - `GTSAM_USE_BOOST_FEATURES` = `ON|OFF`: some of our timers and concept checking in the tests still depend on boost.
 - `GTSAM_ENABLE_BOOST_SERIALIZATION` = `ON|OFF`: serialization of factor graphs, factors, etc still is done using boost

If one or both of these flags are `ON`, you need to install [Boost](http://www.boost.org/users/download/) >= 1.70
    - Mac: `brew install boost`
    - Ubuntu: `sudo apt-get install libboost-all-dev`
    - Windows: We highly recommend using the [vcpkg](https://github.com/microsoft/vcpkg) package manager. For other installation methods or troubleshooting, please see the guidance in the [cmake/HandleBoost.cmake](cmake/HandleBoost.cmake) script.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl) (Ubuntu: [installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo))
    - See [INSTALL.md](INSTALL.md) for more installation information
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.

## GTSAM 4 Compatibility

GTSAM 4 introduces several new features, most notably Expressions and a Python toolbox. It also introduces traits, a C++ technique that allows optimizing with non-GTSAM types. That opens the door to retiring geometric types such as Point2 and Point3 to pure Eigen types, which we also do. A significant change which will not trigger a compile error is that zero-initializing of Point2 and Point3 is deprecated, so please be aware that this might render functions using their default constructor incorrect.

 There is a flag `GTSAM_ALLOW_DEPRECATED_SINCE_V43` for newly deprecated methods since the 4.3 release, which is on by default, allowing anyone to just pull version 4.3 and compile.


## Wrappers

We provide support for [MATLAB](matlab/README.md) and [Python](python/README.md) wrappers for GTSAM. Please refer to the linked documents for more details.

## Citation

If you are using GTSAM for academic work, please use the following citation:

```bibtex
@software{gtsam,
  author       = {Frank Dellaert and GTSAM Contributors},
  title        = {borglab/gtsam},
  month        = May,
  year         = 2022,
  publisher    = {Georgia Tech Borg Lab},
  version      = {4.2a8},
  doi          = {10.5281/zenodo.5794541},
  url          = {https://github.com/borglab/gtsam)}}
}
```

To cite the `Factor Graphs for Robot Perception` book, please use:
```bibtex
@book{factor_graphs_for_robot_perception,
    author={Frank Dellaert and Michael Kaess},
    year={2017},
    title={Factor Graphs for Robot Perception},
    publisher={Foundations and Trends in Robotics, Vol. 6},
    url={http://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf}
}
```

If you are using the IMU preintegration scheme, please cite:
```bibtex
@book{imu_preintegration,
    author={Christian Forster and Luca Carlone and Frank Dellaert and Davide Scaramuzza},
    title={IMU preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation},
    year={2015}
}
```


## The Preintegrated IMU Factor

GTSAM includes a state of the art IMU handling scheme based on

- Todd Lupton and Salah Sukkarieh, _"Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions"_, TRO, 28(1):61-76, 2012. [[link]](https://ieeexplore.ieee.org/document/6092505)

Our implementation improves on this using integration on the manifold, as detailed in

- Luca Carlone, Zsolt Kira, Chris Beall, Vadim Indelman, and Frank Dellaert, _"Eliminating conditionally independent sets in factor graphs: a unifying perspective based on smart factors"_, Int. Conf. on Robotics and Automation (ICRA), 2014. [[link]](https://ieeexplore.ieee.org/abstract/document/6907483)
- Christian Forster, Luca Carlone, Frank Dellaert, and Davide Scaramuzza, _"IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation"_, Robotics: Science and Systems (RSS), 2015. [[link]](http://www.roboticsproceedings.org/rss11/p06.pdf)

If you are using the factor in academic work, please cite the publications above.

In GTSAM 4 a new and more efficient implementation, based on integrating on the NavState tangent space and detailed in [this document](doc/ImuFactor.pdf), is enabled by default. To switch to the RSS 2015 version, set the flag `GTSAM_TANGENT_PREINTEGRATION` to OFF.


## Additional Information

There is a [GTSAM users Google group](https://groups.google.com/forum/#!forum/gtsam-users) for general discussion.

Read about important [GTSAM-Concepts](doc/GTSAM-Concepts.md) here. A primer on GTSAM Expressions,
which support (superfast) automatic differentiation,
can be found on the [GTSAM wiki on BitBucket](https://bitbucket.org/gtborg/gtsam/wiki/Home).

See the [`INSTALL`](INSTALL.md) file for more detailed installation instructions.

GTSAM is open source under the BSD license, see the [`LICENSE`](LICENSE) and [`LICENSE.BSD`](LICENSE.BSD) files.

Please see the [`examples/`](examples) directory and the [`USAGE`](USAGE.md) file for examples on how to use GTSAM.

GTSAM was developed in the lab of [Frank Dellaert](http://www.cc.gatech.edu/~dellaert) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS.md).
