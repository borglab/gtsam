# README - Georgia Tech Smoothing and Mapping Library

**Important Note**

As of Dec 2021, the `develop` branch is officially in "Pre 4.2" mode. A great new feature we will be adding in 4.2 is *hybrid inference* a la DCSLAM (Kevin Doherty et al) and we envision several API-breaking changes will happen in the discrete folder.

In addition, features deprecated in 4.1 will be removed. Please use the last [4.1.1 release](https://github.com/borglab/gtsam/releases/tag/4.1.1) if you need those features. However, most (not all, unfortunately) are easily converted and can be tracked down (in 4.1.1) by disabling the cmake flag `GTSAM_ALLOW_DEPRECATED_SINCE_V42`.

## What is GTSAM?

GTSAM is a C++ library that implements smoothing and
mapping (SAM) in robotics and vision, using Factor Graphs and Bayes
Networks as the underlying computing paradigm rather than sparse
matrices.

The current support matrix is:

| Platform     | Compiler  | Build Status  |
|:------------:|:---------:|:-------------:|
| Ubuntu 18.04 | gcc/clang | ![Linux CI](https://github.com/borglab/gtsam/workflows/Linux%20CI/badge.svg) |
| macOS        | clang     | ![macOS CI](https://github.com/borglab/gtsam/workflows/macOS%20CI/badge.svg) |
| Windows      | MSVC      | ![Windows CI](https://github.com/borglab/gtsam/workflows/Windows%20CI/badge.svg) |


On top of the C++ library, GTSAM includes [wrappers for MATLAB & Python](#wrappers).


## Quickstart

In the root library folder execute:

```sh
#!bash
mkdir build
cd build
cmake ..
make check (optional, runs unit tests)
make install
```

Prerequisites:

- [Boost](http://www.boost.org/users/download/) >= 1.65 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl) (Ubuntu: [installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo))
    - See [INSTALL.md](INSTALL.md) for more installation information
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.

## GTSAM 4 Compatibility

GTSAM 4 introduces several new features, most notably Expressions and a Python toolbox. It also introduces traits, a C++ technique that allows optimizing with non-GTSAM types. That opens the door to retiring geometric types such as Point2 and Point3 to pure Eigen types, which we also do. A significant change which will not trigger a compile error is that zero-initializing of Point2 and Point3 is deprecated, so please be aware that this might render functions using their default constructor incorrect.

GTSAM 4 also deprecated some legacy functionality and wrongly named methods. If you are on a 4.0.X release, you can define the flag `GTSAM_ALLOW_DEPRECATED_SINCE_V4` to use the deprecated methods.

GTSAM 4.1 added a new pybind wrapper, and **removed** the deprecated functionality. There is a flag `GTSAM_ALLOW_DEPRECATED_SINCE_V42` for newly deprecated methods since the 4.1 release, which is on by default, allowing anyone to just pull version 4.1 and compile.


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

There is a [`GTSAM users Google group`](https://groups.google.com/forum/#!forum/gtsam-users) for general discussion.

Read about important [`GTSAM-Concepts`](GTSAM-Concepts.md) here. A primer on GTSAM Expressions,
which support (superfast) automatic differentiation,
can be found on the [GTSAM wiki on BitBucket](https://bitbucket.org/gtborg/gtsam/wiki/Home).

See the [`INSTALL`](INSTALL.md) file for more detailed installation instructions.

GTSAM is open source under the BSD license, see the [`LICENSE`](LICENSE) and [`LICENSE.BSD`](LICENSE.BSD) files.

Please see the [`examples/`](examples) directory and the [`USAGE`](USAGE.md) file for examples on how to use GTSAM.

GTSAM was developed in the lab of [Frank Dellaert](http://www.cc.gatech.edu/~dellaert) at the [Georgia Institute of Technology](http://www.gatech.edu), with the help of many contributors over the years, see [THANKS](THANKS.md).
