# Using GTSAM

This guide summarizes how to consume an installed GTSAM library and introduces
the main concepts used to construct and optimize factor graphs.

## Install GTSAM

Build and install GTSAM by following [INSTALL.md](INSTALL.md). Run the `check`
target when validating a local build.

## Compile and link with CMake

An installation provides CMake package configuration files and the exported
`gtsam` target. Link that target instead of adding include directories or
third-party libraries manually:

```cmake
find_package(GTSAM REQUIRED)

add_executable(my_program main.cpp)
target_link_libraries(my_program PRIVATE gtsam)
```

The exported target supplies GTSAM's include directories, required compiler
settings, and the dependencies enabled when GTSAM was built. For an installation
under a nonstandard prefix, configure the consuming project with:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/gtsam
```

See [`cmake/example_cmake_find_gtsam`](cmake/example_cmake_find_gtsam) for a
complete consuming project.

## Examples

Runnable programs under [`examples/`](examples) cover SLAM, structure from
motion, navigation, discrete inference, robust optimization, and other common
workflows. Unit tests beside each module provide smaller examples of individual
APIs.

## Core concepts

- **Factor graphs** contain variables and factors. A factor expresses a
  measurement, constraint, or cost involving one or more variables.
- **Keys** identify variables. `gtsam::Key` is a 64-bit unsigned integer;
  `gtsam::Symbol` and `gtsam::LabeledSymbol` provide readable structured keys.
- **Values** stores typed variable values indexed by keys and supplies the
  linearization point or initial estimate used by nonlinear optimizers.
- **Optimizers and inference algorithms** operate on factor graphs and values to
  compute estimates, marginals, or discrete assignments.

## Source layout

The public C++ library is organized under `gtsam/`:

- `base`, `geometry`, and `basis` provide foundational mathematical types.
- `inference`, `linear`, `nonlinear`, and `symbolic` provide the core graphical
  model and optimization machinery.
- `discrete` and `hybrid` provide discrete and mixed discrete-continuous
  inference.
- `navigation`, `sam`, `sfm`, and `slam` provide robotics and vision factors and
  algorithms.
- `constrained` and `certifiable` provide constrained and certifiable
  optimization tools.
- `3rdparty` contains vendored dependencies used by the build.
