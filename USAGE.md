# GTSAM USAGE

This file explains how to make use of the library for common SLAM tasks, using a visual SLAM implementation as an example.

## Getting Started

### Install
	
Follow the installation instructions in the README file to build and install gtsam, as well as running tests to ensure the library is working properly.

### Compiling/Linking with GTSAM

The installation creates a binary `libgtsam` at the installation prefix, and an include folder `gtsam`.  These are the only required includes, but the library has also been designed to make use of XML serialization through the `Boost.serialization` library, which requires the the Boost.serialization headers and binaries to be linked.  

If you use CMake for your project, you can use the CMake scripts in the cmake folder for finding `GTSAM`, `CppUnitLite`, and `Wrap`.  

### Examples

To see how the library works, examine the unit tests provided.  
 
## Overview

The GTSAM library has three primary components necessary for the construction of factor graph representation and optimization which users will need to adapt to their particular problem.  

* FactorGraph

	A factor graph contains a set of variables to solve for (i.e., robot poses, landmark poses, etc.) and a set of constraints between these variables, which make up factors.

* Values:

	Values is a single object containing labeled values for all of the variables.  Currently, all variables are labeled with strings, but the type or organization of the variables can change.

* Factors

	A nonlinear factor expresses a constraint between variables, which in the SLAM example, is a measurement such as a visual reading on a landmark or odometry.

The library is organized according to the following directory structure:

    3rdparty      local copies of third party libraries e.g. Eigen3 and CCOLAMD
    base          provides some base Math and data structures, as well as test-related utilities
    geometry      points, poses, tensors, etc
    inference     core graphical model inference such as factor graphs, junction trees, Bayes nets, Bayes trees 
    linear        inference specialized to Gaussian linear case, GaussianFactorGraph etc...
    nonlinear     non-linear factor graphs and non-linear optimization
    slam          SLAM and visual SLAM application code



