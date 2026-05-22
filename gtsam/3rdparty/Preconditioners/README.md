# Preconditioners

This library implements a set of useful algebraic (incomplete factorization-based) [preconditioners](https://en.wikipedia.org/wiki/Preconditioner) that can be used to accelerate the convergence of many iterative numerical linear-algebraic algorithms (such as the [conjugate gradient](https://en.wikipedia.org/wiki/Conjugate_gradient_method), [Lanczos](https://en.wikipedia.org/wiki/Lanczos_algorithm), or [LOBPCG](https://en.wikipedia.org/wiki/LOBPCG) methods).  It builds upon the incomplete Crout symmetric indefinite LDL' factorization provided by the [sym-ildl](https://cs.stanford.edu/people/paulliu/sym-ildl/html/index.html) library, extending its functionality to implement *inertia correction* (so that the constructed preconditioners are *positive-definite*, as required by many inexact linear-algebraic methods), as well as presenting an interface that permits easy integration with [Eigen](https://eigen.tuxfamily.org/index.php).

## Getting Started

This library can be built and exported as a CMake project.  The following installation instructions have been verified on Ubuntu 22.04:

*Step 1:*  Install dependencies

```
$ sudo apt-get install build-essential cmake-gui libeigen3-dev liblapack-dev libblas-dev libsuitesparse-dev
```

*Step 2:*  Clone the repository

```
$ git clone https://github.com/david-m-rosen/Preconditioners Preconditioners
```

*Step 3:*  Create build directory

```
$ cd Preconditioners && mkdir build
```

*Step 4:*  Configure build and generate Makefiles
```
$ cd build && cmake ..
```

*Step 5:*  Build the library

```
$ make -j
```

## References

We are making this software freely available in the hope that it will be useful to others. If you use our code in your own work, please cite our [paper](https://arxiv.org/abs/2207.05257), which describes the design of the inertia-corrected incomplete symmetric indefinite preconditioner implemented in the `ILDL` class:

```
@misc{Rosen2022Accelerating,
  title = {Accelerating Certifiable Estimation with Preconditioned Eigensolvers},
  author = {Rosen, David M.},
  month = may,
  year = {2022},
  publisher = {arXiv},
  doi = {10.48550/ARXIV.2207.05257},
  url = {https://arxiv.org/abs/2207.05257},
}
```

and the following [paper](https://dl.acm.org/doi/abs/10.1145/3054948) of Greif et al., which describes the design of the `sym-ildl` library that this project includes:

```
@article{Greif2017SymILDL,
title = {{SYM-ILDL}: Incomplete {$LDL\transpose$} Factorization of Symmetric Indefinite and Skew-Symmetric Matrices},
author = {Greif, C. and He, S. and Liu, P.},
journal = {{ACM} Trans. Math. Softw.},
volume = {44},
number = {1},
month = apr,
year = {2017},
}
```

## Copyright and License 

The `Preconditioners` software contained herein is copyright (C) 2016-2022 by David M. Rosen, and is distributed under the terms of the GNU Lesser General Public License (LGPL) version 3 (or later).  Please see the [LICENSE](https://github.com/david-m-rosen/Preconditioners/blob/master/LICENSE) for more information.

The modified version of the [sym-ildl](https://cs.stanford.edu/people/paulliu/sym-ildl/html/index.html) library redistributed with this project is released under the MIT license.  Please refer to the [license](https://github.com/david-m-rosen/Preconditioners/blob/master/SymILDL/License.md) file distributed with that project.

Contact: d.rosen@northeastern.edu
