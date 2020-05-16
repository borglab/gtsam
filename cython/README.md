# Python Wrapper

This is the Cython/Python wrapper around the GTSAM C++ library.

## Install

- if you want to build the gtsam python library for a specific python version (eg 2.7), use the `-DGTSAM_PYTHON_VERSION=2.7` option when running `cmake` otherwise the default interpreter will be used.
    - If the interpreter is inside an environment (such as an anaconda environment or virtualenv environment) then the environment should be active while building gtsam.
- This wrapper needs Cython(>=0.25.2), backports_abc>=0.5, and numpy. These can be installed as follows:

```bash
 pip install -r <gtsam_folder>/cython/requirements.txt
```

- For compatibility with gtsam's Eigen version, it contains its own cloned version of [Eigency](https://github.com/wouterboomsma/eigency.git),
named **gtsam_eigency**, to interface between C++'s Eigen and Python's numpy.

- Build and install gtsam using cmake with `GTSAM_INSTALL_CYTHON_TOOLBOX` enabled.
The wrapped module will be installed to `GTSAM_CYTHON_INSTALL_PATH`, which is
by default: `<your CMAKE_INSTALL_PREFIX>/cython`

- To use the library without installing system-wide: modify your `PYTHONPATH` to include the `GTSAM_CYTHON_INSTALL_PATH`:
```bash
export PYTHONPATH=$PYTHONPATH:<GTSAM_CYTHON_INSTALL_PATH>
```
- To install system-wide: run `make install` then navigate to `GTSAM_CYTHON_INSTALL_PATH` and run `python setup.py install`
    - (the same command can be used to install into a virtual environment if it is active)
    - note: if you don't want gtsam to install to a system directory such as `/usr/local`, pass `-DCMAKE_INSTALL_PREFIX="./install"` to cmake to install gtsam to a subdirectory of the build directory.
    - if you run `setup.py` from the build directory rather than the installation directory, the script will warn you with the message: `setup.py is being run from an unexpected location`.
      Before `make install` is run, not all the components of the package have been copied across, so running `setup.py` from the build directory would result in an incomplete package.

## Unit Tests

The Cython toolbox also has a small set of unit tests located in the
test directory. To run them:

```bash
 cd <your GTSAM_CYTHON_INSTALL_PATH>
 python -m unittest discover
```

## Writing Your Own Scripts

See the tests for examples.

### Some Important Notes:

- Vector/Matrix:
  + GTSAM expects double-precision floating point vectors and matrices.
    Hence, you should pass numpy matrices with dtype=float, or 'float64'.
  + Also, GTSAM expects *column-major* matrices, unlike the default storage
    scheme in numpy. Hence, you should pass column-major matrices to gtsam using
    the flag order='F'. And you always get column-major matrices back.
    For more details, see: https://github.com/wouterboomsma/eigency#storage-layout---why-arrays-are-sometimes-transposed
  + Passing row-major matrices of different dtype, e.g. 'int', will also work
    as the wrapper converts them to column-major and dtype float for you,
    using numpy.array.astype(float, order='F', copy=False).
    However, this will result a copy if your matrix is not in the expected type
    and storage order.

- Inner namespace: Classes in inner namespace will be prefixed by <innerNamespace>_ in Python.
Examples: noiseModel_Gaussian, noiseModel_mEstimator_Tukey

- Casting from a base class to a derive class must be done explicitly.
Examples:
```Python
      noiseBase = factor.noiseModel()
      noiseGaussian = dynamic_cast_noiseModel_Gaussian_noiseModel_Base(noiseBase)
```

## Wrapping Your Own Project That Uses GTSAM

- Set PYTHONPATH to include ${GTSAM_CYTHON_INSTALL_PATH}
  + so that it can find gtsam Cython header: gtsam/gtsam.pxd

- In your CMakeList.txt
```cmake
find_package(GTSAM REQUIRED) # Make sure gtsam's install folder is in your PATH
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Wrap
include(GtsamCythonWrap)
include_directories(${GTSAM_EIGENCY_INSTALL_PATH})
wrap_and_install_library_cython("your_project_interface.h"
                                "from gtsam.gtsam cimport *" # extra import of gtsam/gtsam.pxd Cython header
                                "your_install_path"
                                "libraries_to_link_with_the_cython_module"
                                "dependencies_which_need_to_be_built_before_the_wrapper"
                                )
#Optional: install_cython_scripts and install_cython_files. See GtsamCythonWrap.cmake.
```

## KNOWN ISSUES

  - Doesn't work with python3 installed from homebrew
    - size-related issue: can only wrap up to a certain number of classes: up to mEstimator!
    - Guess: 64 vs 32b? disutils Compiler flags?
  - Bug with Cython 0.24: instantiated factor classes return FastVector<size_t> for keys(), which can't be casted to FastVector<Key>
    - Upgrading to 0.25 solves the problem
  - Need default constructor and default copy constructor for almost every classes... :(
    - support these constructors by default and declare "delete" for special classes?


### TODO

- [ ] allow duplication of parent' functions in child classes. Not allowed for now due to conflicts in Cython.
- [ ] a common header for boost shared_ptr? (Or wait until everything is switched to std::shared_ptr in gtsam?)
- [ ] inner namespaces ==> inner packages?
- [ ] Wrap fixed-size Matrices/Vectors?


### Completed/Cancelled:

- [x] Fix Python tests: don't use " import <package> * ": Bad style!!! (18-03-17 19:50)
- [x] Unit tests for cython wrappers @done (18-03-17 18:45) -- simply compare generated files
- [x] Wrap unstable @done (18-03-17 15:30)
- [x] Unify cython/gtsam.h and the original gtsam.h @done (18-03-17 15:30)
- [x] 18-03-17: manage to unify the two versions by removing std container stubs from the matlab version,and keeping KeyList/KeyVector/KeySet as in the matlab version. Probably Cython 0.25 fixes the casting problem.
- [x] 06-03-17: manage to remove the requirements for default and copy constructors
- [ ] 25-11-16: Try to unify but failed. Main reasons are: Key/size_t, std containers, KeyVector/KeyList/KeySet. Matlab doesn't need to know about Key, but I can't make Cython to ignore Key as it couldn't cast KeyVector, i.e. FastVector<Key>, to FastVector<size_t>.
- [ ] Marginal and JointMarginal: revert changes @failed (17-03-17 11:00) -- Cython does need a default constructor! It produces cpp code like this: ```gtsam::JointMarginal __pyx_t_1;```  Users don't have to wrap this constructor, however.
- [x] Convert input numpy Matrix/Vector to float dtype and storage order 'F' automatically, cannot crash! @done (15-03-17 13:00)
- [x] Remove requirements.txt - Frank: don't bother with only 2 packages and a special case for eigency! @done (08-03-17 10:30)
- [x] CMake install script @done (25-11-16 02:30)
- [ ] [REFACTOR] better name for uninstantiateClass: very vague!! @cancelled (25-11-16 02:30) -- lazy
- [ ] forward declaration? @cancelled (23-11-16 13:00) - nothing to do, seem to work?
- [x] wrap VariableIndex: why is it in inference? If need to, shouldn't have constructors to specific FactorGraphs @done (23-11-16 13:00)
- [x] Global functions @done (22-11-16 21:00)
- [x] [REFACTOR] typesEqual --> isSameSignature @done (22-11-16 21:00)
- [x] Proper overloads (constructors, static methods, methods) @done (20-11-16 21:00)
- [x] Allow overloading methods. The current solution is annoying!!! @done (20-11-16 21:00)
- [x] Casting from parent and grandparents @done (16-11-16 17:00)
- [x] Allow overloading constructors. The current solution is annoying!!! @done (16-11-16 17:00)
- [x] Support "print obj" @done (16-11-16 17:00)
- [x] methods for FastVector: at, [], ...  @done (16-11-16 17:00)
- [x] Cython: Key and size_t: traits<size_t> doesn't exist @done (16-09-12 18:34)
- [x] KeyVector, KeyList, KeySet... @done (16-09-13 17:19)
- [x] [Nice to have] parse typedef @done (16-09-13 17:19)
- [x] ctypedef at correct places @done (16-09-12 18:34)
- [x] expand template variable type in constructor/static methods? @done (16-09-12 18:34)
- [x] NonlinearOptimizer: copy constructor deleted!!! @done (16-09-13 17:20)
- [x] Value: no default constructor @done (16-09-13 17:20)
- [x] ctypedef PriorFactor[Vector] PriorFactorVector @done (16-09-19 12:25)
- [x] Delete duplicate methods in derived class @done (16-09-12 13:38)
- [x] Fix return properly @done (16-09-11 17:14)
- [x] handle pair @done (16-09-11 17:14)
- [x] Eigency: ambiguous call: A(const T&) A(const Vector& v) and Eigency A(Map[Vector]& v) @done (16-09-11 07:59)
- [x] Eigency: Constructor: ambiguous construct from Vector/Matrix @done (16-09-11 07:59)
- [x] Eigency: Fix method template of Vector/Matrix: template argument is [Vector] while arugment is Map[Vector] @done (16-09-11 08:22)
- [x] Robust noise: copy assignment operator is deleted because of shared_ptr of the abstract Base class @done (16-09-10 09:05)
- [ ] Cython: Constructor: generate default constructor? (hack: if it's serializable?) @cancelled (16-09-13 17:20)
- [ ] Eigency: Map[] to Block @created(16-09-10 07:59) @cancelled (16-09-11 08:28)

- inference before symbolic/linear
- what's the purpose of "virtual" ??
