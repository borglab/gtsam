This is the Cython/Python wrapper around the GTSAM C++ library.

INSTALL
=======
- This wrapper needs Cython(>=0.25), numpy and eigency, which can be installed 
as follows:

```bash
 cd <gtsam_folder>/cython
 pip install -r requirements.txt
 pip install eigency
```

Note: Currently there's some issue with including eigency in requirements.txt

- Build and install gtsam using cmake with GTSAM_INSTALL_CYTHON_TOOLBOX enabled
Note: By default, the wrapped module will be installed in 
<your_installation_folder>/gtsam_cython. Change that in GTSAM_CYTHON_TOOLBOX_PATH


UNIT TESTS
==========
The Cython toolbox also has a small set of unit tests located in the
test directory. To run them:

```bash
 cd /Users/yourname/gtsam_cython  # Change to wherever you installed the toolbox
 python -m unittest discover
```

WRITING YOUR OWN SCRIPTS
========================
See the tests for examples.

## Some important notes:

- Vector/Matrix: Due to a design choice of eigency, numpy.array matrices with the default order='A'
will always be transposed in C++ no matter how you transpose it in Python. Use order='F', or use
two functions Vector and Matrix in gtsam_utils/np_utils.py for your conveniences. These two functions
also help to avoid a common but very subtle bug of using integers when creating numpy arrays, 
e.g. np.array([1,2,3]). These can't be an input for gtsam functions as they only accept floating-point arrays.
For more details, see: https://github.com/wouterboomsma/eigency#storage-layout---why-arrays-are-sometimes-transposed

- Inner namespace: Classes in inner namespace will be prefixed by <innerNamespace>_ in Python.
Examples: noiseModel_Gaussian, noiseModel_mEstimator_Tukey

- Use keyword arguments to differentiate overloads with the same number of arguments. 
This applies for all constructors, methods and static methods.
Now the variable names become important in these cases!!!
Examples: Pose3(t=Matrix([...])), Pose3(pose2=Pose2()), Pose3(other=Pose3())
Pose2(v=Vector([1,2,3])), Pose2(other=otherPose2)

- Values::insert and update are now templated functions. Their python versions have the corresponding instantiated types added. 
Examples: values.insertPose3(1, Pose3()), values.insertRot3(2, Rot3()), values.insertimuBias_ConstantBias(3, bias)

- Casting from a base class to a derive class must be done explicitly.
Examples: 
```Python
      noiseBase = factor.get_noiseModel()
      noiseGaussian = dynamic_cast_noiseModel_Gaussian_noiseModel_Base(noiseBase)       
```



KNOWN ISSUES
============
  - Doesn't work with python3 installed from homebrew
    - size-related issue: can only wrap up to a certain number of classes: up to mEstimator!
    - Guess: 64 vs 32b? disutils Compiler flags?
  - Bug with Cython 0.24: instantiated factor classes return FastVector<size_t> for keys(), which can't be casted to FastVector<Key>
    - Upgrading to 0.25 solves the problem 
  - Need default constructor and default copy constructor for almost every classes... :(
    - support these constructors by default and declare "delete" for special classes?


TODO
=====
☐ Unify cython/gtsam.h and the original gtsam.h
  - 25-11-16:
    Try to unify but failed. Main reasons are: Key/size_t, std containers, KeyVector/KeyList/KeySet. 
    Matlab doesn't need to know about Key, but I can't make Cython to ignore Key as it couldn't cast KeyVector, i.e. FastVector<Key>,
    to FastVector<size_t>. 

Completed/Cancelled:
✔ CMake install script @done (25-11-16 02:30)
✘ [REFACTOR] better name for uninstantiateClass: very vague!! @cancelled (25-11-16 02:30) -- lazy
✘ forward declaration? @cancelled (23-11-16 13:00) - nothing to do, seem to work?
✔ wrap VariableIndex: why is it in inference? If need to, shouldn't have constructors to specific FactorGraphs @done (23-11-16 13:00)
✔ Global functions @done (22-11-16 21:00)
✔ [REFACTOR] typesEqual --> isSameSignature @done (22-11-16 21:00)
✔ Proper overloads (constructors, static methods, methods) @done (20-11-16 21:00)
✔ Allow overloading methods. The current solution is annoying!!! @done (20-11-16 21:00)
✔ Casting from parent and grandparents @done (16-11-16 17:00)
✔ Allow overloading constructors. The current solution is annoying!!! @done (16-11-16 17:00)
✔ Support "print obj" @done (16-11-16 17:00)
✔ methods for FastVector: at, [], ...  @done (16-11-16 17:00)
✔ Cython: Key and size_t: traits<size_t> doesn't exist @done (16-09-12 18:34)
✔ KeyVector, KeyList, KeySet... @done (16-09-13 17:19)
✔ [Nice to have] parse typedef @done (16-09-13 17:19)
✔ ctypedef at correct places @done (16-09-12 18:34)
✔ expand template variable type in constructor/static methods? @done (16-09-12 18:34)
✔ NonlinearOptimizer: copy constructor deleted!!! @done (16-09-13 17:20)
✔ Value: no default constructor @done (16-09-13 17:20)
✔ ctypedef PriorFactor[Vector] PriorFactorVector @done (16-09-19 12:25)
✔ Delete duplicate methods in derived class @done (16-09-12 13:38)
✔ Fix return properly @done (16-09-11 17:14)
 ✔ handle pair @done (16-09-11 17:14)
✔ Eigency: ambiguous call: A(const T&) A(const Vector& v) and Eigency A(Map[Vector]& v) @done (16-09-11 07:59)
✔ Eigency: Constructor: ambiguous construct from Vector/Matrix @done (16-09-11 07:59)
✔ Eigency: Fix method template of Vector/Matrix: template argument is [Vector] while arugment is Map[Vector] @done (16-09-11 08:22)
✔ Robust noise: copy assignment operator is deleted because of shared_ptr of the abstract Base class @done (16-09-10 09:05)
✘ Cython: Constructor: generate default constructor? (hack: if it's serializable?) @cancelled (16-09-13 17:20)
✘ Eigency: Map[] to Block @created(16-09-10 07:59) @cancelled (16-09-11 08:28)

- inference before symbolic/linear
- what's the purpose of "virtual" ??

Installation:
  ☐ Prerequisite: 
    - Users create venv and pip install requirements before compiling
    - Wrap cython script in gtsam/cython folder
  ☐ Install built module into venv?
