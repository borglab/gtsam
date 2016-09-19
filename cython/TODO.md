
TODO:
☐ forward declaration?
☐ Global functions
☐ wrap VariableIndex: why is it in inference? If need to, shouldn't have constructors to specific FactorGraphs
☐ [REFACTOR] better name for uninstantiateClass: very vague!!
☐ [REFACTOR] typesEqual --> equalSignature
☐ Unify cython/gtsam.h and the original gtsam.h
☐ Proper overloads (constructors, static methods, methods)
☐ CMake install script

Completed/Cancelled:
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
- Need default constructor and default copy constructor for almost every class... :(
  ☐ support these constructors by default and declare "delete" for special classes?

Installation:
  ☐ Prerequisite: 
    - Users create venv and pip install requirements before compiling
    - Wrap cython script in gtsam/cython folder
  ☐ Install built module into venv?

Known issues:
  ☐ Doesn't work with python3 installed from homebrew
    - size-related issue: can only wrap up to a certain number of classes: up to mEstimator!
    - Guess: 64 vs 32b? disutils Compiler flags?