# Information for Developers

### Coding Conventions

* Classes are Uppercase, methods and functions lowerMixedCase.
* Apart from those naming conventions, we adopt Google C++ style.
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.


### Windows

On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.

For example:
```cpp
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT return_type myFunction();
```

More details [here](Using-GTSAM-EXPORT.md).
