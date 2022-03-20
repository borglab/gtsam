# Information for Developers

### Coding Conventions

* Classes are Uppercase, methods and functions lowerMixedCase.
* We use a modified K&R Style, with 2-space tabs, inserting spaces for tabs.
* Use meaningful variable names, e.g. `measurement` not `msm`.


### Windows

On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.

For example:
```cpp
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT return_type myFunction();
```

More details [here](Using-GTSAM-EXPORT.md).
