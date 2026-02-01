# Information for Developers

## Coding Conventions

* Classes are Uppercase, methods and functions lowerMixedCase.
* Apart from those naming conventions, we adopt Google C++ style.
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.

### Header-Wrapper Parameter Name Matching

If you add a C++ function to a `.i` file to expose it to the wrapper, you must ensure that the parameter names match exactly between the declaration in the header file and the declaration in the `.i`. Similarly, if you change any parameter names in a wrapped function in a header file, or change any parameter names in a `.i` file, you must change the corresponding function in the other file to reflect those changes. 

This includes inherited functions wrapped in `.i` files. If `Pose2` inherits `logmap` from `LieGroup` (in `Lie.h`) and wraps `logmap`, `Pose2.logmap` in the `.i` needs to have the same parameter names as the function definition in `Lie.h`. 

> [!IMPORTANT]
> The Doxygen documentation from the C++ will not carry over into the Python docstring if the parameter names do not match exactly!

If you encounter any functions that do not meet this criterion, please submit a PR to make them match.

## Wrapper Maintenance: How to fix C++ GTSAM functions not showing up in Python

The GTSAM Python wrapper is created using the [wrap](https://github.com/borglab/wrap) library ([docs](https://github.com/borglab/wrap/blob/master/DOCS.md)). The following brief guide is intended to help users extend the GTSAM wrapper themselves if necessary.

The Python wrapper for a class is defined in the `*.i` interface file present in the same directory as the class. For example, the wrapper for `gtsam/geometry/Pose3.h` is defined in `gtsam/geometry/geometry.i`; for `gtsam/navigation/ImuFactor.h`, it's `gtsam/navigation/navigation.i`, etc. With that knowledge and following these steps, you can manipulate your local clone of GTSAM and rebuild the Python package with your custom extended bindings.

1. Follow steps to clone, build, and install GTSAM with Python bindings on your OS, to be sure that you can do so before changing the source code. See [INSTALL.md](INSTALL.md) and the [Python README.md](python/README.md).
2. Identify which folder your problem class is in and the corresponding interface file.
3. Edit the interface file to include your desired functions from C++. Largely, this consists of simply copying the function signature into the appropriate class of the `.i`, but you also must make sure that everything is explicitly referenced (e.g. with `gtsam::`, `Eigen::`). In some cases, such as with Vectors or Jacobians, you may have to change a return type or argument type. In such situations, let the similar code in the `.i` be your guide. Commonly, `VectorN` can be changed to `gtsam::Vector`.
4. Rebuild GTSAM with Python bindings and reinstall the Python package.
5. At this point, your function should be available after `import gtsam`.

### Possible remaining issues

- If the source compiled fine, any build issues will be a consequence of errors in your new wrapper code. Look closely for missing namespaces, semicolons, types, etc. See the [wrap docs](https://github.com/borglab/wrap/blob/master/DOCS.md) for syntax guidelines.
- If the new function won't show up in Python, make sure you have properly reinstalled the Python package. You might need to `pip uninstall` before reinstalling. On Windows, you might need to recopy the `.pyd` files and then rebuild as mentioned in the Windows installation instructions.

## Windows

On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.

For example:
```cpp
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT return_type myFunction();
```

More details [here](Using-GTSAM-EXPORT.md).
