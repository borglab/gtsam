For reviewing PRs:
* All functions in header files should have doxygen-style API docs
* Use /// for single-line comments rather than /** */
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.
* Flag overly complex or long/functions: break up in smaller functions
* On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.
* If we add a C++ function to a `.i` file to expose it to the wrapper, we must ensure that the parameter names match exactly between the declaration in the header file and the declaration in the `.i`. Similarly, if we change any parameter names in a wrapped function in a header file, or change any parameter names in a `.i` file, we must change the corresponding function in the other file to reflect those changes.
* Classes are Uppercase, methods and functions lowerMixedCase.
* Apart from those naming conventions, we adopt Google C++ style.
