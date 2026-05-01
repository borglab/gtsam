For reviewing PRs:
* All functions in header files should have doxygen-style API docs, /** */ style, except small functions like getters which can have single line /// comments, no need for @brief, @params etc
* Use /// for single-line comments rather than /** */
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.
* Flag overly complex or long/functions: break up in smaller functions
* On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.
* When adding or modifying public classes/methods, always review and follow `Using-GTSAM-EXPORT.md` before finalizing changes, including template specialization/export rules.
* If we add a C++ function to a `.i` file to expose it to the wrapper, we must ensure that the parameter names match exactly between the declaration in the header file and the declaration in the `.i`. Similarly, if we change any parameter names in a wrapped function in a header file, or change any parameter names in a `.i` file, we must change the corresponding function in the other file to reflect those changes.
* For templated classes/factors in wrappers, prefer the normal `.i` template mechanism over hand-writing one wrapper class per instantiation. Let the wrapper generate names such as `AttitudeFactorRot3` from `template<...> class AttitudeFactor`.
* Do not add or keep C++ `using`/`typedef` aliases solely to manufacture wrapper names. Only keep aliases when they are genuinely useful in the C++ API as well.
* Classes are Uppercase, methods and functions lowerMixedCase.
* Public fields in structs keep plain names (no trailing underscore).
* Apart from those naming conventions, we adopt Google C++ style.
* Prefer concise, elegant examples: use the fewest helpers possible, favor direct construction and small local functors/lambdas over extra adapter functions.
* Notebooks in `*/doc/*.ipynb` and `*/examples/*.ipynb` should follow the standard preamble:
  1) title/introduction markdown cell,
  2) copyright markdown cell tagged `remove-cell`,
  3) Colab badge markdown cell,
  4) Colab install code cell tagged `remove-cell`,
  5) imports/setup code cell.
  Use the same `remove-cell` tagging convention as existing notebooks so docs build and Colab behavior stay consistent.
* After any code change, always run relevant tests via `make -j6 testXXX.run` in the build folder $WORKSPACE/build. If in VS code, ask for escalated permissions if needed.
