# GTSAM repository guidance

GTSAM is a C++17 library for factor graphs and sensor fusion, with Python and
MATLAB wrappers. Follow the existing code near your change and keep changes
focused.

## Public C++ APIs

* All functions in header files should have Doxygen-style API documentation
  using `/** */`, except small functions such as getters, which can use a
  single-line `///` comment. There is no need for `@brief`, `@param`, or similar
  tags when the prose is already clear.
* Use `///` for single-line documentation comments rather than `/** */`.
* Use meaningful variable names, for example `measurement` rather than `msm`;
  avoid abbreviations.
* On Windows, externally accessible library APIs must be explicitly exported.
  When adding or modifying public classes or functions, review and follow
  [`Using-GTSAM-EXPORT.md`](../Using-GTSAM-EXPORT.md), including its template
  specialization and header-only rules.

## C++ style

* Classes and types are `UpperCamelCase`; methods and functions are
  `lowerCamelCase`.
* Public fields in structs use plain names without a trailing underscore.
* Apart from those naming conventions, follow Google C++ style.
* Prefer concise, elegant examples. Use the fewest helpers needed and favor
  direct construction and small local functors or lambdas over adapter
  functions.
* When reviewing changes, flag overly complex or long functions and recommend
  breaking them into smaller functions.

### Eigen Fixed-Size and Dynamic Storage

Use fixed-size types such as `Matrix3`, `Matrix23`, and `Vector6` when
dimensions are known at compile time, especially for `OptionalJacobian`
outputs. Fixed-size types store their coefficients inline and avoid unnecessary
dynamic allocation and fixed/dynamic conversions.

Use `Matrix` and `Vector` when dimensions are determined at runtime or when an
existing API explicitly requires a dynamic type.

When assigning to a fixed-size Jacobian, prefer writing directly to it or using
a fixed-size temporary:

```cpp
if (H) {
  *H = Matrix3{{1.0, 0.0, 0.0},
               {0.0, 1.0, 0.0},
               {0.0, 0.0, 1.0}};
}
```

Use flat initializer lists for vectors and nested initializer lists for
matrices:

```cpp
Vector3 vector{1.0, 2.0, 3.0};
Matrix23 matrix{{1.0, 2.0, 3.0},
                {4.0, 5.0, 6.0}};
```

## Wrappers and vendored code

* When a C++ function is exposed in a wrapper `.i` file, parameter names must
  match exactly between the header declaration and the `.i` declaration.
  Update both whenever a wrapped parameter name changes.
* For templated classes and factors in wrappers, prefer the normal `.i`
  template mechanism over one handwritten wrapper class per instantiation.
  Let the wrapper generate names such as `AttitudeFactorRot3` from
  `template<...> class AttitudeFactor`.
* Do not add or retain C++ `using` or `typedef` aliases solely to manufacture
  wrapper names. Keep aliases only when they are useful in the C++ API.
* The top-level `wrap/` directory is a git subtree of `borglab/wrap`, not a
  submodule. Make wrapper-generator contributions in `borglab/wrap`, then sync
  them into GTSAM with `./update_wrap.sh [branch-or-tag]`, which defaults to
  `master`. Run the sync on a feature branch because it creates GTSAM commits,
  typically a squash followed by a merge commit. Do not edit `wrap/` directly
  for changes that belong upstream.
* Do not edit files under `gtsam/3rdparty/`. These are vendored third-party
  libraries, including Eigen; direct changes make upstream updates difficult.

## Tests

* Run validation relevant to the files changed.
* Run C++ tests from `build/` with `make -j6 testXXX.run`, replacing `testXXX`
  with the relevant test target.
* For Python commands in the standard local development environment, use the
  `py312` conda environment. Run wrapper tests with
  `cmake --build build --target python-test` or
  `cmake --build build --target python-test-unstable`, as appropriate.
* Documentation-only changes do not require unrelated C++ tests, but should
  still pass applicable documentation checks and `git diff --check`.

## Debugging indeterminate linear systems

* Use the correctly spelled `IndeterminateSystemException`. The former
  `IndeterminantLinearSystemException` name is available only through the
  GTSAM 4.3 deprecation machinery.
* Treat `nearbyVariable()` as the key where elimination detected the problem,
  not necessarily its source; the reported key depends on graph structure and
  elimination ordering.
* Remember that the exception also protects against nearly indeterminate
  systems. A mathematically full-rank graph can trigger it when elimination
  produces a Cholesky pivot that is very small relative to its original
  diagonal entry. This test is invariant to diagonal changes of variable units
  but still depends on elimination ordering. For example, a very strong finite
  prior combined with much looser measurement noise can expose a weakly
  observed direction, although the raw weight ratio alone may only reflect
  different variable units and is not sufficient evidence.
* Preserve the failing nonlinear graph, values, and ordering. Linearize at
  those values, request the Jacobian with an explicit ordering, and inspect its
  singular spectrum and null space. Prefer Jacobian rank analysis over a
  determinant or Hessian rank analysis because forming the Hessian squares the
  condition number.
* Check for missing gauge constraints, unused or accidental keys, disconnected
  components, degenerate geometry or motion, lost observability after
  marginalization, inconsistent units or noise scales, and negative curvature
  introduced by custom Hessian factors.
* Use a temporary, physically meaningful prior to test an observability
  hypothesis, then recompute rank. If the intended prior fixes a gauge exactly,
  prefer a hard constraint such as `noiseModel::Constrained::All(dimension)`;
  it expresses that intent without choosing an arbitrary extreme finite weight.
  Do not present damping or a dense solve as a fix unless the model itself
  becomes observable and well conditioned.
* See `gtsam/linear/doc/IndeterminateSystemException.ipynb` for a runnable
  Python example and a full diagnostic checklist.

## C++ test organization

* Avoid one large anonymous namespace collecting every helper at the top of a
  test file. Prefer small, named fixture-style namespaces containing both the
  helpers and tests that use them.
* Put at least one short `//` comment immediately above every `TEST()` explaining
  the behavior being verified.
* When using `/* ************************************************************************* */`
  dividers, put one immediately before each fixture namespace and one
  immediately after it closes. Do not put dividers between tests in the same
  fixture namespace:

  ```cpp
  /* ************************************************************************* */
  namespace my_fixture {

  // Verifies the expected behavior.
  TEST(Suite, Case) {}

  }  // namespace my_fixture
  /* ************************************************************************* */
  ```

## Notebooks

Notebooks in `*/doc/*.ipynb` and `*/examples/*.ipynb` should use this preamble:

1. Title and introductory Markdown cell.
2. Copyright Markdown cell tagged `remove-cell`.
3. Colab badge Markdown cell.
4. Colab installation code cell tagged `remove-cell`.
5. Imports and setup code cell.

Use the existing `remove-cell` metadata convention so documentation builds and
Colab behavior stay consistent.

For graphs and other notebook visualizations, always prefer Plotly when
possible so figures are interactive.
