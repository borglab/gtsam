---
name: annotate-pybind-adapters
description: Diagnose generated pybind compilation failures caused by wrapper interface signatures that differ from their C++ declarations, then apply the wrap `@pybind_lambda` annotation to only the affected methods, static methods, or global functions. Use for compile-annotate-regenerate migrations with full-signature callable-pointer casts, especially for omitted parameters, value/reference differences, synthetic container accessors, incompatible types, inheritance, or templates.
---

# Annotate Pybind Adapters

Use `@pybind_lambda` as an explicit escape hatch when a wrapper declaration is
an adapter rather than the exact C++ callable signature. Keep full-signature
callable-pointer casts as the default.

Read the repository instructions and the `Pybind Callable Adapters` section in
`DOCS.md` before editing an interface file.

## Workflow

1. Reproduce the generated pybind compilation failure with the narrowest
   available build target.
2. Locate the failing binding in generated C++ and map it back to one callable
   declaration in the wrapper `.i` file.
3. Inspect the real C++ header. Do not infer signature equivalence from the `.i`
   file or from generator heuristics.
4. Confirm that the old forwarding call is valid even though the generated
   full-signature `static_cast` does not match the C++ declaration.
5. Add the marker after any `template<...>` prefix and immediately before that
   callable:

   ```text
   @pybind_lambda
   ReturnType method(Arguments...);

   template<T = {double}>
   @pybind_lambda
   T templatedMethod(T value);
   ```

6. Regenerate the binding and verify the annotated declaration emits a lambda
   while nearby unannotated declarations still emit full-signature casts.
7. Re-run the failing compile target, relevant wrapper tests, and the full test
   suite prescribed by the repository.
8. Report each annotation and the concrete C++ signature mismatch that requires
   it.

Use the `py312` conda environment for Python commands in this workspace. In the
standalone wrap repository, run focused pytest tests and then:

```bash
conda run -n py312 python -m pytest tests
```

When working in an integrated build that provides the repository-prescribed
target, run its `make -j6 testXXX.run` target with the required permissions.

## Decision Rules

Annotate when the wrapper intentionally differs from C++, including:

- omitted underlying parameters with C++ defaults;
- wrapper values that bind to C++ references;
- synthetic value-returning interfaces such as `at` or `front` over reference
  returns;
- incompatible types or inherited/template declarations whose pointer signature
  does not match the wrapper spelling.

Do not annotate merely because a callable is overloaded or templated. A
full-signature cast selects an exact overload even when other overloads appear
only in the C++ header. The generator also retains its existing automatic
template-specialization and adapter cases.

## Guardrails

- Annotate one callable declaration at a time; never annotate a class or file in
  bulk.
- Do not add symbol-name lists, GTSAM-specific heuristics, or guessed mismatch
  detection to wrap.
- Do not change declaration/binding order or type-registration behavior while
  fixing an adapter compile failure.
- Preserve Python names, defaults, policies, docstrings, and overload exposure.
- Treat the marker as pybind-only. MATLAB output must remain unchanged.
- Remove no existing automatic lambdas unless the task explicitly changes their
  semantics.
