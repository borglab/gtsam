# Creating a `Manifold` Class

This guide explains the minimal requirements for creating a new `Manifold` class in GTSAM. The manifold is the most fundamental concept in GTSAM for representing non-linear spaces. It also serves as the base for more specialized types like `LieGroup`.

GTSAM uses a combination of two design patterns:

1.  **Traits-based Metaprogramming**: Instead of requiring your class to inherit from a specific base class, GTSAM uses a `traits` helper struct. You specialize this `traits` struct for your class to "teach" GTSAM's generic algorithms how to interact with it.
2.  **Concept Checking**: GTSAM provides macros that perform compile-time checks to ensure your class correctly implements all the requirements of a concept (like `Manifold`).

More details on these mechanisms can be found in [GTSAM-Concepts](GTSAM-Concepts.md).

### 1. The Core Idea: Retraction and Local Coordinates

A manifold is defined by two fundamental operations that must be inverses of each other:

*   **`retract(const TangentVector& v) const`**: This operation maps a vector `v` from the tangent space at the current point (`*this`) back onto the manifold, producing a new point on the manifold. It's a generalization of vector addition.
*   **`localCoordinates(const ManifoldType& other) const`**: This operation computes the tangent space vector that connects the current point (`*this`, the origin) to another point `other` on the manifold. It's a generalization of vector subtraction.

These operations must satisfy the invariant that `a.retract(a.localCoordinates(b))` should be approximately equal to `b`.

### 2. Minimal Implementation Requirements

To create a new class `MyManifold`, you must provide the following components. Note that your class itself does not inherit from anything.

#### `MyManifold.h` - Header File Requirements

1.  **Class Definition**:
    *   Define your class, `MyManifold`. It should contain the necessary data members to represent its state.

2.  **Essential Typedefs and Constants**:
    *   `static const int dimension = D;`
    *   `using TangentVector = gtsam::VectorD;`
    *   `using ChartJacobian = gtsam::OptionalJacobian<D, D>;`

3.  **Core Manifold Methods**:
    *   `Dim()` returns positive integer or Eigen::Dynamic
    *   `dim()` returns fixed-size or dynamic size, if dimension==Eigen::Dynamic
    *   `MyManifold retract(const TangentVector& v, ChartJacobian H_this = {}, ChartJacobian H_v = {}) const;`
    *   `TangentVector localCoordinates(const MyManifold& other, ChartJacobian H_this = {}, ChartJacobian H_other = {}) const;`

4.  **`Testable` Concept Utilities**:
    *   The `Manifold` concept requires the `Testable` concept. You must implement:
        *   `void print(const std::string& s = "") const;`
        *   `bool equals(const MyManifold& other, double tol = 1e-9) const;`

5.  **GTSAM Traits Specialization**:
    *   Below your class definition, you must specialize the `gtsam::traits` struct. This is how you register your class with GTSAM's framework. The `internal::Manifold` helper bundles the `ManifoldTraits` and `Testable` traits.
    ```cpp
    namespace gtsam {
    template <>
    struct traits<MyManifold> : public internal::Manifold<MyManifold> {};
    
    template <>
    struct traits<const MyManifold> : public internal::Manifold<MyManifold> {};
    }
    ```

### 3. How the `traits` Specialization Works

By providing the `traits` specialization, you are not adding methods to your class directly. Instead, you are enabling GTSAM's generic, global functions to work with your type. When a GTSAM algorithm calls a function like:

```cpp
gtsam::traits<MyManifold>::Local(origin, other);
```

The compiler, via your traits specialization, translates this into the member function call you implemented:

```cpp
origin.localCoordinates(other);
```

This design keeps your class clean of framework-specific inheritance while allowing full integration with GTSAM's generic programming model.

### 4. Concept Checking
To ensure your implementation is correct, add the following macro to your unit test file (testMyManifold.cpp). It will produce a compile-time error if any of the Manifold requirements are missing.

```cpp
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Manifold.h>

// Your class include
#include <gtsam/geometry/MyManifold.h>

// This macro will fail to compile if MyManifold is not a valid Manifold.
GTSAM_CONCEPT_MANIFOLD_INST(MyManifold)

// ... your unit tests ...
```

This is the most effective way to verify that your class fulfills its contract with the GTSAM framework.
