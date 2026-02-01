# Creating a `LieGroup`

This guide explains how to create a `LieGroup` class, which builds upon the `Manifold` concept. **Please read [Manifold.md](Manifold.md) first**, as it explains the foundational traits-based design and concept checking used in GTSAM.

A Lie group is a manifold that is also a group, with smooth group operations. In GTSAM, this means it has all the properties of a `Manifold` plus notions of composition, identity, and inversion. GTSAM uses the [Curiously Recurring Template Pattern (CRTP)](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) to automatically provide many of the more complex manifold and group methods.

### 1. From Manifold to Lie Group

A Lie group has a special element: the **identity element**. This allows us to define global `Expmap` and `Logmap` operations that are centered at this identity. The `retract` and `localCoordinates` methods required by the `Manifold` concept are then implemented in terms of these identity-centered operations.

To implement a Lie group, you inherit from `gtsam::LieGroup<MyGroup, D>`.

### 2. Minimal Implementation Requirements for `MyGroup`

#### `MyGroup.h` - Header File Requirements

1.  **Class Definition and Inheritance**:
    *   Inherit publicly from `gtsam::LieGroup<MyGroup, D>`, where `D` is the dimension of the group's tangent space.

2.  **Essential Typedefs and Constants**:
    *   `static const size_t dimension = D;`
    *   `using ChartJacobian = gtsam::OptionalJacobian<D, D>;`

3.  **Constructors**:
    *   A default constructor that initializes to the identity element.

4.  **Group Primitives**:
    *   `static MyGroup Identity();`
    *   `MyGroup inverse() const;`
    *   `MyGroup operator*(const MyGroup& other) const;`

5.  **Lie Group Primitives**:
    *   `static MyGroup Expmap(const gtsam::VectorD& v, ChartJacobian H = {});`: The Exponential map *at identity*. Must support an optional derivative.
    *   `static gtsam::VectorD Logmap(const MyGroup& p, ChartJacobian H = {});`: The Logarithm map *at identity*. Must support an optional derivative.
    *   `gtsam::MatrixD AdjointMap() const;`: Computes the Adjoint map.
    *   `ChartAtOrigin` struct: This nested struct implements a first-order `retract` and `local`. You must define:
        *   `static MyGroup Retract(const gtsam::VectorD& v, ChartJacobian H = {});`
        *   `static gtsam::VectorD Local(const MyGroup& p, ChartJacobian H = {});`

6.  **`using` Declaration for `inverse`**:
    *   `using LieGroup<MyGroup, D>::inverse;`: This makes the base class `inverse(ChartJacobian H)` method visible.

7.  **Utilities (`Testable` concept)**:
    *   `void print(const std::string& s) const;`
    *   `bool equals(const MyGroup& other, double tol) const;`

#### Handling Dynamically-Sized Lie Groups

If your Lie group can change size at runtime, you must make two corresponding changes:

1.  Set the static dimension to `Eigen::Dynamic` in the `LieGroup` template parameter:
    *   `class MyGroup : public gtsam::LieGroup<MyGroup, Eigen::Dynamic> { ... };`
2.  You **must** also provide an instance method that returns the object's runtime dimension:
    *   `int dim() const;`

The `LieGroup` framework relies on this method to correctly size Jacobians and other matrices at runtime.

### 3. What You Get for Free (via CRTP)

By inheriting from `gtsam::LieGroup` and defining the primitives above, your class automatically receives correct implementations for all of the following methods, including their Jacobian versions. **Do NOT implement these yourself**, unless you are *sure* you are able to more efficiently (and correctly) implement the Jacobians.

*   `compose(const MyGroup& other)`: Composes `*this` with `other`.
*   `between(const MyGroup& other)`: Calculates the relative transformation from `*this` to `other`.
*   `retract(const VectorD& v)`: Implemented as `compose(ChartAtOrigin::Retract(v))`.
*   `localCoordinates(const MyGroup& other)`: Implemented as `ChartAtOrigin::Local(between(other))`.
*   `expmap(const VectorD& v)`: Implemented as `compose(Expmap(v))`.
*   `logmap(const MyGroup& other)`: Implemented as `Logmap(between(other))`.

### 4. Traits and Concept Checking

The `LieGroup` concept builds on `Manifold`. You do not need to specialize the traits struct yourself, as the `LieGroup` base class handles it.

To verify your implementation, use the `GTSAM_CONCEPT_LIE_INST` macro in your unit test file.

```cpp
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Lie.h>

// Your class include
#include <gtsam/geometry/MyGroup.h>

// This macro will fail to compile if MyGroup is not a valid LieGroup.
GTSAM_CONCEPT_LIE_INST(MyGroup)

// ... your unit tests ...
```