# Creating a `MatrixLieGroup`

This guide explains how to create a `MatrixLieGroup`, a specific type of `LieGroup` that has a matrix representation. **Please read [Manifold.md](Manifold.md) and [LieGroup.md](LieGroup.md) first.** This document focuses only on the additional requirements for a matrix Lie group.

A `MatrixLieGroup` has all the properties of a `LieGroup`, but its structure is tied to an underlying `N x N` matrix. This allows for additional operations specific to matrix algebra.

### 1. From Lie Group to Matrix Lie Group

To implement a `MatrixLieGroup`, you inherit from `gtsam::MatrixLieGroup<MyGroup, D, N>`, which in turn inherits from `gtsam::LieGroup`. This inheritance provides even more functionality for free.

### 2. Minimal Additional Requirements

You must implement everything from the `LieGroup.md` guide, plus the following:

#### `MyGroup.h` - Header File Additions

1.  **Class Definition and Inheritance**:
    *   Change your inheritance to `public gtsam::MatrixLieGroup<MyGroup, D, N>`, where `N` is the side-length of your matrix.

2.  **Lie Algebra Typedef**:
    *   `using LieAlgebra = gtsam::MatrixN;`: Defines the type of a Lie algebra element (which is an `N x N` matrix).

3.  **Matrix Lie Group Primitives**:
    *   `const gtsam::MatrixN& matrix() const;`: An accessor for the underlying matrix data member.
    *   `static gtsam::MatrixN Hat(const gtsam::VectorD& xi);`: Maps a `D`-dimensional tangent vector to its `N x N` matrix representation in the Lie algebra.
    *   `static gtsam::VectorD Vee(const gtsam::MatrixN& X);`: The inverse of `Hat`, mapping an `N x N` Lie algebra matrix back to a `D`-dimensional vector.

### 3. What You Get for Free (Additionally)

By inheriting from `gtsam::MatrixLieGroup`, you get:

*   **A default `AdjointMap()` implementation**: This generic version works by repeatedly calling your `Hat` and `Vee` methods. While it is correct, it is often slow. For performance-critical applications, you should still provide a faster, closed-form `AdjointMap()` implementation in your class, which will override the default.
*   **A `vec()` method**: This vectorizes the `N x N` matrix representation of your group element into an `(N*N) x 1` vector.

### 4. Traits and Concept Checking

Finally, the traits specialization in your header file must be updated to reflect that your class is now a `MatrixLieGroup`.

1.  **GTSAM Traits Specialization**:
    *   This is the final, crucial step. Specialize the `traits` struct using the `internal::MatrixLieGroup` helper.
    ```cpp
    namespace gtsam {
    template <>
    struct traits<MyGroup> : public internal::MatrixLieGroup<MyGroup, N> {};

    template <>
    struct traits<const MyGroup> : public internal::MatrixLieGroup<MyGroup, N> {};
    }
    ```

2.  **Concept Checking**:
    *   Update the macro in your unit test file to check the `MatrixLieGroup` concept.
    ```cpp
    #include <gtsam/base/TestableAssertions.h>
    #include <gtsam/base/MatrixLieGroup.h>

    // Your class include
    #include <gtsam/geometry/MyGroup.h>

    // This macro will fail to compile if MyGroup is not a valid MatrixLieGroup.
    GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(MyGroup)

    // ... your unit tests ...
    ```
This modular approach ensures that your class provides all the necessary components for full integration into the GTSAM framework.
