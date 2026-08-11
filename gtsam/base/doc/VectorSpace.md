# Creating a `VectorSpace`

This guide explains the requirements for creating a class that satisfies the `VectorSpace` concept. In GTSAM, a `VectorSpace` is a specialized `AdditiveGroup` that adds the notion of scalar multiplication and a vector norm. This is the concept that should be satisfied by types that behave like mathematical vectors.

**Prerequisite:** Please read [Group.md](Group.md), as a `VectorSpace` must first fulfill all the requirements of an `AdditiveGroup`.

---

### 1. The Core Idea: Scalable Vectors

A `VectorSpace` builds directly upon the `AdditiveGroup` concept. It represents objects that can be added and subtracted, but with one crucial addition: they can be **scaled** by a scalar value (a `double`). This allows for operations like `0.5 * v`, which are fundamental in optimization and linear algebra.

---

### 2. Minimal Implementation Requirements

To be a `VectorSpace`, your class must first implement everything required by `AdditiveGroup` and `Testable`, plus a few new methods.

#### A `AdditiveGroup` Requirements (A Brief Recap)

Your class must provide:
*   A static `Identity` method (returning the zero vector).
*   An `operator+` for composition (vector addition).
*   A binary `operator-` for `Between` (vector subtraction).
*   A unary `operator-` for `Inverse` (vector negation).

#### B New `VectorSpace` Requirements

In addition to the group operations, your class must provide:

*   **Scalar Multiplication**: An `operator*` that takes a `double` as the *first* argument and your class type as the second.
*   **Dimension**: An instance method `dim()` that returns the dimension of the vector as an `int` or `size_t`. This is required for both fixed-size and dynamically-sized vector spaces.
*   **Dot Product**: An instance method `dot` that computes the dot product with another instance of the class.
*   **Norm**: An instance method `norm` that computes the L2 norm of the vector.

#### C `Testable` Prerequisite

As with all concepts, you must also provide:
*   A `print` method.
*   An `equals` method.

---

### 3. Linking Your Class to GTSAM (The `traits` Specialization)

To register your class as a `VectorSpace`, you specialize the `gtsam::traits` struct for your class. This specialization must inherit from the `internal::VectorSpace<MyVectorSpace>` helper. This helper bundles the `AdditiveGroup` and `Testable` traits and adds the `vector_space_tag`.

You will add a specialization of `traits<MyVectorSpace>` in the `gtsam` namespace that inherits from `internal::VectorSpace<MyVectorSpace>`.

---

### 4. Built-in Support for Common Types

A key feature of `VectorSpace.h` is that it already provides complete `VectorSpace` implementations for some common C++ and Eigen types. **You do not need to write a custom class or traits specialization if you are using any of the following**:

*   **Primitive Scalar Types**: `double` and `float` are automatically treated as 1D vector spaces.
*   **Fixed-Size Eigen Matrices and Vectors**: All fixed-size Eigen types, such as `gtsam::Vector3` (`Eigen::Vector3d`), `gtsam::Matrix2` (`Eigen::Matrix2d`), etc., are treated as vector spaces where the dimension is the total number of elements (e.g., a 2x3 matrix has dimension 6).
*   **Dynamically-Sized Eigen Matrices and Vectors**: All dynamically-sized Eigen types, including `gtsam::Vector` (`Eigen::VectorXd`) and `Eigen::MatrixXd`, are fully supported.

This built-in support means you can use Eigen types directly in GTSAM algorithms that require a `VectorSpace` without any extra setup.

---

### 5. Concept Checking

To verify that your custom implementation is correct, add the `GTSAM_CONCEPT_VECTOR_SPACE_INST` macro to your unit test file. This will cause a compile-time error if any of the requirements for a `VectorSpace` (including `AdditiveGroup` and `Testable`) are not met.