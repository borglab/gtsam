# Guide to Creating a `Group`

This guide explains the minimal requirements for creating a class that satisfies the `Group` concept in GTSAM. The `Group` is a foundational algebraic concept that precedes the geometric `Manifold` concept. It defines types that have a binary operation (composition), an identity element, and an inverse for every element.

Like other GTSAM concepts, this is achieved through **traits-based metaprogramming**, not class inheritance, see [GTSAM-Concepts](GTSAM-Concepts.md).


### 1. Group Operations

At its heart, a group is defined by four abstract operations: `Identity`, `Compose`, `Between`, and `Inverse`. GTSAM distinguishes between two "flavors" of groups based on the C++ syntax used to implement these operations.

### 2. Implementation Requirements: Choose a Flavor

You must implement the methods for one of the two flavors.

#### A) `MultiplicativeGroup` Flavor

This is for groups that use `operator*` for composition, like rotation or pose types. Your class must provide:

*   A static `Identity` method.
*   An `operator*` for composition.
*   An `inverse` method.

#### B) `AdditiveGroup` Flavor

This is for groups that behave like vector spaces, using `operator+` for composition. Your class must provide:

*   A static `Identity` method.
*   An `operator+` for composition.
*   A binary `operator-` for the `Between` operation.
*   A unary `operator-` for the `Inverse` operation.

### 3. Prerequisite: The `Testable` Concept

The `Group` concept requires the `Testable` concept. Before satisfying the group requirements, your class must also implement:

*   A `print` method.
*   An `equals` method.

### 4. Linking Your Class to GTSAM (The `traits` Specialization)

This is the crucial step that registers your class with GTSAM. You must specialize the `gtsam::traits` struct for your class. This specialization should inherit from one of the helpers provided in `Group.h`. The helper automatically maps your chosen C++ operators to the abstract `Compose`, `Between`, and `Inverse` operations.

To do this, you will add a specialization of `traits<MyGroup>` in the `gtsam` namespace. If your group is multiplicative, this specialization will inherit from `internal::MultiplicativeGroup<MyGroup>`. If your group is additive, it will inherit from `internal::AdditiveGroup<MyGroup>`.

### 5. Concept Checking

To verify that your implementation is correct, add the `GTSAM_CONCEPT_GROUP_INST` macro, followed by your class name in parentheses, to your unit test file. This will cause a compile-time error if any of the requirements for a `Group` (including `Testable`) are not met.