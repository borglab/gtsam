# Base

The base module provides fundamental data structures, utilities, and mathematical operations that form the foundation of GTSAM. Here are the key header files:

## Core Mathematical Concepts
Utilities to define group, manifold, and Lie group classes, as well as concept checks for them.
- [Manifold.h](Manifold.h) - Defines the concept for differentiable manifolds with retract/local operations. See [Manifold](doc/Manifold.md) docs.
- [Lie.h](Lie.h) - Implements the Lie group concept combining group operations with manifold structure. See [LieGroup](doc/LieGroup.md) docs.
- [MatrixLieGroup.h](MatrixLieGroup.h) - Base class for matrix Lie groups (SO(n), SE(n), SL(n), etc.) with Hat/Vee operators. See [MatrixLieGroup](doc/MatrixLieGroup.md) docs.

And, less common:
- [Group.h](Group.h) - Defines the group concept with identity, inverse, and composition operations. See [Group](doc/Group.md) docs.
- [VectorSpace.h](VectorSpace.h) - Defines the vector space concept. See [VectorSpace](doc/VectorSpace.md) docs.
- [ProductLieGroup.h](ProductLieGroup.h) - Cartesian products of Lie groups
- [lieProxies.h](lieProxies.h) - Proxy functions for testing

## Linear Algebra
Linear algebra operations optimized for robotics applications. `Vector` and `Matrix` are essentially typedefs and wrappers for Eigen:
- [Vector.h](Vector.h) - Vector operations and mathematical functions
- [Matrix.h](Matrix.h) - Essential matrix operations and linear algebra utilities built on Eigen
- [cholesky.h](cholesky.h) - Cholesky decomposition algorithms for solving linear systems

These are used in Jacobian and Hessian factors:
- [VerticalBlockMatrix.h](VerticalBlockMatrix.h) - Specialized block matrix structure for efficient sparse operations
- [SymmetricBlockMatrix.h](SymmetricBlockMatrix.h) - Symmetric block matrix implementation for optimization problems

## Jacobians
- [numericalDerivative.h](numericalDerivative.h) - Numerical differentiation utilities for automatic derivative computation
- [OptionalJacobian.h](OptionalJacobian.h) - Optional Jacobian computation wrapper for optimization efficiency

## Container and Data Structures
- High-performance container types (FastSet, FastMap, etc.) optimized for GTSAM (unclear whether still relevant for recent c++ compilers)
- [DSFMap.h](DSFMap.h) - Disjoint Set Forest implementation using maps for union-find operations
- [DSFVector.h](DSFVector.h) - Disjoint Set Forest implementation using vectors for better performance
- [TreeTraversal.h](TreeTraversal.h) - Tree traversal algorithms and utilities for factor graphs
- [ConcurrentMap.h](ConcurrentMap.h) - Thread-safe concurrent map implementation

## Debugging and Development Tools
- [debug.h](debug.h) - Debugging utilities, assertion macros, and conditional compilation flags
- [timing.h](timing.h) - Performance timing and profiling utilities for benchmarking
- [TestableAssertions.h](TestableAssertions.h) - Assertion macros for unit testing with tolerance checking
- [chartTesting.h](chartTesting.h) - Utilities for testing manifold chart operations

## Sampling and Statistics
- [WeightedSampler.h](WeightedSampler.h) - Weighted random sampling utilities for Monte Carlo methods
- [sampler.h](sampler.h) - General sampling utilities and random number generation

## Graph Algorithms
- [Kruskal.h](Kruskal.h) - Kruskal's algorithm for minimum spanning trees in factor graphs
- [BTree.h](BTree.h) - Balanced tree implementation for efficient storage and retrieval

## Type System and Traits
- [types.h](types.h) - Common type definitions, aliases, and forward declarations
- [concepts.h](concepts.h) - C++ concept definitions for type checking and constraints
- [Testable.h](Testable.h) - Base class and concept for objects that can be tested for equality

## Serialization Support
- [SerializationBase.h](SerializationBase.h) - Base serialization functionality for saving/loading objects
- [StdOptionalSerialization.h](StdOptionalSerialization.h) - Serialization support for std::optional types

## Memory Management and Performance
- [utilities.h](utilities.h) - General utility functions, memory management helpers, and common operations
- [FastDefaultAllocator.h](FastDefaultAllocator.h) - Custom allocator for improved performance in critical paths
- [Value.h](Value.h) - Type-erased value wrapper for heterogeneous containers

## Template Metaprogramming
- [treeTraversal-inst.h](treeTraversal-inst.h) - Template instantiations for tree traversal algorithms
- [concepts.h](concepts.h) - SFINAE utilities and type trait helpers

