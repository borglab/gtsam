# Linear

The `linear` module in GTSAM provides the core classes and algorithms for working with linear (Gaussian) factor graphs, which are central to efficient inference in many estimation and SLAM problems. This module defines the structures and solvers for Gaussian graphical models, and forms the backbone for linearization of nonlinear problems.

## Core Concepts

-   **GaussianFactor**: Abstract base class for all linear (Gaussian) factors. Represents a factor with a quadratic error function. All linearized nonlinear factors are represented as GaussianFactors.
-   **JacobianFactor**: Derived from GaussianFactor, represents a factor in terms of a Jacobian matrix and a right-hand side vector. Typically results from linearizing a nonlinear factor.
-   **HessianFactor**: Derived from GaussianFactor, represents a factor in terms of a Hessian matrix (information matrix) and a gradient vector. Useful for direct construction from second-order information.
-   **GaussianFactorGraph**: A collection of GaussianFactors, representing the full linearized system. Supports construction, manipulation, and elimination.
-   **GaussianConditional**: Represents a conditional Gaussian distribution, typically the result of eliminating variables from a GaussianFactorGraph.
-   **GaussianBayesNet**: Directed acyclic graph (DAG) of GaussianConditionals, representing the result of sequential variable elimination.
-   **GaussianBayesTree**: Tree structure of cliques, each containing a GaussianConditional, resulting from multifrontal elimination.
-   **VectorValues**: Efficient container for storing and manipulating solution vectors indexed by variable keys.
-   **NoiseModel**: Encapsulates the noise characteristics (covariance, robust loss, etc.) associated with each factor.

## Linear Solvers & Algorithms

-   **ConjugateGradientSolver**: Iterative solver for large, sparse linear systems using the conjugate gradient method.
-   **PCGSolver**: Preconditioned conjugate gradient solver for improved convergence.
-   **IterativeSolver**: Base class for iterative solvers.
-   **SubgraphPreconditioner / SubgraphSolver**: Methods for constructing and solving preconditioned systems using subgraph techniques.
-   **KalmanFilter**: Implements the standard and square-root Kalman filter for linear dynamical systems.
-   **Sampler**: Utility for sampling from Gaussian distributions defined by linear systems.

## Linearization in Nonlinear Inference

-   **Role of GaussianFactor and Derived Classes**: In nonlinear optimization (see `nonlinear` module), nonlinear factors are linearized at each iteration, resulting in JacobianFactor or HessianFactor instances. These are then assembled into a GaussianFactorGraph, which is solved using the algorithms in this module.

## Visualization & Utilities

-   **DotWriter**: (Shared with `inference`) Helper for generating Graphviz `.dot` files for visualizing factor graphs and trees.
-   **LossFunctions**: Implements robust loss functions for handling outliers in linear systems.
-   **Scatter**: Utility for mapping variable indices to vector blocks.

## More for internal use

-   **GaussianEliminationTree**: Tree structure representing the order and dependencies during sequential elimination.
-   **GaussianJunctionTree**: Cluster tree used in multifrontal elimination, holding factors before elimination into conditionals.
-   **AcceleratedPowerMethod / PowerMethod**: Algorithms for estimating the largest eigenvalue of a matrix, useful for preconditioning and analysis.

## Testing & Examples

-   The `tests/` directory contains unit tests for all major classes and algorithms.
-   Example usage can be found in the `examples/` directory at the root of the repository.

