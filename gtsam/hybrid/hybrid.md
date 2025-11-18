# Hybrid

The `hybrid` module provides classes and algorithms designed for inference in probabilistic graphical models involving **both discrete and continuous variables**. It extends the core concepts from the `inference` module to handle these mixed variable types, enabling the modeling and solving of systems with discrete modes, decisions, or states alongside continuous states.

## Core Hybrid Concepts

-   [HybridValues](doc/HybridValues.ipynb): Container holding assignments for discrete (`DiscreteValues`), continuous (`VectorValues`), and nonlinear (`Values`) variables simultaneously.
-   [HybridFactor](doc/HybridFactor.ipynb): Abstract base class for factors involving discrete and/or continuous variables.
-   [HybridConditional](doc/HybridConditional.ipynb): Type-erased wrapper for conditionals (Gaussian, Discrete, HybridGaussian) resulting from hybrid elimination.

## Hybrid Factor Types

-   [HybridNonlinearFactor](doc/HybridNonlinearFactor.ipynb): Represents a factor where a discrete choice selects among different `NonlinearFactor` (NoiseModelFactor) components, potentially with associated scalar energies.
-   [HybridGaussianFactor](doc/HybridGaussianFactor.ipynb): Represents a factor where a discrete choice selects among different Gaussian factor components, potentially with associated scalar energies.
-   [HybridGaussianProductFactor](doc/HybridGaussianProductFactor.ipynb): Internal decision tree structure holding pairs of `GaussianFactorGraph` and `double` scalars, used during hybrid elimination.

## Hybrid Factor Graphs

-   [HybridFactorGraph](doc/HybridFactorGraph.ipynb): Base class for factor graphs designed to hold hybrid factors.
-   [HybridNonlinearFactorGraph](doc/HybridNonlinearFactorGraph.ipynb): A factor graph containing Nonlinear, Discrete, and/or `HybridNonlinearFactor` types, used to model nonlinear hybrid systems.
-   [HybridGaussianFactorGraph](doc/HybridGaussianFactorGraph.ipynb): A factor graph containing Gaussian, Discrete, and/or `HybridGaussianFactor` types, typically resulting from linearization. Supports hybrid elimination.

## Hybrid Bayes Nets and Bayes Trees

-   [HybridGaussianConditional](doc/HybridGaussianConditional.ipynb): Represents a conditional density $P(X | M, Z)$ where continuous variables $X$ depend on discrete parents $M$ and continuous parents $Z$, implemented as a decision tree of `GaussianConditional`s.
-   [HybridBayesNet](doc/HybridBayesNet.ipynb): Represents the result of sequential variable elimination on a `HybridGaussianFactorGraph` as a DAG of `HybridConditional`s.
-   [HybridBayesTree](doc/HybridBayesTree.ipynb): Represents the result of multifrontal variable elimination on a `HybridGaussianFactorGraph` as a tree of cliques, each containing a `HybridConditional`.

## Incremental Hybrid Inference

-   [HybridGaussianISAM](doc/HybridGaussianISAM.ipynb): Incremental Smoothing and Mapping (ISAM) algorithm for `HybridGaussianFactorGraph`s, based on updating a `HybridBayesTree`.
-   [HybridNonlinearISAM](doc/HybridNonlinearISAM.ipynb): Wrapper providing an ISAM interface for nonlinear hybrid problems, managing linearization and an underlying `HybridGaussianISAM`.
-   [HybridSmoother](doc/HybridSmoother.ipynb): An incremental fixed-lag smoother interface for hybrid systems, managing updates to a `HybridBayesNet` posterior.

## Hybrid Elimination Intermediates

-   [HybridEliminationTree](doc/HybridEliminationTree.ipynb): Tree structure representing the dependencies during sequential elimination of a `HybridGaussianFactorGraph`.
-   [HybridJunctionTree](doc/HybridJunctionTree.ipynb): Intermediate cluster tree structure used in multifrontal elimination, holding original factors within cliques before elimination.
