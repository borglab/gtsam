# Inference

The `inference` module provides the foundational classes and algorithms for probabilistic graphical models in GTSAM, focusing on variable elimination and the resulting structures like Bayes Nets and Bayes Trees.

## Core Concepts

-   [Key](doc/Key.ipynb): Base type (`uint64_t`) for uniquely identifying variables.
-   [Symbol](doc/Symbol.ipynb): A Key type encoding a character and an index (e.g., `x0`).
-   [LabeledSymbol](doc/LabeledSymbol.ipynb): A `Symbol` variant with an additional label character, useful for multi-robot scenarios (e.g., `xA0`).
-   [EdgeKey](doc/EdgeKey.ipynb): A Key type encoding a pair of 32-bit integers.
-   [Factor](doc/Factor.ipynb): Abstract base class for all factors (relationships between variables).
-   [FactorGraph](doc/FactorGraph.ipynb): Base class representing a collection of factors.
-   [Conditional](doc/Conditional.ipynb): Abstract base class for conditional distributions/densities resulting from elimination ( $P(\text{Frontals} | \text{Parents})$ ).

## Elimination Algorithms & Control

-   [Ordering](doc/Ordering.ipynb): Specifies the order in which variables are eliminated, crucial for efficiency.
-   [VariableIndex](doc/VariableIndex.ipynb): Maps variables to the factors they appear in, used for efficient elimination ordering and construction.
-   [EliminateableFactorGraph](https://github.com/borglab/gtsam/blob/develop/gtsam/inference/EliminateableFactorGraph.h): A mixin class providing `eliminateSequential` and `eliminateMultifrontal` methods to concrete factor graph types (like `GaussianFactorGraph`, `SymbolicFactorGraph`).

## Elimination Results & Structures

-   [BayesNet](doc/BayesNet.ipynb): Represents the result of sequential variable elimination as a directed acyclic graph (DAG) of conditionals.
-   [EliminationTree](doc/EliminationTree.ipynb): Tree structure representing the dependencies and computations during sequential elimination.
-   [ClusterTree](doc/ClusterTree.ipynb): Base class for tree structures where nodes are clusters of factors (e.g., JunctionTree).
-   [JunctionTree](doc/JunctionTree.ipynb): A cluster tree representing the cliques formed during multifrontal elimination, holding the factors before they are eliminated into conditionals.
-   [BayesTreeCliqueBase](https://github.com/borglab/gtsam/blob/develop/gtsam/inference/BayesTreeCliqueBase.h): Abstract base class for the nodes (cliques) within a BayesTree.
-   [BayesTree](doc/BayesTree.ipynb): Represents the result of multifrontal variable elimination as a tree of cliques, where each clique contains the conditional $P(\text{Frontals} | \text{Separator})$.

## Incremental Inference

-   [ISAM](doc/ISAM.ipynb): Incremental Smoothing and Mapping algorithm based on updating a BayesTree (original version, often superseded by ISAM2 in `nonlinear`).

## Visualization

-   [DotWriter](doc/DotWriter.ipynb): Helper class to customize the generation of Graphviz `.dot` files for visualizing graphs and trees.