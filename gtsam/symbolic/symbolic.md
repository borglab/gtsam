# Symbolic

The `symbolic` module in GTSAM deals with the *structure* of factor graphs and Bayesian networks, independent of the specific numerical types of factors (like Gaussian or discrete). It allows for analyzing graph connectivity, determining optimal variable elimination orders, and understanding the sparsity structure of the resulting inference objects.

This is crucial for efficient inference, as the symbolic elimination steps determine the computational complexity and memory requirements of the numerical factorization.

The classes here are used primarily to *illustrate* symbolic elimination. Internally, GTSAM does analyze the structure of other factor graph types without explicitly converting to a symbolic factor graph.
 

## Classes

Here's an overview of the key classes in the `symbolic` module:

### Factor Graph

-   **[SymbolicFactor](doc/SymbolicFactor.ipynb)**: Represents the connectivity between a set of variables (keys) in a factor graph, without any specific numerical function associated with it. It defines the hyperedges of the graph.
-   **[SymbolicFactorGraph](doc/SymbolicFactorGraph.ipynb)**: A collection of `SymbolicFactor` objects, representing the overall structure of a factor graph.

### Elimination Products

These classes represent the results of symbolic variable elimination:

-   **[SymbolicConditional](doc/SymbolicConditional.ipynb)**: Represents the structure of a conditional probability distribution P(Frontals | Parents). It stores the keys of the frontal (conditioned) and parent variables resulting from eliminating one or more variables from a set of factors.
-   **[SymbolicBayesNet](doc/SymbolicBayesNet.ipynb)**: A directed acyclic graph composed of `SymbolicConditional` objects, representing the structure of a factorized distribution P(X) = Π P(Xi | Parents(Xi)). Typically results from sequential elimination.
-   **[SymbolicBayesTree](doc/SymbolicBayesTree.ipynb)**: A tree structure where each node (`SymbolicBayesTreeClique`) represents a `SymbolicConditional` P(Frontals | Separator). This is the result of multifrontal elimination and is the underlying structure for efficient incremental updates (iSAM) and exact marginal computation.
-   **[SymbolicBayesTreeClique](doc/SymbolicBayesTreeClique.ipynb)**: Represents a single clique (node) within a `SymbolicBayesTree`, containing a `SymbolicConditional`.

### Elimination Structures

These classes represent intermediate structures used during the elimination process:

-   **[SymbolicEliminationTree](doc/SymbolicEliminationTree.ipynb)**: Represents the dependency structure of sequential variable elimination. Each node corresponds to eliminating a single variable.
-   **[SymbolicJunctionTree](doc/SymbolicJunctionTree.ipynb)** (Clique Tree): An intermediate structure used in multifrontal elimination, derived from an elimination tree. Each node (`SymbolicCluster`) represents a clique of variables eliminated together and stores the *factors* associated with that clique.

**Importance**
Performing symbolic analysis can be used for:
1. Choosing an Optimal Ordering: Finding an ordering that minimizes fill-in (new connections created during elimination) is key to efficient numerical factorization.
2. Predicting Computational Cost: The structure of the Bayes net or Bayes tree determines the complexity of subsequent numerical operations like solving or marginalization.
3. Memory Allocation: Knowing the structure allows for pre-allocation of memory for the numerical factorization.
The symbolic module provides the foundation for GTSAM's efficient inference algorithms.
