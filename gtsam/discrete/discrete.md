# Discrete Inference in GTSAM

The discrete module in GTSAM provides a comprehensive framework for probabilistic inference over discrete random variables. This module implements Bayesian networks, factor graphs, and efficient inference algorithms using decision trees and elimination methods.

## Overview

The discrete inference workflow in GTSAM typically follows this pattern:

1. **Model Definition**: Start with Bayesian networks using `DiscreteBayesNet` and `DiscreteConditional`
2. **Likelihood Integration**: Transform to factor graphs with `DiscreteFactorGraph` and various factor types
3. **Inference**: Eliminate variables to obtain `DiscreteBayesTree` for efficient marginalization
4. **Specialized Operations**: Use advanced classes for specific inference tasks

## Bayesian Networks for Modeling

### DiscreteBayesNet
The foundational class for representing Bayesian networks over discrete variables. A Bayes net defines a joint probability distribution as a product of conditional probability distributions.

```cpp
DiscreteBayesNet bayesNet;
bayesNet.add(conditional1);
bayesNet.add(conditional2);
```

**Notebook**: [DiscreteBayesNet.ipynb](doc/DiscreteBayesNet.ipynb)

### DiscreteConditional
Represents conditional probability tables P(X|Y1, Y2, ...) in the Bayesian network. These form the building blocks of Bayes nets and encode the conditional dependencies between variables.

```cpp
DiscreteConditional conditional(key, parents, signature);
```

**Notebook**: [DiscreteConditional.ipynb](doc/DiscreteConditional.ipynb)

### DiscreteDistribution
Represents marginal probability distributions P(X) for discrete variables. Used for priors.

**Notebook**: [DiscreteDistribution.ipynb](doc/DiscreteDistribution.ipynb)

### Signature
Handy class to define the conditional probability structure for discrete variables, specifying parent-child relationships and cardinalities.

**Notebook**: [Signature.ipynb](doc/Signature.ipynb)

## Discrete Factor Graphs

### DiscreteFactorGraph
The central class for factor graph representation. Bayes nets can be transformed into factor graphs by converting conditionals to factors, allowing for the integration of likelihood factors from observations.

```cpp
DiscreteFactorGraph graph = bayesNet.toFactorGraph();
graph.add(likelihoodFactor);
```

**Notebook**: [DiscreteFactorGraph.ipynb](doc/DiscreteFactorGraph.ipynb)

### DiscreteFactor
Abstract base class for all discrete factors. Factors represent arbitrary functions over discrete variables and can encode both prior knowledge and likelihood information.

### DecisionTreeFactor
A concrete implementation of discrete factors using decision trees for efficient storage and computation. Particularly effective for factors with conditional independence structure.

**Notebook**: [DecisionTreeFactor.ipynb](doc/DecisionTreeFactor.ipynb)

### TableFactor
Represents factors as explicit probability tables. Useful for small factors or when the full joint distribution needs to be stored explicitly.

## Inference: Bayes Trees

### DiscreteBayesTree
The result of variable elimination on a factor graph. Bayes trees enable efficient exact inference through their clique tree structure, supporting fast marginalization and conditioning operations.

```cpp
DiscreteBayesTree bayesTree = graph.eliminateMultifrontal(ordering);
DiscreteValues result = bayesTree.optimize();
```

### DiscreteEliminationTree
Intermediate structure used during the elimination process. Represents the computational tree for variable elimination before conversion to a Bayes tree.

### DiscreteJunctionTree
Alternative tree structure for inference, particularly useful for repeated marginal queries over the same model.

## Specialized and Exotic Classes

### DiscreteMarginals
Provides efficient computation of marginal probabilities for all variables in a factor graph or Bayes tree. Essential for uncertainty quantification.

### DiscreteLookupDAG
Specialized data structure for fast lookup operations in discrete probabilistic models. Optimized for scenarios with repeated queries.

### DiscreteSearch
Implements search algorithms over discrete probability spaces, including optimization and sampling methods.

### AlgebraicDecisionTree
Implements decision trees with algebraic operations (addition, multiplication) for efficient factor operations. The computational backbone for many discrete operations.

### Assignment and DiscreteValues
Core data structures for representing variable assignments and probability values in discrete models.

### DiscreteKey
Represents discrete random variables with their cardinalities, fundamental for defining the variable space.

## Decision Trees and Computational Primitives

### DecisionTree
Template-based decision tree implementation supporting arbitrary value types. Provides the foundation for efficient discrete computations.

### Ring
Mathematical abstraction for algebraic operations on decision tree values, enabling generic algorithms over different semirings.

## Parser and Utilities

### SignatureParser
Parses string representations of conditional probability signatures, enabling convenient model specification from text.

### TableDistribution
Utility class for working with probability distributions represented as tables.

## Example Workflows

### Basic Bayesian Network
1. Define variables with `DiscreteKey`
2. Create conditionals with `DiscreteConditional`
3. Build network with `DiscreteBayesNet`
4. Query marginals or sample

### Factor Graph Inference
1. Start with `DiscreteBayesNet` or build `DiscreteFactorGraph` directly
2. Add likelihood factors using `DecisionTreeFactor` or `TableFactor`
3. Eliminate variables to get `DiscreteBayesTree`
4. Query results using `DiscreteMarginals`

### Advanced Operations
1. Use `AlgebraicDecisionTree` for custom factor operations
2. Employ `DiscreteLookupDAG` for repeated queries
3. Apply `DiscreteSearch` for optimization problems
