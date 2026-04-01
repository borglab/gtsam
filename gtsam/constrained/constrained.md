# Constrained

The `constrained` module provides nonlinear constrained optimization tools on top of GTSAM factor graphs. It separates an optimization problem into a cost term, equality constraints, and inequality constraints, and then solves the resulting problem with penalty or augmented-Lagrangian style outer loops.

## Core Problem Representation

- [ConstrainedOptProblem](ConstrainedOptProblem.h): Container for a constrained problem of the form `min 0.5 ||f(X)||^2` subject to `h(X) = 0` and `g(X) <= 0`.
- [NonlinearConstraint](NonlinearConstraint.h): Base class for nonlinear constraints represented as `NoiseModelFactor`s with constrained noise models.
- [NonlinearEqualityConstraint](NonlinearEqualityConstraint.h): Base class and concrete expression-based equality constraints.
- [NonlinearInequalityConstraint](NonlinearInequalityConstraint.h): Base class and concrete scalar expression inequality constraints.
- [InequalityPenaltyFunction](InequalityPenaltyFunction.h): Ramp and smooth ramp variants used to penalize inequality violations.

## Optimizer Hierarchy

- [ConstrainedOptimizer](ConstrainedOptimizer.h): Base class for constrained optimizers, including shared convergence logic and iteration state.
- [PenaltyOptimizer](PenaltyOptimizer.h): Classical penalty method. Each outer iteration increases penalty parameters and solves an unconstrained nonlinear least-squares subproblem.
- [AugmentedLagrangianOptimizer](AugmentedLagrangianOptimizer.h): Generic augmented Lagrangian method with explicit multiplier updates for equality and inequality constraints.
- [BoundConstrainedLagrangian](BoundConstrainedLagrangian.h): A more specialized bound-constrained augmented Lagrangian variant with `eta` and `omega` thresholds controlling multiplier and penalty updates.

## Supporting Pieces

- [AugmentedLagrangianFunction](AugmentedLagrangian.h): Builds the augmented Lagrangian as a `NonlinearFactorGraph`.
- [ConstrainedOptProblem::auxiliaryProblem](ConstrainedOptProblem.h): Converts scalar inequality constraints into equality constraints with auxiliary variables when that formulation is useful.
- [BoundConstrainedLagrangian algorithm note](doc/BoundConstrainedLagrangian.md): Nocedal/Conn background for the BCL-style augmented-Lagrangian variant, its mapping to the current code, and a short note on the Carpentier implementations.

## Current Algorithms

### Penalty Method

`PenaltyOptimizer` forms a merit function

`f(x) + 0.5 * muEq * ||h(x)||^2 + 0.5 * muIneq * ||g(x)_+||^2`

and solves a sequence of unconstrained nonlinear least-squares problems. This is the simplest constrained solver in the module and is often the easiest baseline to reason about.

### Augmented Lagrangian Method

`AugmentedLagrangianOptimizer` adds explicit Lagrange multiplier updates on top of the penalty formulation. Equality multipliers are updated by dual ascent, and inequality multipliers are projected to remain nonnegative. The implementation builds the augmented Lagrangian directly as a factor graph, including biased factors and anti-factors for the inequality terms.

This is the main generic constrained optimizer currently in the module.

### Bound-Constrained Augmented Lagrangian

`BoundConstrainedLagrangian` is a more specialized outer-loop strategy. Instead of always updating multipliers, it checks:

- the infinity norm of the cost gradient against `omega`
- the infinity norm of the equality constraint violation against `eta`

If the iterate is sufficiently stationary and feasible, it updates multipliers and tightens thresholds; otherwise, it increases the penalty parameter. This solver currently focuses on the equality-constrained case and reuses the augmented-Lagrangian factor-graph construction.

This is the solver in the module that is most closely aligned, at a high level, with the bound-constrained / proximal augmented-Lagrangian ideas used in the `aligator` ecosystem. In this branch, that work should be attributed to Yetong, including the later `BoundConstrainedLagrangian` implementation.

## Class Structure Summary

The module is organized in roughly four layers:

1. Problem specification:
   `ConstrainedOptProblem`, `NonlinearConstraint`, `NonlinearEqualityConstraint`, `NonlinearInequalityConstraint`
2. Penalty primitives:
   `InequalityPenaltyFunction`, penalty-factor construction on constraints
3. Outer-loop optimizers:
   `PenaltyOptimizer`, `AugmentedLagrangianOptimizer`, `BoundConstrainedLagrangian`
4. Tests and examples:
   unit tests in `tests/` and small synthetic constrained examples in `tests/constrainedExample.h`

## Benchmarking Note

For the BCL globalization logic itself, the cleanest local reference is `~/git/proxsuite`, whose `ProxQP` implementation explicitly cites Conn and Nocedal and exposes a BCL update rule.

For an end-to-end nonlinear solver from the same ecosystem, the closest local reference is `~/git/aligator`, whose `SolverProxDDP` applies related BCL-style augmented-Lagrangian ideas inside a trajectory-optimization solver.

`~/git/crocoddyl` currently points more toward constrained DDP and SQP formulations than to this particular augmented-Lagrangian outer-loop design.
