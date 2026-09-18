# Linear

The `linear` module represents and solves Gaussian factor graphs. Nonlinear
optimizers use these classes after linearizing a nonlinear factor graph around
the current estimate.

## Gaussian Factors and Graphs

- [GaussianFactor](GaussianFactor.h) is the common interface for linear
  Gaussian factors.
- [JacobianFactor](JacobianFactor.h) represents a whitened least-squares term
  in Jacobian form, $\frac{1}{2}\lVert A x-b\rVert^2$.
- [HessianFactor](HessianFactor.h) represents the equivalent quadratic term in
  information form.
- [GaussianFactorGraph](GaussianFactorGraph.h) stores a collection of Gaussian
  factors and provides elimination, optimization, gradient, Jacobian, and
  Hessian operations.
- [GaussianConditional](GaussianConditional.h) represents a conditional
  produced by eliminating one or more frontal variables.
- [VectorValues](VectorValues.h) stores vector-valued assignments such as a
  linear solution or nonlinear tangent-space update.

## Direct Solvers and Elimination

- [MultifrontalSolver](doc/MultifrontalSolver.ipynb) explains the reusable,
  imperative multifrontal solver, its lifecycle, supported factors, packed
  numerical storage, partial-elimination export, and every tuning option with
  its default.
- [GaussianBayesNet](GaussianBayesNet.h) and
  [GaussianBayesTree](GaussianBayesTree.h) store the results of sequential and
  multifrontal elimination. Their graph structure is introduced in the
  [inference module](../inference/inference.md).
- [GaussianEliminationTree](GaussianEliminationTree.h) and
  [GaussianJunctionTree](GaussianJunctionTree.h) organize elimination work from
  an ordering.

## Iterative Solvers

- [ConjugateGradientSolver](ConjugateGradientSolver.h) implements conjugate
  gradients for linear systems.
- [PCGSolver](PCGSolver.h) implements preconditioned conjugate gradients.
- [Preconditioner](Preconditioner.h) is the interface for iterative-solver
  preconditioners.

## Diagnostics

- [Debugging an Indeterminate Linear System](doc/IndeterminateSystemException.ipynb)
  shows how to preserve a failing graph, inspect its Jacobian rank and null
  space, and repair missing gauge constraints.
