# Gaussian augmented Lagrangian solver

`AugmentedLagrangianSolver` solves a fixed convex quadratic program directly from Gaussian factors. It belongs in `gtsam/constrained`: its constraints and multiplier updates distinguish it from the Gaussian elimination routines in `gtsam/linear`. The augmented inner problem is piecewise quadratic because inequality rows enter and leave the PHR penalty. The solver does not linearize nonlinear functions.

Supply three `GaussianFactorGraph` objects: quadratic costs, equality rows `E*x=d`, and inequality rows `A*x<=b`. Costs retain their Jacobian or Hessian representation. Constraints must be Jacobian factors with unit or finite positive diagonal noise scales. The graph determines the constraint sense; a zero-sigma constrained noise model is rejected. Constraint scaling affects multiplier units: divide each returned multiplier by its original sigma to obtain a multiplier for the unscaled row.

```python
import numpy as np
import gtsam

# Minimize 0.5*||x-[2,-1]||^2, with x[0]+x[1]=1 and x[0]<=0.5.
costs = gtsam.GaussianFactorGraph()
costs.add(gtsam.HessianFactor(0, np.eye(2), np.array([2., -1.]), 0.))
equalities = gtsam.GaussianFactorGraph()
equalities.add(gtsam.JacobianFactor(
    0, np.array([[1., 1.]]), np.array([1.]),
    gtsam.noiseModel.Unit.Create(1)))
inequalities = gtsam.GaussianFactorGraph()
inequalities.add(gtsam.JacobianFactor(
    0, np.array([[1., 0.]]), np.array([0.5]),
    gtsam.noiseModel.Unit.Create(1)))
params = gtsam.AugmentedLagrangianParams()
params.absoluteViolationTolerance = 1e-8
params.absoluteStationarityTolerance = 1e-8
solver = gtsam.AugmentedLagrangianSolver(costs, equalities, inequalities, params)
state = solver.optimizeWithState()
assert state.converged
np.testing.assert_allclose(state.values.at(0), [0.5, 0.5], atol=1e-6)
```

`optimize()` returns `VectorValues`; `optimizeWithState()` also returns multipliers, convergence status and work counters. Both accept an optional finite `VectorValues` start, which may violate constraints. Without a start, zeros are inferred from all objective and constraint keys. Equality multipliers retain their input factor blocks; inequality blocks are scalarized in factor/row order and have nonnegative multipliers. Repeated calls start with fresh multipliers and caches. `storeOptProgress` enables history; otherwise no history is retained.

`QpProblem::optimize(..., QpSolverType::AugmentedLagrangian)` is a small convenience driver using the same Gaussian solver with native defaults. It unwraps Gaussian factors once and preserves supplied `Vector` and `Matrix` value shapes. Pass `AugmentedLagrangianParams` to `QpProblem::optimize` for explicit ALM options; use `AugmentedLagrangianSolver` directly when examining status and multipliers. Sparse active set remains the `QpProblem` default.

## QR selection for QP solves

The QP convenience overload accepts the same ALM parameters as the direct solver. This exposes the existing multifrontal QR setting without adding another flag or changing its default:

```python
params = gtsam.AugmentedLagrangianParams()
params.lmParams.multifrontalParams.qrMode = gtsam.MultifrontalParameters.QRMode.Off
problem = gtsam.QpProblem()
problem.addCost(gtsam.HessianFactor(0, np.eye(2), np.array([2., -1.]), 0.))
result = problem.optimize(params)  # Or problem.optimize(initial_values, params).
np.testing.assert_allclose(result.atVector(0), [2., -1.], atol=1e-6)
```

`Off` disables leaf QR, `Allow` retains automatic selection (the default), and `Force` requests QR on eligible leaves. This option controls the ALM multifrontal workspace. The same parameters also configure `AugmentedLagrangianOptimizer`. It changes numerical factorization, while preserving penalties, damping, objective and convergence criteria.

In C++, use `params.lmParams.multifrontalParams.qrMode = MultifrontalParameters::QRMode::Off`. In MATLAB, retrieve `lm = params.lmParams` and `multifrontal = lm.multifrontalParams`, change `multifrontal.qrMode`, then assign `lm.multifrontalParams = multifrontal` and `params.lmParams = lm` before calling `problem.optimize(params)`. The QP convenience overload copies its options and preserves supplied value shapes.

## Shared implementation with the nonlinear optimizer

`AugmentedLagrangianOptimizer` and `AugmentedLagrangianSolver` use one internal engine for PHR terms, BCL/Aggressive outer updates, damping search, diagnostics and stopping. The nonlinear adapter evaluates original functions and Jacobians and retracts on the original variable types. The Gaussian adapter retains coefficient matrices, shifts only objective RHS/linear terms, and changes augmented constraint matrices only when penalties or activity change. Both reuse one ordering, symbolic structure and multifrontal workspace per call.

Numerical elimination is still repeated for each attempted LM solve. Reusing a numerical factorization or applying low-rank changes is future work. `lmParams` controls damping, ordering, multifrontal settings and inner limits; its optimizer backend, hooks, verbosity and log file are unused. No permanent regularization floor is introduced. Shared model reduction uses gradient/Hessian products, and roundoff-sized cost differences require a trial-gradient check before acceptance.

## Convergence and performance

Check `state.converged` and `state.status`: inner, outer and damping limits return the available candidate rather than certifying it. Native BCL convergence tests augmented stationarity and generalized feasibility. It does not promise an absolute original-QP duality-gap bound. A QP benchmark or application with a required gap tolerance must independently check the original objective, constraints and returned multipliers.

`state.statistics` always reports counts, including nonlinear linearizations, model updates, symbolic analyses and numerical eliminations. Set `params.enableProfiling=True` for separate diagnostic runs to collect preparation, evaluation, model assembly, elimination, back-substitution and diagnostic wall times. Leave profiling disabled for headline timing comparisons. The [QP benchmark](QpBenchmark.md) reports both successes and remaining failures on MPC and G1 inputs.
