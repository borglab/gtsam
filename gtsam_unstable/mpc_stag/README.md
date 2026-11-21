# MPC on Factor Graphs with MPPI

This module plans the controls of a robot with MPPI (model predictive path
integral control) on a GTSAM factor graph. The plan is the future half of a
STAG graph (sense, think, act with factor graphs): the states x_0 ... x_H and
the controls u_0 ... u_{H-1} over a horizon of H steps, linked by dynamics
factors, with any GTSAM factors as costs.

## MPPI Version

The controller implements the information-theoretic MPPI of Williams et al.
[1, 2], without the importance-sampling control-cost term in the sample
weights: the case alpha = 1, so gamma = lambda (1 - alpha) = 0, of [2]. The
update is described in [The MPPI Update](#the-mppi-update).

## Contents

| Folder | Class | Role |
|---|---|---|
| `dynamics/` | `StagDynamicsModel<STATE, CONTROL>` | Base class for a discrete-time model x_{k+1} = f(x_k, u_k, dt), with Jacobians. |
| `dynamics/` | `StagDynamicsFactor<STATE, CONTROL>` | Factor on x_k, u_k and x_{k+1}, with error Local(f(x_k, u_k, dt), x_{k+1}). |
| `dynamics/models/` | `PointMass`, `Unicycle` | A point mass in the plane and a unicycle, with the dynamics factors `PointMassDynamicsFactor` and `UnicycleDynamicsFactor`. |
| `controller/` | `StagMPCController` | Base class for controllers that optimize the future graph. |
| `controller/` | `GenericMPPIController<STATE, CONTROL, SAMPLER>` | MPPI on the future graph, generic over the control noise sampler. |
| `controller/` | `VanillaMPPIController<STATE, CONTROL>` | MPPI with Gaussian control noise from a `GaussianNoiseSampler`. |
| `controller/sampler/` | `GaussianNoiseSampler` | Gaussian control noise for many control sequences at once, the same for any number of threads. |

## Using the Controller

A plan takes three steps:

1. Build the future graph: one `StagDynamicsFactor` per step on X(k), U(k) and
   X(k+1), and the cost factors, for example priors on the states and the
   controls.
2. Fill the initial values. x_0 is the state every rollout starts from, the
   controls are the nominal plan, and every other state needs a value of its
   type.
3. Call `VanillaMPPIController::optimize(graph, initial, controlKeys)`. It
   returns the planned controls and the states rolled out with them.

The controller needs:

- a CONTROL that is a fixed-size Eigen vector, such as `Vector2`;
- values that hold exactly a STATE and a CONTROL at the keys of the dynamics
  factors, for example a `Vector2` and not a dynamic `Vector`;
- one dynamics factor per control key, chained in step order with distinct
  keys;
- a model and cost factors that are safe to evaluate from several threads at
  once, since with TBB all samples are rolled out and scored in parallel.

The examples in `examples/mppi` show trajectory optimization with one call to
`optimize()`, and model predictive control with a call at every control step.

## The MPPI Update

Each iteration of `optimize()` is the information-theoretic MPPI update of
[1, 2]. K control sequences v_i = ubar + eps_i are sampled, with eps_i drawn
from N(0, Sigma) at every step, and each is rolled out and scored with S_i,
the error of the future graph. The nominal controls become the weighted mean
ubar = sum_i w_i v_i, with w_i = exp(-(S_i - min S) / lambda) normalized. The
weights have no importance-sampling control-cost term: this is the case
alpha = 1, so gamma = lambda (1 - alpha) = 0, of [2], where the base
distribution is that of the current plan. Control costs are factors of the
future graph on the sampled controls.

A rollout satisfies the dynamics factors of its chain, so they are not
evaluated. Priors of exactly the type `PriorFactor<STATE>` or
`PriorFactor<CONTROL>` on the chain's keys are scored on the rolled-out states
and controls directly, and every other factor through `Values`, with the same
result. With TBB, the costs and the plan are the same for any number of
threads.

## Tests

The tests are in `tests/`. From a GTSAM build folder, run them with:

```
make check.mpc_stag_unstable
```

## References

1. G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots and
   E. A. Theodorou, "Information Theoretic MPC for Model-Based Reinforcement
   Learning", ICRA, pp. 1714-1721, 2017.
   [doi:10.1109/ICRA.2017.7989202](https://doi.org/10.1109/ICRA.2017.7989202)
2. G. Williams, P. Drews, B. Goldfain, J. M. Rehg and E. A. Theodorou,
   "Information-Theoretic Model Predictive Control: Theory and Applications to
   Autonomous Driving", IEEE Transactions on Robotics, 34(6):1603-1622, 2018.
   [doi:10.1109/TRO.2018.2865891](https://doi.org/10.1109/TRO.2018.2865891),
   [arXiv:1707.02342](https://arxiv.org/abs/1707.02342)
