# Dynamics factors for MPPI planning

By Akash Ratheesh and Frank Dellaert

This part of STAG (sense, think, act with factor graphs) models motion as a
chain of states and controls. A dynamics factor on `X(k), U(k), X(k+1)`
compares the model's prediction with the supplied next state. Its residual
is zero when the two agree. A nonlinear optimizer can differentiate the
residual, and a sampling planner can use the same prediction to construct
a feasible trajectory. [Planning architecture and API](../../gtsam/planning/doc/PlanningOverview.md)

| Class | Role |
| --- | --- |
| `StagDynamicsModel<State, Control>` | Predicts the next state for a given control and time step, with optional Jacobians. |
| `StagDynamicsFactor<State, Control>` | Evaluates the local coordinates of the next state about the predicted state. |
| `PointMass` and `PointMassDynamicsFactor` | Describe planar motion under acceleration. The point mass also exposes its tangent velocity through `PointMass::dynamics`. |
| `Unicycle` and `UnicycleDynamicsFactor` | Advance a `Pose2` by a forward body-frame translation and a turn. |

The point-mass example passes a vector field to `SamplingPlanner`. The
planner's Euler step integrates position from velocity and velocity from
acceleration, reproducing `PointMass::predict`. Both unicycle examples pass
a discrete transition that calls `Unicycle::predict`, so planning and
simulation use exactly the same motion over one time step.

A graph may contain the dynamics factors themselves. The open-loop unicycle
example includes them alongside the state and control priors. Every sampled
trajectory satisfies the dynamics by construction, so their residuals vanish.
The receding-horizon example scores just those priors and uses its transition
to enforce the same dynamics. Both examples use a Gaussian proposal and the
MPPI update. [C++ examples](../examples/mppi/README.md)

The Python guide constructs a unicycle horizon and checks the discrete
predictions against its dynamics factors.
[Scoring a unicycle trajectory with dynamics factors](doc/MPPIWithDynamicsFactors.ipynb)
