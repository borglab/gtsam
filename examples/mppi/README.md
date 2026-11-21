# MPPI Examples

These examples plan the controls of simple robots with MPPI (model predictive
path integral control) on a GTSAM factor graph. They use the `mpc_stag` module
of `gtsam_unstable`, so they are built when `GTSAM_BUILD_UNSTABLE` is on.

Each example builds a factor graph over a horizon of states and controls.
Dynamics factors link each state to the next, and priors give quadratic costs
on the states (toward a goal) and on the controls (toward zero). MPPI samples
control sequences around the current plan, rolls each one out through the
dynamics, scores it with the error of the graph, and moves the plan to the
weighted mean of the samples, where cheaper samples weigh more.

## Examples

| Example | Robot | What it shows |
|---|---|---|
| `PointMassMPPITrajectoryOptimizationExample` | A point mass in the plane: state [px, py, vx, vy], control the acceleration [ax, ay] | Trajectory optimization from rest at the origin to rest at (3, 2) in 3 s. |
| `UnicycleMPPITrajectoryOptimizationExample` | A unicycle: state a `Pose2`, control [v, omega], the forward speed and the yaw rate | Trajectory optimization for a robot that cannot move sideways, from the origin facing along x to (3, 2) facing along y in 3 s. |
| `UnicycleMPPIMPCExample` | The same unicycle, simulated with process noise | Model predictive control toward the same goal, until the unicycle is within 0.05 m and 0.1 rad of it. |

## Trajectory Optimization and MPC

The two trajectory optimization examples call `optimize()` once. MPPI runs 30
iterations on one problem and returns an open-loop plan of the controls and
the states over the whole horizon. Both plans end within 3 cm of the goal.

The MPC example runs the same optimization in a receding-horizon loop. At
every control step, x_0 is set to the current pose, the controls are
warm-started with the previous plan shifted by one step, MPPI runs one
iteration, and only the first control is applied. The simulated unicycle
drifts from the plan because of its process noise, and each new plan starts
from the pose it reached. It reaches the goal tolerance in under 5 s of
simulated time.

## Building and Running

From a GTSAM build folder, build and run an example with its `.run` target:

```
make PointMassMPPITrajectoryOptimizationExample.run
make UnicycleMPPITrajectoryOptimizationExample.run
make UnicycleMPPIMPCExample.run
```

The trajectory optimization examples print the cost of the initial and of the
optimized plan, the plan every 0.5 s, and the final distance to the goal. The
MPC example prints the pose and the applied control every 0.5 s, and the time
at which it reached the goal.
