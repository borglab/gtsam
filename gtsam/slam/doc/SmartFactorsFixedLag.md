# Smart Factors in Fixed-Lag Smoothers

When a pose observed by a smart factor leaves the window of a `BatchFixedLagSmoother` or an `IncrementalFixedLagSmoother`, `FixedLagSmoother::setMarginalizationMode` selects what happens to the factors that touch it:

*   `MARGINALIZE` (default) marginalizes the pose out of all of them. This is exact, but it consumes the smart factor into a dense linear marginal over the other poses of its track, and the landmark receives no further measurements.
*   `CONDITION` first replaces every factor that supports `NonlinearFactor::conditionOn` by a copy in which the pose is held constant at its current estimate, then marginalizes the remaining factors. The smart factor keeps the measurement of such a *fixed camera* for triangulation and for the landmark information, but the camera has no key and no Jacobian block.

The notation is that of [SmartFactors.md](SmartFactors.md). Two runnable examples exist as a C++ program in `examples/` and as a notebook in `python/gtsam/examples/nonlinear/`: `VisualFixedLagSmootherExample_SmartFactor` runs both smoothers in both modes on the visual odometry dataset of `StereoVOExample_large`, and `FixedLagSmootherExample_SmartFactor` compares the modes with the full batch solution on a simulated loop whose tracks start and end at random.

## Marginalization

Let $x_1$ be the variable that leaves the window, $x_r$ its Markov blanket, and $\|A_1\,\delta x_1 + A_r\,\delta x_r - c\|^2$ the linearized factors touching $x_1$. Eliminating $x_1$ leaves the marginal information

```math
\Lambda_r = A_r^\top \big(I - A_1 (A_1^\top A_1)^{-1} A_1^\top\big) A_r, \qquad
\eta_r = A_r^\top \big(I - A_1 (A_1^\top A_1)^{-1} A_1^\top\big) c ,
```

the Schur complement of [SmartFactors.md](SmartFactors.md) with the roles of camera and landmark exchanged. Both smoothers store it as `LinearContainerFactor`s and never relinearize the variables it touches again (first-estimate Jacobians; the batch smoother does so under its default `enforceConsistency`). For a smart factor this has two consequences: the landmark cannot be re-attached to later measurements without counting the old ones twice, so its track ends, and every pose that shared the factor with $x_1$ stops being relinearized, which under dense co-visibility is the whole window.

`IncrementalFixedLagSmoother` indexes a factor by the keys it had when it was added, so a later `add()` leaves the variable index and the Bayes tree of iSAM2 inconsistent ([issue #1976](https://github.com/borglab/gtsam/issues/1976)). To add a measurement, copy the factor, add the measurement to the copy, and pass the copy in `newFactors` together with the slot of the old factor in `factorsToRemove`; `FixedLagSmoother::Result::newFactorsIndices` reports the slot of the copy. Both examples follow this *replace protocol*, which works with both smoothers. `BatchFixedLagSmoother` finds the factors of a leaving key by their current keys, so a factor inside it may also grow in place with `add()`, provided the caller reads it back from `getFactors()` at its slot after each update: `CONDITION` replaces it in place by its conditioned copy, and a slot that no longer holds a smart factor means the factor was marginalized.

## Conditioning on Fixed Cameras

All measurements of a smart factor share an isotropic noise model, so with whitened $F$, $E$, $b$ the matrix $Q = I - E P E^\top$, $P = (E^\top E)^{-1}$, is an orthogonal projector and the factor linearizes to $G = F^\top Q F$, $g = F^\top Q b$. A fixed camera has no Jacobian, so with the rows split into active rows $a$ and fixed rows, $F = [F_a; 0]$ and

```math
G = F_a^\top \big(I - E_a P E_a^\top\big) F_a, \qquad
g = F_a^\top \big(b_a - E_a P E^\top b\big),
```

with $P$ and $E^\top b$ taken over all rows. This is exactly the block of the unconditioned factor's $(G, g)$ that belongs to the live cameras. Holding $x_1$ fixed is also the limit of marginalizing it with an infinitely tight prior: eliminating $\delta x_1$ from $\|F_1\,\delta x_1 + E_1\,\delta p - b_1\|^2 + \|W\,\delta x_1\|^2$ leaves the fixed row $\|E_1\,\delta p - b_1\|^2$ as $W \to \infty$.

Conditioning is therefore not marginalization: it replaces $x_1$ by a point mass at its estimate $\hat x_1$, ignoring its uncertainty, and the information the measurement carried about $x_1$ never reaches the marginal. In a four-camera example the difference between the conditioned information and the exact marginal over the remaining poses had eigenvalues from $-2.3 \cdot 10^{2}$ to $+2.5 \cdot 10^{5}$. In return every measurement is used once, the track continues, and only the odometry neighbours of $x_1$ are frozen.

The incremental smoother fixes $x_1$ at the estimate of the previous update, because iSAM2 replaces factors before it marginalizes, and it marginalizes keys that arrive already outside the window, which have no estimate yet; the batch smoother uses the estimate it has just optimized. Both replace a factor added in the same update in place. Triangulation uses the fixed cameras together with the live ones, so live cameras that are degenerate on their own, for example under pure rotation, become well posed once a fixed camera provides baseline. A landmark that stays in view gains a fixed camera every frame, so `SmartProjectionParams::maxFixedCameras` (default 10, zero for unbounded) makes `conditionOn` drop the oldest fixed cameras beyond that number.

`SmartProjectionPoseFactor` and `SmartProjectionFactor` support conditioning in the `HESSIAN`, `JACOBIAN_Q` and `JACOBIAN_SVD` modes; `SmartProjectionRigFactor` (every measurement taken from the fixed body pose) in `HESSIAN`, and `SmartStereoProjectionPoseFactor` in `HESSIAN` and `JACOBIAN_SVD`, the modes those two factors linearize in. A measurement of `SmartStereoProjectionFactorPP` or `SmartProjectionPoseFactorRollingShutter` still depends on a variable after one pose is fixed, so those factors return `nullptr` from `conditionOn` and are marginalized.

| C++ / Python | Purpose |
|---|---|
| `NonlinearFactor::conditionOn(fixedValues)` | copy with the keys in `fixedValues` held constant, or `nullptr` (`None`) if the factor does not support it or would keep none or all of its keys |
| `fixCamera(key, values)` on the pose, projection and rig factors | copy of the same type with one camera fixed, without the `maxFixedCameras` bound |
| `fixedCameras()`, `fixedMeasurements()`, `isFixedMeasurement(i)`, `activeMeasurements()` | the fixed cameras, and which measurements are fixed or live |
| `cameraForMeasurement(i, world_P_body)` | camera of measurement `i` of a rig or stereo factor for a body pose |
| `FixedLagSmoother::setMarginalizationMode` | select `MARGINALIZE` or `CONDITION` |
| `FixedLagSmoother::Result::conditionedFactorIndices`, `newFactorsIndices` | slots of the replaced factors, and of the new factors: the caller's first, then the replacements in the same order |

## Measured Behaviour

`gtsam/nonlinear/tests/smartFactorFixedLagScenario.h` simulates an orbiting camera over 30 frames with lag 6, 24 landmarks each visible for 12 frames, 1 px pixel noise, odometry noise of 0.03 m and 0.005 rad per step, and a prior on $x_0$ only. The reference is the full batch solution over all data so far, whose newest pose drifts up to 0.99 m from the truth.

| incremental smoother, newest pose | worst distance to full batch | worst distance to truth | longest factor | frozen keys |
|---|---|---|---|---|
| `MARGINALIZE` | 1.51 m | 1.03 m | 7 = lag + 1 | 7, the whole window |
| `CONDITION` | 1.71 m | 1.55 m | 12, the whole track | 1 |
| `CONDITION`, cameras fixed at the true poses | | 1.01 m | 12 | 1 |

The batch smoother behaves the same way (1.02 m and 1.30 m from the truth). The last row shows that the loss of `CONDITION` comes from the estimation error a pose has when it is fixed: `MARGINALIZE` keeps the correlation between the leaving pose and the window, so later measurements still correct the window relative to the past, whereas a fixed camera anchors the window at the error it had when it was fixed.

Mean / maximum newest-pose error against the full batch after the first marginalization, in meters; "anchored" adds an absolute pose measurement with $\sigma = 0.05$ to every frame:

| scenario | incremental `MARGINALIZE` | incremental `CONDITION` | batch `MARGINALIZE` | batch `CONDITION` |
|---|---|---|---|---|
| drifting, seed 42, lag 6 | 0.594 / 1.511 | 0.937 / 1.707 | 0.563 / 1.447 | 0.836 / 1.535 |
| drifting, seed 7, lag 6 | 0.320 / 0.540 | 0.343 / 0.707 | 0.315 / 0.551 | 0.472 / 0.957 |
| drifting, seed 3, lag 4 | 0.483 / 0.967 | 0.296 / 0.726 | 0.491 / 0.972 | 0.235 / 0.586 |
| drifting, seed 42, lag 10 | 0.039 / 0.076 | 0.695 / 1.406 | 0.050 / 0.105 | 0.495 / 1.097 |
| anchored, seed 42, lag 6 | 0.012 / 0.025 | 0.010 / 0.025 | 0.012 / 0.025 | 0.009 / 0.022 |
| anchored, seed 7, lag 4 | 0.012 / 0.024 | 0.007 / 0.015 | 0.012 / 0.024 | 0.006 / 0.015 |

When the trajectory drifts, `MARGINALIZE` has the lower mean error in most runs, and a longer lag helps it far more than `CONDITION`. When it is anchored, so that a pose hardly changes after leaving the window, `CONDITION` matches or beats `MARGINALIZE`: there is no correction to lose and continuity is a pure gain.

In the loop of `examples/FixedLagSmootherExample_SmartFactor.cpp` (forward-looking camera, 120 landmarks, 30 percent detection dropout, 60 frames, lag of 8 frames) the outcome depends on the data. Mean newest-pose error against the full batch, in meters, for the default seed 7 and eight further seeds passed as the program's argument:

| seed | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| incremental `MARGINALIZE` | 0.140 | 0.169 | 0.105 | 0.097 | 0.099 | 0.042 | 0.092 | 0.123 | 0.078 |
| incremental `CONDITION` | 0.079 | 0.161 | 0.117 | 0.067 | 0.117 | 0.050 | 0.082 | 0.158 | 0.082 |
| batch `MARGINALIZE` | 0.141 | 0.170 | 0.114 | 0.095 | 0.099 | 0.040 | 0.089 | 0.122 | 0.075 |
| batch `CONDITION` | 0.078 | 0.157 | 0.116 | 0.066 | 0.103 | 0.045 | 0.061 | 0.160 | 0.087 |

With dense co-visibility a new pose can be consumed into a marginal in the update that adds it, and the incremental smoother then freezes it at its initial value. Initialize each new pose from the current estimate composed with the odometry step; the newest-pose estimates of the two smoothers then differ by at most 4.1 cm with `MARGINALIZE` and 6.5 cm with `CONDITION` over the nine seeds.

## Choosing a Mode

`MARGINALIZE` computes the marginal of the joint posterior over the window and remains the default. `CONDITION` is a different model: it is unbiased only when the poses leaving the window are well determined, and it is overconfident otherwise. Use it when track continuity and a relinearizable window matter and the trajectory is anchored, for example by absolute position measurements.

With the defaults, adding, replacing and conditioning smart factors keeps every data structure, the time per update, and the memory constant over thousands of frames. Without `maxFixedCameras` a landmark that never leaves the view grows its factor, and its linearization cost, without bound. Custom `ISAM2Params` must keep `findUnusedFactorSlots` true, as the smoother's defaults do; otherwise every removed factor leaves an empty slot ([issue #1452](https://github.com/borglab/gtsam/issues/1452)).
