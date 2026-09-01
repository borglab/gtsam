# Logmap IMU Preintegration Results

This report summarizes the executed
[Galilean IMU factor NEES notebook](./GalileanImuFactorNEES.ipynb). All four
GTSAM preintegration backends use the default $SE_2(3)$ Logmap factor error,

```cpp
error = state_j.logmap(predictedState_j);
```

The notebook is the executable source of truth. Its mechanical Monte Carlo,
statistics, tables, and Plotly code are kept in
[`galilean_imu_factor_nees.py`](./galilean_imu_factor_nees.py).

The Galilean row is the companion paper's $\mathrm{Gal}(3)\times\mathbb R^6$
direct-product model. It retains Delama et al.'s held-input composition and
uses Brossard et al.'s endpoint and rotating-frame structure, but its physical
bias model, left-invariant covariance, and right correction are the companion
paper's formulation.

## How to read the results

Normalized Estimation Error Squared (NEES) tests whether a residual and its
predicted covariance agree. The expected mean is the residual dimension: 9 for
the ordinary IMU factor and 15 for the Combined factor. A value nearest that
mean is best; a smaller NEES is not automatically better.

Physical position and velocity RMS errors are reported separately. A residual
chart cannot repair deterministic integration error.

## One-second inertial stress test

This experiment integrates simultaneous high rotation and acceleration at
20 Hz. The expected 95% interval for mean 9D NEES is [8.849, 9.152].

| Backend | Position RMS (m) | Velocity RMS (m/s) | Mean Logmap error norm | Mean NEES |
|---|---:|---:|---:|---:|
| Manifold | 0.0578 | 0.1049 | 0.0818 | 14.502 |
| Tangent | 0.0578 | 0.1049 | 0.0818 | 14.507 |
| Lie group | 0.0578 | 0.1049 | 0.0818 | 14.502 |
| Galilean | **0.0427** | **0.0767** | **0.0011** | **8.959** |

The established backends are overconfident in this deliberately stressful
finite-rate case. Galilean preintegration removes most of the held-input mean
error and remains statistically consistent.

## Powered ascent in a rotating Earth frame

The four-second sounding-rocket scenario includes 12 g measured specific
force, changing attitude, gravity, and Earth rotation. “Specified” means the
known Earth rate is supplied through `omegaCoriolis`.

| Backend | Earth rate | Position RMS (m) | Velocity RMS (m/s) | Mean NEES |
|---|---|---:|---:|---:|
| Manifold | Not specified | 1.4079 | 0.8057 | 16.715 |
| Manifold | Specified | 1.2364 | 0.6859 | 14.012 |
| Tangent | Not specified | 1.4079 | 0.8057 | 16.715 |
| Tangent | Specified | 1.2364 | 0.6859 | 14.012 |
| Lie group | Not specified | 1.4079 | 0.8057 | 16.715 |
| Lie group | Specified | 1.2364 | 0.6859 | 14.012 |
| Galilean | Not specified | **0.8491** | **0.5317** | 9.663 |
| Galilean | Specified | **0.8213** | **0.5070** | **9.090** |

The expected mean-NEES interval is again [8.849, 9.152]. Correctly supplying
Earth rate improves every backend. Galilean is the only backend inside the
interval when Earth rate is specified.

## Long-horizon uncertainty with fixed nonzero bias

This ten-second experiment centers every backend on its own noise-free
discrete mean, isolating accumulated sensor uncertainty while retaining a
fixed nonzero accelerometer and gyroscope bias. The expected mean-NEES interval
is [8.815, 9.187].

| Backend | Mean NEES | Median NEES | Mean Logmap error norm |
|---|---:|---:|---:|
| Manifold | 8.995 | 8.266 | **0.2156** |
| Tangent | **8.996** | 8.286 | 0.3133 |
| Lie group | 8.995 | 8.266 | **0.2156** |
| Galilean | 8.993 | 8.253 | 0.2193 |

All four Logmap results are statistically consistent and within 0.007 of the
theoretical mean.

## Full 15D uncertainty with bias random walk

The Combined experiment samples accelerometer and gyroscope bias random walks
and retains the full state-bias covariance. Its expected mean-NEES interval is
[14.761, 15.241].

| Backend | Mean NEES | Median NEES | Mean Logmap error norm |
|---|---:|---:|---:|
| Manifold | 15.011 | 14.310 | **0.1369** |
| Tangent | 15.027 | 14.329 | 0.3117 |
| Lie group | 15.011 | 14.310 | **0.1369** |
| Galilean | **15.010** | 14.333 | 0.1390 |

Every backend remains inside the confidence interval when the state block uses
the Logmap residual.

## Right-applied first-order bias correction

The companion paper defines the Galilean first-order correction

$$
\widehat\Upsilon_{ij}(\hat b+\delta b)
\simeq
\widehat\Upsilon_{ij}(\hat b)\operatorname{Exp}(J_b\delta b)
$$

on the right because its direct-product state uses a standard left-invariant
error. The deterministic experiment compares this correction, and each other
backend's corresponding first-order approximation, against complete
reintegration. It uses a nonzero bias linearization point, one second of smooth
three-axis motion, 2,000 paired random directions per magnitude, and
$\lVert\delta b_a\rVert=30\lVert\delta b_\omega\rVert$.
The table reports median error at the largest joint update, 0.35.

| Backend | Rotation (microdeg) | Position (mm) | Velocity (cm/s) |
|---|---:|---:|---:|
| Manifold | 209.922 | 0.5047 | 0.1593 |
| Tangent | **1.503** | 0.5047 | 0.1593 |
| Lie group | 209.922 | 0.3862 | 0.0226 |
| Galilean | 209.922 | **0.3035** | **0.0130** |

The corrected Lie-group implementation materially improves the position and
velocity approximation over the additive Manifold/Tangent correction. Its
velocity error is about seven times smaller at the largest update. Galilean is
best in position and velocity for this held-input trajectory. Manifold,
Lie-group, and Galilean have nearly identical attitude error, while Tangent's
tangent-space correction differs.

All backends are exact at zero bias update, and their local first-order
approximation errors scale quadratically with update magnitude.

## Recommendations

- Use the default **Logmap** IMU factor-error mode.
- Use **Galilean** preintegration for high dynamics, simultaneous rotation and
  acceleration, lower IMU rates, longer held-input intervals, or rotating
  navigation frames.
- Use **Tangent** for established general-purpose and PIM-merging workflows.
- Use **Lie group** when the formulation requires $SE_2(3)$ increments and its
  complete right-applied group bias update.
- Supply `omegaCoriolis` whenever navigation-frame rotation is known.

`Legacy` and `ComponentWise` remain compatibility choices, but they are not
active modes in the current notebooks.
