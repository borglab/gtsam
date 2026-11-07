---
title: QP solver benchmarks
downloads:
  - file: qp-benchmark-report.html
    title: HTML report with interactive charts (internet connection required)
---

# QP solver benchmarks

This benchmark compares quadratic-programming solvers on robotic control and trajectory-optimization problems. A quadratic program (QP) minimizes a quadratic cost while satisfying linear equations and limits. The variables can represent planned motion or contact forces. We measure time and independently check each returned solution against its original input.

The public [MPC benchmark](https://qpsolvers.github.io/qpbenchmark/mpc_qpbenchmark/) contains 64 problems. Each runs at four accuracy settings, giving 256 combinations per solver. Three larger G1 humanoid captures provide a separate comparison. These are fixed QPs, not full trajectory-optimization runs. Both Dual backends are also attempted on all three G1 captures. GTSAM Dense Primal and DAQP are not run on G1.

Download the report: {download}`qp-benchmark-report.html`. Interactive charts use the Plotly CDN and require an internet connection. Full per-call measurements and the sortable table remain in the local benchmark workspace and are not published.

## Benchmark datasets

Model predictive control (MPC) plans a short future horizon, applies the first action, and replans. Each snapshot here is solved cold; the timings do not measure controller warm starts.

- **LIPMWALK** contains 30 humanoid-walking QPs based on a linear inverted-pendulum model, each with 16 variables.
- **WHLIPBAL** contains 30 wheeled-balancing QPs, each with 50 variables.
- **QUADCMPC** contains four quadruped centroidal-control QPs. They plan whole-body motion through foot-contact forces, include equations and fixed bounds, and have diagonal rank-deficient Hessians.
- **G1SP20** contains 171 contact-support slices from a 1,000-sample, 78–98 s humanoid motion clip. **G1CMCTWO** uses all 1,000 samples of that clip. **G1SPFULL** contains 1,780 support slices from a 0–219.94 s recording. Motion windows are not solver time limits.

Constraint counts below refer to the original inputs. Each finite lower or upper bound counts once; a fixed variable contributes two bound sides. G1's equality equations were eliminated before capture, and its stored objective already contains the application's LM damping.

| Dataset | Cases | Variables | Equality rows | Inequality rows | Finite bound sides | Fixed variables |
| --- | --- | --- | --- | --- | --- | --- |
| LIPMWALK | 30 | 16 | 0 | 32 | 0 | 0 |
| WHLIPBAL | 30 | 50 | 0 | 100 | 0 | 0 |
| QUADCMPC1 | 1 | 768 | 384 | 512 | 512 | 192 |
| QUADCMPC2 | 1 | 576 | 288 | 384 | 384 | 144 |
| QUADCMPC3 | 1 | 384 | 192 | 256 | 128 | 0 |
| QUADCMPC4 | 1 | 384 | 192 | 256 | 256 | 96 |
| G1SP20 | 1 | 18,549 | 0 | 230 | 0 | 0 |
| G1CMCTWO | 1 | 106,836 | 0 | 378 | 0 | 0 |
| G1SPFULL | 1 | 193,566 | 0 | 2,437 | 0 | 0 |

## Solver methods

GTSAM ALM uses an augmented Lagrangian method with native BCL penalty/multiplier updates and Levenberg–Marquardt inner solves from zero. The [QP user guide](QpProblem.ipynb) explains the public API.

GTSAM Sparse and Dense are the primal entries and use an active-set algorithm: they solve on a working set of binding constraints, then add or release rows. Both use FeasibleStepThenProjectedGradient initialization and continue to full optimization. MPC retains the requested proximal weight of 1e-9; G1 uses zero additional regularization. Sparse reuses an incremental Gaussian Bayes tree as constraints change. A changed proximal center uses the cached factorization with no numerical eliminations. In the MPC implementation, Sparse also applies one original-gradient residual correction through that same factorization to retain accuracy along weak working directions. Exact affine constraints and the requested regularization are preserved.

Dense packs square-root costs and constraints once, eliminates permanent equalities, and explores inequality working sets in the remaining coordinates. Fixed coordinates are substituted exactly. One per-call Dense workspace shares its equality preparation across cold feasibility, projected-gradient initialization and the main solve. Dense caches an objective QR factorization and updates LDLᵀ factors for the changing working constraints. The primal solve and retained equality factors also supply multipliers when they pass the original-objective checks. A rank-aware null-space fallback handles singular curvature. Original rows remain authoritative for feasibility and multipliers; no extra damping or dropped constraints are introduced.

GTSAM Sparse Dual and GTSAM Dense Dual select the new Dual formulation. They start with zero multipliers, bypass primal feasibility search and primal initialization, and use proximal weight 1e-9 on MPC and zero additional regularization on G1. Both eliminate permanent equalities and update an inequality working LDLᵀ system. Dense uses reduced square-root objective factors; Sparse applies a cached equality-constrained inverse without a global dense inverse or nullspace. Original-coordinate refinement preserves weak-direction accuracy, and all convergence certificates use the original QP. These additional entries are measured on both MPC and G1.

ProxQP uses a proximal primal-dual augmented Lagrangian method with semismooth Newton inner iterations. DAQP uses a dual active-set method with recursive LDLᵀ updates. Clarabel uses a primal-dual interior-point method. OSQP uses operator splitting, alternating linear-system solves with projections onto constraint bounds.

| Method | Optimization family | Configuration |
| --- | --- | --- |
| GTSAM ALM | Augmented Lagrangian | Native BCL penalty updates and Levenberg–Marquardt inner solves with a persistent nonlinear multifrontal workspace; zero initial values |
| GTSAM Sparse | Active set | Multivariate factor inputs; incremental Gaussian Bayes-tree working solves, cached RHS changes and one original-gradient residual correction; FeasibleStepThenProjectedGradient initialization and full original-QP optimality checks |
| GTSAM Chronological | Active set | The same sparse solver with chronological state blocks and shared-variable retirement; G1 only |
| GTSAM Slice COLAMD | Active set | Chronological slice groups, constrained COLAMD within each group of original key blocks; G1 only |
| GTSAM Dense | Active set | Permanent equality preparation shared across feasibility, componentwise initialization and the main solve; packed square-root costs, cached objective QR, working LDLᵀ updates and reused primal/equality factors for multipliers; rank-aware null-space fallback; MPC only |
| GTSAM Sparse Dual | Dual active set | Zero-multiplier cold start; equality elimination through cached sparse constrained solves; working LDLᵀ updates and original-coordinate refinement; proximal weight 1e-9 on MPC, zero additional regularization on G1; MPC and G1 |
| GTSAM Dense Dual | Dual active set | Zero-multiplier cold start; permanent equality reduction and square-root objective factors; working LDLᵀ updates and original-coordinate refinement; proximal weight 1e-9 on MPC, zero additional regularization on G1; MPC and G1 |
| ProxQP | Proximal primal-dual augmented Lagrangian | Solves regularized subproblems with semismooth Newton inner iterations |
| DAQP | Dual active set | Recursive LDLᵀ updates; dense MPC only; version 0.8.7 uses selective proximal regularization for weak curvature |
| Clarabel | Interior point | Primal-dual interior-point steps toward the constrained optimum |
| OSQP | Operator splitting (ADMM) | Alternates linear-system solves and projections onto constraint bounds |

## Timing and accuracy

GTSAM timings begin with assembled factors, its native input. Translation from benchmark matrices into those factors is excluded and recorded separately. The measured interval includes solver construction, Dense matrix packing, ordering, feasibility search, initialization, optimization, multiplier recovery, and output conversion. Dense equality projection, numerical factorization and Sparse residual correction are included. The 10-second MPC watchdog starts at this same boundary. The four GTSAM primal/dual entries were rerun together for this update; ALM and external-solver measurements retain their previous provenance. Baseline timings retain their upstream matrix-input adapters and setup. This is a comparison from each library's input representation, rather than the cost of translating a common matrix format into every representation.

GTSAM's MPC factor graphs group related scalar unknowns into vector variables, each identified by one key. This representation preserves the QP dimension. WALK and BAL keep their condensed control sequence in one vector key with one objective factor. QUAD uses one 12-dimensional state key and four 3-dimensional contact-force keys per time step, with 12-row dynamics factors. All matrix coefficients, original constraint rows and dual mappings are preserved; no state variables are reintroduced. G1 already retains its original variable blocks and Jacobian factors.

MPC runs serially on CPU 0 with one numerical-library thread and one TBB thread. All inputs use cold starts. Five additional fresh-solver timings follow each initially passing combination; they reuse no optimizer state. Default preserves native stopping rules and uses the upstream external threshold of 1.0. Low, mid, and high accuracy use absolute thresholds of 1e-3, 1e-6, and 1e-9. Default is therefore not an equal-accuracy comparison.

Independent checks evaluate primal feasibility, stationarity, and the primal–dual gap on the original unregularized QP. A supplemental dual-feasibility check verifies multiplier signs. A native success flag or a merely feasible step does not establish optimality. Numerical failures, iteration limits, failed certificates, and watchdog stops remain distinct. Failed calls remain in profile denominators and receive no speed reward.

## MPC results

| Solver | Published checks | Also dual feasibility | QUAD, all settings |
| --- | --- | --- | --- |
| GTSAM ALM | 178/256 | 178/256 | 7/16 |
| GTSAM Sparse | 253/256 | 253/256 | 16/16 |
| GTSAM Dense | 253/256 | 253/256 | 16/16 |
| GTSAM Sparse Dual | 256/256 | 256/256 | 16/16 |
| GTSAM Dense Dual | 256/256 | 256/256 | 16/16 |
| ProxQP | 253/256 | 235/256 | 13/16 |
| DAQP | 248/256 | 248/256 | 8/16 |
| Clarabel | 256/256 | 256/256 | 16/16 |
| OSQP | 247/256 | 247/256 | 9/16 |

```{iframe} /qp-plots/mpc-success.html
:width: 100%
:class: qp-chart
:title: Counts passing published metrics and supplemental dual feasibility.

Counts passing published metrics and supplemental dual feasibility.
```

A separate controlled sweep covers 108 seeded strictly convex QPs with 16, 64, or 128 variables, zero or half as many independent equalities, and inequality counts below, at, and above the reduced dimension. Each backend/formulation combination has three cold calls: all 1,944 calls pass the absolute 1e-7 original-input checks with zero proximal weight. That campaign used the earlier dimension-based Automatic rule. A fresh validation of the new policy passes all 648 Automatic calls (three calls per backend per problem), selecting Dual whenever inequalities are present and Primal otherwise. These synthetic objectives have at most two positive inequality multipliers, so their speed results favor small dual working sets and do not establish a universal crossover.

Automatic is now the default formulation, with Sparse as the default numerical backend. It selects Dual for cold starts with inequalities, independently of their count. A converged C++ warm start retains its previous formulation when the supplied point is feasible; nonconverged states or infeasible points select Dual. Values alone provide a cold proximal center. Equality-only and fully fixed problems use Primal, and singular reduced curvature without regularization falls back to Primal during setup. LPs resolve Automatic to Primal and reject explicit Dual. Selection remains fixed for the optimization call; both formulations can still be requested explicitly.

The earlier dimension rule selected Primal on all 64 MPC cases and missed measured Dual speedups. The new policy passes all 512 MPC validation calls (64 problems, four accuracies, both backends), selecting Dual in every cold case. The plotted Primal and Dual timing series retain their original measurements and fingerprints; these new Automatic calls are validation, not replacement timing measurements. Sparse Dual already combines sparse objective factorization with a dense inequality working system. Choosing Dual does not remove the Dense backend's primal objective storage, as the G1 memory-limit results demonstrate.

A performance profile compares each passing time with the fastest passing solver on that same problem. At a horizontal value of 2, the vertical value is the fraction of all problems solved within twice that fastest time. Higher and further left is better. Curves ending below 1 indicate failures. Profiles use first-call times; the QUAD timing table gives medians of the five additional calls.

```{iframe} /qp-plots/mpc-profile.html
:width: 100%
:class: qp-chart
:title: Performance profiles over all 64 MPC cases at four accuracies.

Performance profiles over all 64 MPC cases at four accuracies.
```

## QUADCMPC results

Each step covers one of the four quadruped cases. Default timings below are medians of five additional fresh-solver calls, in seconds.

```{iframe} /qp-plots/quad-profile.html
:width: 100%
:class: qp-chart
:title: Performance profiles for the four QUADCMPC cases.

Performance profiles for the four QUADCMPC cases.
```

```{iframe} /qp-plots/quad-times.html
:width: 100%
:class: qp-chart
:title: Default-setting first-call times for QUADCMPC.

Default-setting first-call times for QUADCMPC.
```

| Default repeat median (s) | GTSAM ALM | GTSAM Sparse | GTSAM Dense | GTSAM Sparse Dual | GTSAM Dense Dual | ProxQP | DAQP | Clarabel | OSQP |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| QUADCMPC1 | 0.1465 | 0.03687 | 0.08361 | 0.02583 | 0.0551 | 0.1142 | 0.03363 | 0.005422 | 0.00369 |
| QUADCMPC2 | 0.1408 | 0.0223 | 0.03282 | 0.01152 | 0.02262 | 0.04279 | 0.01277 | 0.003868 | 0.003888 |
| QUADCMPC3 | 0.01988 | 0.003092 | 0.0132 | 0.004455 | 0.01331 | 0.01279 | 0.002231 | 0.002113 | 0.00166 |
| QUADCMPC4 | 0.02171 | 0.00809 | 0.01025 | 0.005249 | 0.007308 | 0.01182 | 0.003719 | 0.002202 | 0.002219 |

## GTSAM augmented Lagrangian results

ALM passes 178/256 MPC combinations and returns native convergence on 215. 41 calls stop with an unconverged inner solve. Native convergence alone can miss the independent gap or multiplier-stationarity checks: ALM checks an augmented gradient and feasibility, while the benchmark checks the original QP with the returned multipliers. No multiplier repair is applied.

Default retains all native ALM settings. Other accuracies change only absolute violation and stationarity tolerances. The outer limit remains 20, each inner LM limit 100, initial penalty 10, penalty growth 100, initial LM damping 1e-5, and LM relative/absolute error tolerances 1e-5. Native inner damping is not an additional permanent objective term.

## GTSAM working-system setup

Permanent equality elimination reduces the Dense working dimension before inequality exploration. QUADCMPC1 has 768 variables, 384 original equalities, and 192 fixed bounds; its 576 independent equations leave 192 free coordinates. The equality projection is part of measured solve time. Square-root objectives retain weak directions, and residual-based refinement uses cached triangular factors without refactorization. Ambiguous roundoff-sized constraint projections retain the original dense representation rather than truncating coefficients.

Exact diagonal Hessians use their diagonal square roots directly. Equality preparation excludes only rows that become exactly zero after substituting fixed coordinates; their original consistency checks and multiplier indices are preserved. Working LDLᵀ factors support constraint additions, removals and changed proximal centers. Refinement stops when the boundary residual is exactly zero or its correction cannot change the stored point, while weak directions retain refinement. No numerical elimination is needed for an RHS-only change.

Cold Dense feasibility checks the cached affine equality point against the original inequalities and retains the phase-one fallback when needed. Projected-gradient initialization uses the same equality basis and transformed normals in reduced tangent coordinates, without an affine offset. Each connected component keeps its own gradient scale and iteration budget, and the existing Dense active-set solver handles its auxiliary QP. Lifted directions are checked against the original normalized tangent rows before working-set selection.

The setup chart separates solver construction and Dense packing from initialization and numerical solving. All displayed segments are included in the benchmark time. Matrix-to-factor translation is excluded and does not appear as a timed segment. These are default-setting primary calls; the chart does not assume setup and solve components have the same bottleneck across solvers.

```{iframe} /qp-plots/gtsam-timing.html
:width: 100%
:class: qp-chart
:title: Included setup and solve components of the default GTSAM QUAD calls.

Included setup and solve components of the default GTSAM QUAD calls.
```

## DAQP settings

DAQP is a dual active-set solver using recursive LDLᵀ updates. The comparison pins DAQP 0.8.7 and qpsolvers 4.13.0, with the upstream accuracy settings. DAQP uses selective proximal regularization for QUAD's singular Hessians: zero/weak curvature pivots receive eps_prox, and proximal iterations recenter the forcing. It is not an unregularized QUAD solve. The configured eps_prox is 1e-6 and eta_prox is 1e-06; tightening upstream primal/dual tolerances does not change these proximal settings. Returned solutions still face the original-input certificates. DAQP passes QUAD at default and low accuracy; all four mid/high cases miss the absolute-gap check.

| Setting | primal_tol | dual_tol | eps_prox | eta_prox |
| --- | --- | --- | --- | --- |
| default | 1e-06 | 1e-12 | 1e-06 | 1e-06 |
| low_accuracy | 0.001 | 0.001 | 1e-06 | 1e-06 |
| mid_accuracy | 1e-06 | 1e-06 | 1e-06 | 1e-06 |
| high_accuracy | 1e-09 | 1e-09 | 1e-06 | 1e-06 |

## G1 results

G1 includes both Dual backends, alongside GTSAM ALM, three Sparse Primal orderings, and the ProxQP, Clarabel, and OSQP baselines. Each new dual call starts cold with zero vectors and zero multipliers, uses the original block Jacobian factors and captured LM damping, and adds no QP regularization. Independent checks use the original weighted Jacobian and inequalities, with a common absolute 1e-7 threshold for primal feasibility, stationarity, dual feasibility, complementarity, and gap.

Sparse Dual certifies 2/3 captures. G1SP20: 3.697 s (certified); G1CMCTWO: 25.504 s (certified); G1SPFULL: ≥300.0 s (watchdog timeout; no returned solution).

Dense Dual reaches the 16 GiB memory limit on every capture before completing a dual working solve. Its current backend still packs and factors the primal objective densely. The initial dense Hessian alone requires 2.56, 85.04, 279.16 GiB for G1SP20, G1CMCTWO, and G1SPFULL; their packed objective Jacobians require 10.59, 354.45, 1155.52 GiB. These are individual-array sizes, before workspaces and copies. Small inequality counts make the dual working system small, but do not remove this dense primal setup. The table records allocation failures rather than treating aborted attempts as fast solves; Dense Dual has no timing bars.

The new dual timings start with assembled native factors and include solver construction, ordering, objective factorization, working-set solves, and multiplier recovery and conversion. Loading, factor reconstruction, and independent certificate scoring are excluded. G1SP20 also has five additional fresh calls; Sparse Dual certifies 5/5 repeats, with a median of 3.694 s. Both dual backends use eight TBB threads on CPUs 0–7 and one numerical-library thread, a 300-second solver watchdog, and a 16 GiB worker limit enforced by an RSS monitor and an address-space cap. Allocation failures are caught and marked as memory limits.

The existing Primal, ALM, and external-solver measurements are retained with their original provenance. Their GTSAM times subtract the separately recorded native factor reconstruction from each call and repeat; other Python preparation remains included. COLAMD seeks to limit fill automatically. Chronological ordering eliminates state blocks in time order and retires shared variables at their last observing slice. Slice COLAMD uses COLAMD within chronological groups of original key blocks. Every ordering includes the complete variable set; no scalar rekeying is used.

The retained measurements used eight TBB threads on CPUs 0–7 and one numerical-library thread. Their original watchdog capped the total call at 300 seconds, including reconstruction; the plotted censored lower bounds subtract that reconstruction time. They are neither completed solves nor new 300-second runs from prepared factors. The second chart panel includes baseline Hessian-conversion time, shown separately because forming normal equations can change rounding. Dense Primal and DAQP remain excluded from G1.

| Capture: seconds | GTSAM ALM | GTSAM Sparse | GTSAM Chronological | GTSAM Slice COLAMD | GTSAM Sparse Dual | GTSAM Dense Dual | ProxQP | Clarabel | OSQP |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| G1SP20 | 42.099 (uncertified) | 7.123 | 8.826 | 8.720 | 3.697 | Memory limit (16 GiB) | 1.002 | 2.555 | 5.584 |
| G1CMCTWO | 11.287 | 164.498 | 121.616 | 120.943 | 25.504 | Memory limit (16 GiB) | ≥300.0 (censored) | 14.489 | 22.504 |
| G1SPFULL | ≥297.8 (censored) | ≥297.8 (censored) | ≥297.8 (censored) | ≥297.8 (censored) | ≥300.0 (censored) | Memory limit (16 GiB) | ≥300.0 (censored) | 42.293 | 101.755 |

```{iframe} /qp-plots/g1-times.html
:width: 100%
:class: qp-chart
:title: G1 timing comparison with uncertified and censored outcomes marked.

G1 timing comparison with uncertified and censored outcomes marked.
```

## Reproducibility

MPC uses qpbenchmark 2.7.0 and qpsolvers 4.13.0. Baseline versions and settings, build fingerprints, and dataset checksums are in the downloadable HTML report. No private G1 coefficients, solution vectors, benchmark scripts, or per-call measurement files are published.
