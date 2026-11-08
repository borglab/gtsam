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

## Dense Dual whitening: selected before/after comparison

Whitening eliminates repeated original-variable solves and improves several small QPs. Its cold-call benefit depends on how much working-set work repays the upfront constraint transformation; one-iteration and large unprojected cases can regress. The tables and charts include those regressions and failed original-input checks.

This selected campaign compares the exact pre-whitening library at 3ea2fe19f with the working-tree implementation on seven MPC cases. WALK0/WALK20 and BAL0/BAL10 cover small problems with different active-set work; QUAD2/3/4 cover larger equality-constrained problems. Both Dense equality settings are measured. Sparse Dual is measured in both libraries as a control; DAQP uses the installed, unmodified 0.8.7 wheel. These are fresh cold solves, with no caches shared between calls.

Times below are medians of eleven fresh calls at explicit 1e-6 accuracy, independently checked against the original input. Interactive controls also show 1e-9 and native Default stopping with a 1e-6 external check.

### Complete cold calls at 1e-6

| Problem | Before / project (ms) | Whitened / project (ms) | Before / keep E (ms) | Whitened / keep E (ms) | Sparse Dual (ms) | DAQP (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| LIPMWALK0 | 0.0945 | 0.0780 | 0.1022 | 0.0758 | 0.1393 | 0.0322 |
| LIPMWALK20 | 0.0957 | 0.0833 | 0.0979 | 0.0829 | 0.1466 | 0.0332 |
| WHLIPBAL0 | 0.2466 | 0.2239 | 0.2512 | 0.2287 | 0.2723 | 0.1319 |
| WHLIPBAL10 | 0.2078 | 0.2274 | 0.1919 | 0.2215 | 0.2158 | 0.0560 |
| QUADCMPC2 | 22.9379 | 24.4217 | 70.6264 | 104.2768 | 11.4075 | 12.2807 FAILED |
| QUADCMPC3 | 13.4886 | 14.9301 | 14.8807 | 23.8977 | 4.4703 | 2.2657 FAILED |
| QUADCMPC4 | 7.5324 | 7.8144 | 18.7764 | 29.6107 | 5.2019 | 3.5506 FAILED |

```{iframe} /qp-plots/dense-whitening-times.html
:width: 100%
:class: qp-chart
:title: Selected cold-call times with accuracy controls; crosses show failed DAQP checks.

Selected cold-call times with accuracy controls; crosses show failed DAQP checks.
```

The transformed loop approaches DAQP’s kernel cost, while complete solves still have substantial surrounding work. At 1e-6 with projection enabled, WALK20’s four working solves take 5.85 µs and its final recovery/check takes 2.94 µs; DAQP reports 2.09 µs for its native solve. BAL0 takes 8.13 µs for four working solves and 4.34 µs for recovery/check, versus DAQP’s 5.18 µs native solve. BAL0 additionally spends about 86.5 µs constructing the GTSAM solver and 31.4 µs transforming constraints. DAQP’s native solve timer also includes its final recovery; it does not expose that cost separately. These phase boundaries and GTSAM’s additional original-QP certification prevent treating the kernel figures as identical operations.

Transformation cost can outweigh the saved iterations. WHLIPBAL10 (project) 0.208 → 0.227 ms (1.09× time); QUADCMPC3 (project) 13.489 → 14.930 ms (1.11× time); QUADCMPC2 (keep E) 70.626 → 104.277 ms (1.48× time); QUADCMPC3 (keep E) 14.881 → 23.898 ms (1.61× time); QUADCMPC4 (keep E) 18.776 → 29.611 ms (1.58× time). BAL10 needs only one working solve, so transforming every row has little work to amortize. Unprojected QUAD must whiten the larger original dimension and factor its permanent equality working basis. Projection remains the safer performance choice in this selection, and Sparse Dual remains faster on the larger cases. Equality settings have the same numerical path on equality-free WALK/BAL; their timing differences include run-to-run variation and do not imply an equality-projection effect.

### Setup, iterations and recovery

Phase timings come from additional instrumented calls. Setup excludes solver construction and includes Dense workspace preparation, equality reduction/rank handling, curvature factors, row transformation and the permanent equality basis. The stacked chart separates equality handling and transformation. Whitened iteration time includes center-dependent RHS updates and warm-row changes. Recovery includes original-variable back-substitution, row checks and any conditional refinement; the original objective/KKT certificate and equality multiplier recovery are shown separately. The preserved binary cannot expose its old inner-loop recovery separately, so its table reports optimization and certificate together. Timer overhead and output conversion mean diagnostic phase sums should not be substituted for the paired complete-call timings. Cached-factor counters start after objective packing; construction can also validate or form a square root of native Hessians, and that work is included in the construction stage.

| Problem / equality setting | Before setup (µs) | Before optimization + certificate (µs) | Whitened setup (µs) | Whitened iterations (µs) | Whitened recovery (µs) |
| --- | --- | --- | --- | --- | --- |
| LIPMWALK0 / project | 10.5 | 40.4 | 11.9 | 5.2 | 2.1 |
| LIPMWALK0 / keep E | 11.1 | 41.6 | 11.6 | 4.7 | 2.1 |
| LIPMWALK20 / project | 14.5 | 48.4 | 13.6 | 5.9 | 2.9 |
| LIPMWALK20 / keep E | 10.7 | 40.4 | 12.5 | 5.1 | 2.2 |
| WHLIPBAL0 / project | 42.3 | 101.2 | 69.6 | 8.1 | 4.3 |
| WHLIPBAL0 / keep E | 42.4 | 95.3 | 71.1 | 8.1 | 4.6 |
| WHLIPBAL10 / project | 40.8 | 41.7 | 72.3 | 2.8 | 4.3 |
| WHLIPBAL10 / keep E | 43.7 | 42.9 | 65.9 | 2.6 | 4.0 |
| QUADCMPC2 / project | 18753.5 | 2947.7 | 19625.8 | 86.9 | 209.6 |
| QUADCMPC2 / keep E | 56184.0 | 13112.6 | 97393.1 | 1599.9 | 598.9 |
| QUADCMPC3 / project | 11932.8 | 952.0 | 13434.5 | 56.6 | 138.2 |
| QUADCMPC3 / keep E | 13015.9 | 1476.7 | 21740.9 | 141.4 | 222.9 |
| QUADCMPC4 / project | 6113.1 | 775.1 | 6339.0 | 26.5 | 97.1 |
| QUADCMPC4 / keep E | 17318.1 | 1724.7 | 27356.8 | 170.2 | 232.8 |

```{iframe} /qp-plots/dense-whitening-phases.html
:width: 100%
:class: qp-chart
:title: Dense whitening stage costs, including equality handling and row transformation.

Dense whitening stage costs, including equality handling and row transformation.
```

### Internal work counters

Whitening removes full primal solves from the ordinary Dense Dual inner loop. All 252 instrumented whitened calls use zero full primal solves, zero cached inverse responses, one cached Dense-workspace curvature factorization and one constraint transformation per optimization. WALK20 and BAL0 use four transformed working solves and one final original-variable recovery; QUAD uses two recoveries because its original-QP certificate requires proximal recentering. 12 conditional refinement steps occur across the diagnostic calls, with no legacy fallbacks in this selected set. Separate native tests exercise unavailable whitening and numerical refinement/fallback.

| Problem / equality setting | Working solves | Before full primal solves | Whitened full primal solves | Final recoveries | Cached curvature factors / transformations |
| --- | --- | --- | --- | --- | --- |
| LIPMWALK0 / project | 4 | 11 | 0 | 1 | 1 / 1 |
| LIPMWALK0 / keep E | 4 | 11 | 0 | 1 | 1 / 1 |
| LIPMWALK20 / project | 4 | 11 | 0 | 1 | 1 / 1 |
| LIPMWALK20 / keep E | 4 | 11 | 0 | 1 | 1 / 1 |
| WHLIPBAL0 / project | 4 | 11 | 0 | 1 | 1 / 1 |
| WHLIPBAL0 / keep E | 4 | 11 | 0 | 1 | 1 / 1 |
| WHLIPBAL10 / project | 1 | 2 | 0 | 1 | 1 / 1 |
| WHLIPBAL10 / keep E | 1 | 2 | 0 | 1 | 1 / 1 |
| QUADCMPC2 / project | 7 | 20 | 0 | 2 | 1 / 1 |
| QUADCMPC2 / keep E | 7 | 20 | 0 | 2 | 1 / 1 |
| QUADCMPC3 / project | 2 | 4 | 0 | 2 | 1 / 1 |
| QUADCMPC3 / keep E | 2 | 4 | 0 | 2 | 1 / 1 |
| QUADCMPC4 / project | 2 | 4 | 0 | 2 | 1 / 1 |
| QUADCMPC4 / keep E | 2 | 4 | 0 | 2 | 1 / 1 |

```{iframe} /qp-plots/dense-whitening-counts.html
:width: 100%
:class: qp-chart
:title: Working solves and original-variable recoveries before and after whitening.

Working solves and original-variable recoveries before and after whitening.
```

### Original-input failures

Every returned solution is rescored against the original matrices for primal feasibility, stationarity, nonnegative mathematical inequality multipliers and absolute duality gap. The selected campaign uses explicit 1e-6 and 1e-9 tolerances; Default keeps native stopping settings but is rescored at 1e-6. This stricter Default check differs from the historical full-suite profile below, whose published Default threshold is 1.0. No DAQP proximal settings are retuned. All 1,512 GTSAM calls and all 756 diagnostic calls pass. DAQP passes the four small cases and fails the original gap checks on all three larger cases at all three settings; its returned success flag is recorded but does not override those failures.

| DAQP case | Setting | Passing fresh calls | Worst primal residual | Worst stationarity residual | Worst absolute gap | Returned flags |
| --- | --- | --- | --- | --- | --- | --- |
| QUADCMPC2 | 1e-06 | 0/11 | 2.07e-12 | 7.04e-07 | 2e-05 | 1 |
| QUADCMPC3 | 1e-06 | 0/11 | 2.78e-17 | 1.11e-07 | 2.83e-06 | 1 |
| QUADCMPC4 | 1e-06 | 0/11 | 2.78e-17 | 4.14e-07 | 1.09e-05 | 1 |
| QUADCMPC2 | 1e-09 | 0/11 | 2.07e-12 | 7.04e-07 | 2e-05 | 1 |
| QUADCMPC3 | 1e-09 | 0/11 | 2.78e-17 | 1.11e-07 | 2.83e-06 | 1 |
| QUADCMPC4 | 1e-09 | 0/11 | 2.78e-17 | 4.14e-07 | 1.09e-05 | 1 |
| QUADCMPC2 | Default (check 1e-6) | 0/11 | 2.07e-12 | 7.04e-07 | 2e-05 | 1 |
| QUADCMPC3 | Default (check 1e-6) | 0/11 | 2.78e-17 | 1.11e-07 | 2.83e-06 | 1 |
| QUADCMPC4 | Default (check 1e-6) | 0/11 | 2.78e-17 | 4.14e-07 | 1.09e-05 | 1 |

### Numerical method and measurement protocol

Dense Dual now caches whitened constraint rows and performs working-set solves and feasibility scans in those coordinates. It uses M = A P R⁻¹ for the cached square-root factor and its variable permutation, normalizes rows, updates a working LDLᵀ factorization, and recovers original variables only for certification, proximal recentering or return. With equality projection enabled it works in the reduced affine space. With projection disabled, independent equality rows remain permanently active with unrestricted multipliers. Redundant equalities retain their original row identities for checks and multiplier recovery.

The numerical path follows Algorithm 1 in the DAQP paper and the transformed-point, violation-scan, immutable-row and LDLᵀ update structure in DAQP 0.8.7. It preserves GTSAM’s original-QP KKT certificate, value shapes, ordering, multiplier signs, warm starts, iteration budget and requested proximal weight. Numerical rank/conditioning guards, uncertain rays, or failed refinement retain the previous Dense Dual numerical path without changing formulation or adding a regularization floor. Sparse is unchanged.

The paired campaign contains 1,512 GTSAM calls: 21 problem/accuracy combinations, both libraries, two Dense equality settings plus Sparse, and one initial call plus eleven fresh repetitions. A separate 756-call diagnostic campaign uses one initial call plus five repetitions. DAQP has 252 calls, with the same twelve-call schedule per problem/accuracy. Runs are serial on CPU 0 with one numerical-library thread and one TBB thread; both GTSAM libraries and DAQP are interleaved in a randomized order with seed 749. GTSAM timing starts with assembled native factors and includes solver construction, packing, equality handling, transformation, optimization, certification, multiplier recovery and output conversion. DAQP starts with prepacked native arrays and includes its fresh setup and solve. Independent scoring is outside both clocks.

Algorithm and implementation references: [DAQP paper](https://cse.lab.imtlucca.it/~bemporad/publications/papers/ieeetac_daqp.pdf), [DAQP 0.8.7 source release](https://pypi.org/project/daqp/0.8.7/).

The existing 64-case MPC profiles, Cholesky before/after section and G1 results are retained historical measurements. Their Dense Dual series predates whitening. Only this selected section measures the new implementation; no G1 or unselected MPC results are presented as newly rerun.

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

GTSAM Sparse Dual and GTSAM Dense Dual select the new Dual formulation. They start with zero multipliers, bypass primal feasibility search and primal initialization, and use proximal weight 1e-9 on MPC and zero additional regularization on G1. Both enforce permanent equalities and update an inequality working LDLᵀ system. Dense uses native-Hessian Cholesky when eligible, otherwise equality-reduced square-root factors; Sparse applies a cached equality-constrained inverse without a global dense inverse or nullspace. Original-coordinate refinement preserves weak-direction accuracy, and all convergence certificates use the original QP. These additional entries are measured on both MPC and G1.

ProxQP uses a proximal primal-dual augmented Lagrangian method with semismooth Newton inner iterations. DAQP uses a dual active-set method with recursive LDLᵀ updates. Clarabel uses a primal-dual interior-point method. OSQP uses operator splitting, alternating linear-system solves with projections onto constraint bounds.

| Method | Optimization family | Configuration |
| --- | --- | --- |
| GTSAM ALM | Augmented Lagrangian | Native BCL penalty updates and Levenberg–Marquardt inner solves with a persistent nonlinear multifrontal workspace; zero initial values |
| GTSAM Sparse | Active set | Multivariate factor inputs; incremental Gaussian Bayes-tree working solves, cached RHS changes and one original-gradient residual correction; FeasibleStepThenProjectedGradient initialization and full original-QP optimality checks |
| GTSAM Chronological | Active set | The same sparse solver with chronological state blocks and shared-variable retirement; G1 only |
| GTSAM Slice COLAMD | Active set | Chronological slice groups, constrained COLAMD within each group of original key blocks; G1 only |
| GTSAM Dense | Active set | Permanent equality preparation shared across feasibility, componentwise initialization and the main solve; native-Hessian Cholesky for eligible positive-definite equality-free objectives, square-root/QR fallback, working LDLᵀ updates and reused primal/equality factors for multipliers; rank-aware null-space fallback; MPC only |
| GTSAM Sparse Dual | Dual active set | Zero-multiplier cold start; equality elimination through cached sparse constrained solves; working LDLᵀ updates and original-coordinate refinement; proximal weight 1e-9 on MPC, zero additional regularization on G1; MPC and G1 |
| GTSAM Dense Dual | Dual active set | Native positive-definite Hessian Cholesky when eligible; equality reduction and square-root fallback otherwise; LDLᵀ working updates, original-coordinate refinement and KKT checks; MPC and G1, with retained G1 timings |
| ProxQP | Proximal primal-dual augmented Lagrangian | Solves regularized subproblems with semismooth Newton inner iterations |
| DAQP | Dual active set | Recursive LDLᵀ updates; dense MPC only; version 0.8.7 uses selective proximal regularization for weak curvature |
| Clarabel | Interior point | Primal-dual interior-point steps toward the constrained optimum |
| OSQP | Operator splitting (ADMM) | Alternates linear-system solves and projections onto constraint bounds |

## Timing and accuracy

GTSAM timings begin with assembled factors, its native input. Translation from benchmark matrices into those factors is excluded and recorded separately. The measured interval includes solver construction, Dense matrix packing, ordering, feasibility search, initialization, optimization, multiplier recovery, and output conversion. Dense equality projection, numerical factorization and Sparse residual correction are included. The 10-second MPC watchdog starts at this same boundary. The four GTSAM primal/dual entries were rerun together for this update; ALM and external-solver measurements retain their previous provenance. Baseline timings retain their upstream matrix-input adapters and setup. This is a comparison from each library's input representation, rather than the cost of translating a common matrix format into every representation.

GTSAM's MPC factor graphs group related scalar unknowns into vector variables, each identified by one key. This representation preserves the QP dimension. WALK and BAL keep their condensed control sequence in one vector key with one objective factor. QUAD uses one 12-dimensional state key and four 3-dimensional contact-force keys per time step, with 12-row dynamics factors. All matrix coefficients, original constraint rows and dual mappings are preserved; no state variables are reintroduced. G1 already retains its original variable blocks and Jacobian factors.

MPC runs serially on CPU 0 with one numerical-library thread and one TBB thread. All inputs use cold starts. Five additional fresh-solver timings accompany each combination; they reuse no optimizer state. Default preserves native stopping rules and uses the upstream external threshold of 1.0. Low, mid, and high accuracy use absolute thresholds of 1e-3, 1e-6, and 1e-9. Default is therefore not an equal-accuracy comparison.

Independent checks evaluate primal feasibility, stationarity, and the primal–dual gap on the original unregularized QP. A supplemental dual-feasibility check verifies multiplier signs. A native success flag or a merely feasible step does not establish optimality. Numerical failures, iteration limits, failed certificates, and watchdog stops remain distinct. Failed calls remain in profile denominators and receive no speed reward.

## Dense Cholesky before and after

Dense now uses direct Cholesky for well-conditioned, positive-definite Hessian-native objectives without permanent equalities. It checks the original Hessian before adding the requested proximal term and preserves the original-QP feasibility, multiplier, stationarity and gap checks. Singular, poorly conditioned, mixed Jacobian/Hessian, and equality-constrained objectives retain the existing square-root/QR path. This is an internal selection; no new flag or regularization floor is introduced.

The before/after campaign contains 12,800 native calls: 6,144 per library version across both backends, both explicit formulations, all 64 MPC cases and four accuracy settings, plus 512 new Automatic checks. Each explicit combination has one initial call and five fresh repetitions. The two versions run in interleaved order on CPU 0 with one numerical and TBB thread. The table takes each case’s five-repeat median and then the family median; paired speedup is the geometric mean of per-case before/after ratios. No optimizer state or numerical factors carry between calls.

| Default · family median | Formulation | Before (ms) | After (ms) | Paired speedup |
| --- | --- | --- | --- | --- |
| LIPMWALK | Primal | 0.6088 | 0.5844 | 1.04× |
| LIPMWALK | Dual | 0.1257 | 0.0999 | 1.25× |
| WHLIPBAL | Primal | 0.3641 | 0.2483 | 1.44× |
| WHLIPBAL | Dual | 0.3215 | 0.1916 | 1.66× |

```{iframe} /qp-plots/dense-cholesky.html
:width: 100%
:class: qp-chart
:title: Complete Dense solve times before and after native-Hessian Cholesky.

Complete Dense solve times before and after native-Hessian Cholesky.
```

Both versions retain 253/256 certified primary cases per Primal backend and 256/256 per Dual backend. The same three low-accuracy WALK Primal initialization failures remain, with no new failing combinations. All 512 Automatic checks pass and select Dual. Sparse is a control, and QUAD is ineligible for this fast path; small timing changes there are not attributed to Cholesky. G1 and external solvers were not rerun.

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

A separate controlled sweep covers 108 seeded strictly convex QPs with 16, 64, or 128 variables, zero or half as many independent equalities, and inequality counts below, at, and above the reduced dimension. Each backend/formulation combination has three cold calls: all 1,944 calls pass the absolute 1e-7 original-input checks with zero proximal weight. That campaign used the earlier dimension-based Automatic rule. The earlier validation of the new policy passes all 648 Automatic calls (three calls per backend per problem), selecting Dual whenever inequalities are present and Primal otherwise. These synthetic objectives have at most two positive inequality multipliers, so their speed results favor small dual working sets and do not establish a universal crossover.

Automatic is now the default formulation, with Sparse as the default numerical backend. It selects Dual for cold starts with inequalities, independently of their count. A converged C++ warm start retains its previous formulation when the supplied point is feasible; nonconverged states or infeasible points select Dual. Values alone provide a cold proximal center. Equality-only and fully fixed problems use Primal, and singular reduced curvature without regularization falls back to Primal during setup. LPs resolve Automatic to Primal and reject explicit Dual. Selection remains fixed for the optimization call; both formulations can still be requested explicitly.

The dimension-only Automatic rule previously selected Primal on every MPC case. The present Dual-first policy passes all 512 new Automatic MPC checks, selecting Dual for both backends in all cold cases. The four plotted GTSAM timing series were refreshed for the Cholesky change; G1, ALM and external-solver measurements retain their earlier fingerprints. Sparse Dual already combines sparse objective factorization with a dense inequality working system. A small dual working set does not eliminate the Dense primal setup or the G1 memory limits.

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
| QUADCMPC1 | 0.1465 | 0.03705 | 0.08735 | 0.02571 | 0.05885 | 0.1142 | 0.03363 | 0.005422 | 0.00369 |
| QUADCMPC2 | 0.1408 | 0.02239 | 0.0335 | 0.01148 | 0.02312 | 0.04279 | 0.01277 | 0.003868 | 0.003888 |
| QUADCMPC3 | 0.01988 | 0.003134 | 0.01341 | 0.004534 | 0.01359 | 0.01279 | 0.002231 | 0.002113 | 0.00166 |
| QUADCMPC4 | 0.02171 | 0.008102 | 0.01049 | 0.005254 | 0.007369 | 0.01182 | 0.003719 | 0.002202 | 0.002219 |

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
