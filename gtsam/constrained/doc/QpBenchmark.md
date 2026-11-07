---
title: QP solver benchmarks
downloads:
  - file: qp-benchmark-report.html
    title: Standalone HTML report with graphics and measurements
  - file: qp-benchmark-results.json
    title: Measurements, settings, versions, and input fingerprints
---

# QP solver benchmarks

This benchmark compares quadratic-programming solvers on robotic control
and trajectory-optimization problems. A quadratic program (QP) minimizes
a quadratic cost subject to linear equations and limits; its variables
can represent future motion or contact forces. The comparison measures
solve time and checks the accuracy of the returned solutions independently.

The input collection combines the public
[qpbenchmark model predictive control test set](https://qpsolvers.github.io/qpbenchmark/mpc_qpbenchmark/)
and three larger QPs captured from a G1 humanoid application. MPC compares
**GTSAM Sparse, GTSAM Dense, GTSAM ALM, DAQP, Clarabel, OSQP, and ProxQP**.
Sparse retains its proximal regularization and uses incremental factor updates
and cached RHS solves. G1 compares
the sparse-capable solvers, GTSAM ALM, and an additional GTSAM chronological-ordering variant;
**GTSAM Dense and DAQP are not run on G1**.
The GTSAM active-set entries use `FeasibleStepThenProjectedGradient` initialization
and continue to full optimization. GTSAM ALM uses its native augmented
Lagrangian procedure from zero, with BCL penalty updates and LM inner solves. The [QP user guide](QpProblem.ipynb)
explains those API choices.

Download the self-contained report, including graphics and an offline
filterable results table: {download}`qp-benchmark-report.html`.
The report embeds its scalar measurements and works without a network
connection. The download panel also provides JSON. The standalone report generates MPC
and problem-size CSV downloads directly from its embedded measurements; CSV
files are not stored in the repository. Hover over the Plotly charts to inspect
values, click legends to compare methods, and drag to zoom. The complete report
works offline; the individual website chart embeds load a pinned Plotly runtime
from its CDN.

## Solver methods

GTSAM Sparse, Dense and Chronological use an **active-set algorithm**.
They solve with a current set of binding constraints and add or release
rows as optimization proceeds. Sparse uses incremental factor-graph
elimination and cached RHS solves; Dense selects dense working-system
solves; Chronological changes the sparse variable-elimination order on G1.
GTSAM ALM is a separate augmented Lagrangian optimizer with penalty and
multiplier updates and Levenberg–Marquardt inner solves.

| Method | Optimization family | Configuration in this comparison |
| --- | --- | --- |
| GTSAM Sparse | Active set | Incremental factor-graph solves with COLAMD and cached RHS solves; proximal weight `1e-9` on MPC, zero additional regularization on G1 |
| GTSAM Chronological | Active set | The same sparse solver with chronological state blocks and shared-variable retirement; G1 only |
| GTSAM Dense | Active set | Dense linear solves for the current working constraints; MPC only |
| GTSAM ALM | Augmented Lagrangian | Native BCL updates and LM inner solves; zero initial values; retained native damping and stopping rules |
| DAQP | Dual active set | Recursive LDLᵀ updates; dense MPC only; version 0.8.7 uses selective proximal regularization for weak curvature |
| Clarabel | Interior point | Primal-dual interior-point steps toward the constrained optimum |
| OSQP | Operator splitting, using ADMM | Alternates linear-system solves and projections onto constraint bounds |
| ProxQP | Proximal primal-dual augmented Lagrangian | Solves regularized subproblems with semismooth Newton inner iterations |

The baseline algorithms are documented by the
[DAQP authors](https://cse.lab.imtlucca.it/~bemporad/publications/papers/ieeetac_daqp.pdf),
[Clarabel](https://clarabel.org/),
[OSQP](https://osqp.org/docs/solver/), and the
[ProxQP authors](https://scaron.info/publications/tro-2025.html).
ProxQP's authors also describe its
[semismooth Newton inner iterations](https://github.com/Simple-Robotics/proxsuite/discussions/222).
ADMM stands for alternating direction method of multipliers.
Iteration counts describe different operations in these methods; elapsed
time and independently checked solutions provide the common comparison.

## Benchmark datasets

Model predictive control (MPC) plans over a short future horizon, applies
the first action, and then plans again. Its test problems are snapshots
of that planning calculation. Each snapshot here is solved from a cold
start, so the timings do not measure reuse of a previous controller solution.

- **LIPMWALK: humanoid walking.** A linear inverted-pendulum model simplifies
  balance during walking. The dataset contains 30 small QPs with 16 variables.
- **WHLIPBAL: balancing on wheels.** A wheeled inverted-pendulum model captures
  a body balancing above its wheels while tracking motion. There are 30 QPs
  with 50 variables.
- **QUADCMPC: quadruped centroidal control.** Four gait scenarios plan the
  overall motion of a four-legged robot using foot-contact forces. These
  larger QPs include explicit equations and bounds; their sparse quadratic
  costs leave some directions unpenalized.
- **G1: global humanoid trajectory QPs.** These are captured GTDynamics
  subproblems, separate from the public MPC collection. They couple many
  motion samples and contact variables. The three captures use different
  motion windows or sampling, described below.

The MPC family descriptions follow the
[pinned dataset generators](https://github.com/qpsolvers/mpc_qpbenchmark/tree/954906ac1f48b77e84a5bc314b7c2f00fb0d4ebc/generate).

### Variables and constraints

Counts refer to the original input, before fixed-bound normalization.
LIPMWALK and WHLIPBAL have uniform sizes within each family; each QUAD and
G1 capture is listed individually.

<!-- qp-sizes:start -->

| Dataset | Cases | Variables | Equality rows | Inequality rows | Finite bound sides | Fixed variables |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| LIPMWALK | 30 | 16 | 0 | 32 | 0 | 0 |
| WHLIPBAL | 30 | 50 | 0 | 100 | 0 | 0 |
| QUADCMPC1 | 1 | 768 | 384 | 512 | 512 | 192 |
| QUADCMPC2 | 1 | 576 | 288 | 384 | 384 | 144 |
| QUADCMPC3 | 1 | 384 | 192 | 256 | 128 | 0 |
| QUADCMPC4 | 1 | 384 | 192 | 256 | 256 | 96 |
| G1SP20 | 1 | 18,549 | 0 | 230 | 0 | 0 |
| G1CMCTWO | 1 | 106,836 | 0 | 378 | 0 | 0 |
| G1SPFULL | 1 | 193,566 | 0 | 2,437 | 0 | 0 |

<!-- qp-sizes:end -->

Equality rows require an equation to hold; inequality rows impose a
one-sided limit. “Finite bound sides” counts each finite lower or upper
bound, so a fixed variable contributes two sides. Fixed variables are a
subset of those bounds, not extra constraints. G1 has no explicit equality
rows because those equations were already eliminated into its reduced
coordinates. Least-squares residual rows belong to the objective and are
not counted as constraints.

## Results on all 64 MPC problems

Every solver entry runs at the upstream default, low, mid, and high accuracy
settings, with a 10-second call watchdog. Five additional fresh-solver
timings follow each initially successful combination.
The upstream default setting keeps each solver's native stopping criteria
and scores primal, stationarity, and gap against `1.0`; it is not an
equal-accuracy comparison. Low, mid, and high use common absolute thresholds
of `1e-3`, `1e-6`, and `1e-9`.

```{iframe} /qp-plots/mpc-success.html
:width: 100%
:class: qp-chart
:title: Counts passing the published MPC checks and the supplemental dual-sign check at each accuracy setting.

Success counts under the published primal, stationarity, and gap checks
(left), and with the additional dual-feasibility check (right).
```

<!-- qp-summary:start -->

| Solver | Published checks | Also dual feasibility |
| --- | ---: | ---: |
| GTSAM Sparse | 253/256 | 253/256 |
| GTSAM Dense | 253/256 | 253/256 |
| GTSAM ALM | 178/256 | 178/256 |
| DAQP | 248/256 | 248/256 |
| Clarabel | 256/256 | 256/256 |
| OSQP | 247/256 | 247/256 |
| ProxQP | 253/256 | 235/256 |

Sparse and Dense counts come from fresh complete MPC sweeps. ALM and
baseline measurements retain their recorded results. Failures remain
in the summaries and performance-profile denominators. The comparison
below records robustness and timing changes against saved implementations.

<!-- qp-summary:end -->

```{iframe} /qp-plots/mpc-profile.html
:width: 100%
:class: qp-chart
:title: Per-setting performance profiles showing the fraction of 64 problems solved within a runtime ratio of the fastest certified solver.

Performance profiles use the official first cold-call timings. Failed
certificates have infinite ratios and remain in the denominator. The HTML
table separately reports the median and range of five additional timings.
```

At a horizontal value of **2**, a curve's height is the fraction of problems
the solver passed within **twice the fastest passing time** for each problem.
Higher and further left is better. A curve ending below 1 did not pass every
case. Read speed together with reliability: a fast failed call never counts
as a fast solution.

<!-- qp-sparse-refresh:start -->

### Current Sparse timing comparison

The current Sparse solver was retimed after the constrained-QR improvement. The campaign uses the freshly built library directly, without diagnostic hooks or library overrides. All 64 MPC problems run at four accuracy settings on CPU 0, with one TBB/numerical-library thread, a 10-second watchdog, and five additional fresh timings after each primary success. The initializer remains FeasibleStepThenProjectedGradient and the proximal weight remains 1e-9.

| MPC family | Current passes | Current / prior Sparse repeat-time ratio |
| --- | --- | --- |
| LIPMWALK | 117/120 | 0.990 |
| WHLIPBAL | 120/120 | 1.001 |
| QUADCMPC | 16/16 | 0.932 |

Ratios compare median fresh timings for matching successful cases against the preceding Sparse report. Values below one mean faster. The 253/256 success set and main iteration counts are unchanged; all 1,265 additional MPC calls pass. These comparisons include setup and initialization, so kernel speedups do not translate directly into equal end-to-end gains.

| Default setting | Prior Sparse median (s) | Current Sparse median (s) | Current range (s) |
| --- | --- | --- | --- |
| QUADCMPC1 | 0.1178 | 0.1096 | 0.1095–0.1099 |
| QUADCMPC2 | 0.0641 | 0.0604 | 0.0601–0.0608 |
| QUADCMPC3 | 0.0122 | 0.0114 | 0.0112–0.0115 |
| QUADCMPC4 | 0.0244 | 0.0229 | 0.0229–0.0231 |

All three G1 captures were also rerun for COLAMD, chronological ordering, and chronological slices with COLAMD inside each slice. They retain their original Jacobian factors and captured damping, zero additional regularization, eight TBB threads on CPUs 0–7, one numerical-library thread, and an explicit 300-second diagnostic watchdog. There is one primary attempt per larger capture and five additional fresh timings for each certified G1SP20 ordering. Dense and all baseline measurements are preserved.

| Capture / ordering | Prior first call (s) | Current first call (s) | Current certificate |
| --- | --- | --- | --- |
| G1SP20 / GTSAM Sparse | 11.904 | 7.329 | certified |
| G1SP20 / GTSAM Chronological | 12.135 | 9.033 | certified |
| G1SP20 / GTSAM Slice COLAMD | 11.978 | 8.928 | certified |
| G1CMCTWO / GTSAM Sparse | 182.378 | 165.703 | certified |
| G1CMCTWO / GTSAM Chronological | 136.282 | 122.811 | certified |
| G1CMCTWO / GTSAM Slice COLAMD | 134.940 | 122.147 | certified |
| G1SPFULL / GTSAM Sparse | 300 s limit | 300 s limit | watchdog_timeout |
| G1SPFULL / GTSAM Chronological | 300 s limit | 300 s limit | watchdog_timeout |
| G1SPFULL / GTSAM Slice COLAMD | 300 s limit | 300 s limit | watchdog_timeout |

<!-- qp-sparse-refresh:end -->

### Incremental sparse solves with proximal regularization

GTSAM Sparse retains its `1e-9` proximal weight and reuses the working-system
factorization. Adding or releasing constraints updates only the affected
Bayes-tree region. Changing the proximal center with fixed curvature and
working constraints uses cached triangular tree solves, with **zero numerical
eliminations**. Initialization, objective, regularization values, tolerances,
ordering, blocking steps, multiplier recovery and original-QP certificates
remain unchanged. `useIncrementalSparseQp = false` retains batch elimination.

`GaussianISAM2` provides a general objective-forcing query in the linear layer.
It has no knowledge of QPs or proximal regularization. Its registered original
factors and cached separator factors retain a fixed reference RHS. The query
changes neither them nor the conditional factorization. `ActiveSetSolver`
computes the forcing change as the proximal weight times the displacement
from its reference center. Later working-set updates therefore reconstruct
a consistent graph after any sequence of RHS changes.

<!-- qp-proximal-summary:start -->

| Family | Previous batch, rho=1e-9 | Previous zero regularization | Current incremental, rho=1e-9 | Current/batch repeat-time ratio | Current/previous-zero repeat-time ratio | Matched batch passes | Lost / gained vs batch |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LIPMWALK | 117/120 | 117/120 | 117/120 | 1.01 | 1.035 | 117 | 0 / 0 |
| WHLIPBAL | 120/120 | 120/120 | 120/120 | 0.959 | 1.106 | 120 | 0 / 0 |
| QUADCMPC | 16/16 | 0/16 | 16/16 | 0.7784 | — | 16 | 0 / 0 |

<!-- qp-proximal-summary:end -->

The table compares this fresh sweep against saved implementations. Only the
current Sparse implementation appears in profiles and the measurements table;
full historical measurements and the original library remain local.
The paired time ratio compares five-run medians for identical problem/accuracy
combinations that pass in both current and previous batch runs, then takes
the median ratio within each family. Values below 1 mean faster. Lost and
gained passes are reported independently so speed cannot conceal a regression.
The previous-zero ratio includes only passing cases and changes regularization
as well as implementation; it does not isolate the effect of factorization reuse.

The current implementation preserves **253/256** passes, including **16/16
QUAD** combinations. It retains the same three low-accuracy LIPMWALK feasibility
initialization failures. Matched five-run medians are about 17% faster on QUAD,
5% faster on WHLIPBAL, and 2% slower on LIPMWALK. These are measured family
medians, not an assumption that incremental elimination always wins.
For the largest case, QUADCMPC1 at default settings, the five-call median
decreases from 173.36 ms to 116.87 ms.

The previous zero-regularization experiment passed 237/256, failing all 16
QUAD combinations with indeterminate working systems. The current approach
keeps proximal regularization while enabling incremental elimination; it does
not remove that numerical support. Native regression tests verify zero
elimination callbacks on RHS-only queries, batch agreement with affine exact
constraints, and reference-cache consistency across repeated RHS/working-set
changes, including dependent rows and weak weighted directions.

### QUADCMPC results

The full MPC profiles contain 60 small problems and only four larger
quadruped problems. These separate profiles focus on those four cases.
Each step represents one problem (25%), so individual cases strongly affect
the curves.

```{iframe} /qp-plots/quad-profile.html
:width: 100%
:class: qp-chart
:title: Performance profiles restricted to the four QUADCMPC problems, at each accuracy setting, for all seven MPC solver entries.

QUAD-only performance profiles retain failed checks and timeouts in the
four-problem denominator.
```

```{iframe} /qp-plots/quad-times.html
:width: 100%
:class: qp-chart
:title: Default-setting cold-call runtimes for the four quadruped centroidal-control QPs and seven solver entries, with failed calls marked.

QUADCMPC default-setting calls. Hatched bars did not pass; a limit marker
indicates a stopped call rather than a completed solve. The downloadable
table includes every accuracy setting, residual, and failure reason.
```

All GTSAM MPC entries use fixed-bound-to-equality normalization and restore
the original dual ordering. Every score uses the original unregularized
input. Dense selects the same backend as `QpSolverType::Dense`; active-set state is
retrieved through `ActiveSetSolver` to expose multipliers for scoring.
Cost-free Dense variables are declared with exact zero-cost factors.

Both active-set backends use `regularization = 1e-9` as a recentered
proximal weight in their working subproblems and certify the original QP.
Dense no longer applies a permanent penalty or increases regularization
through a hidden floor. Both backends share feasibility search,
initialization, blocking and row-release decisions, and convergence checks.
G1 retains zero additional regularization and its captured damping.

### Dense implementation stages

Dense and Sparse now pass **253/256** MPC combinations, including **16/16
QUAD** combinations. The remaining failures are the same three low-accuracy
LIPMWALK feasibility failures. The saved Dense implementation passed 179/256;
shared solver behavior raises this to 241/256, and the dense rank-aware
reference raises it to 253/256. Factorization reuse preserves that robustness.
Each implementation stage was followed immediately by QUAD and full-MPC
sweeps at all four settings, with five extra cold timings per primary success.
Historical results and binaries remain local.

Dense packs contiguous objective and constraint blocks once, preserving
original Jacobian square roots. Its null-space QR reference handles dependent
boundaries, including the 832-row QUAD working set. Every original selected
row remains enforced and retains its multiplier identity. A numerical basis
represents redundant boundaries; it does not remove those constraints.

Dense reuse caches objective QR and transforms constraint normals lazily.
Independent additions extend working QR; removals use Givens rotations.
Releasing a basis row while redundant rows remain triggers a controlled
basis rebuild. Singular objectives retain a cached null-space fallback.
RHS-only changes use cached solves and perform zero numerical factorizations,
verified with native counters. The mutable workspace belongs to one solve call.

```{iframe} /qp-plots/dense-stages.html
:width: 100%
:class: qp-chart
:title: Default QUAD timing medians and ranges for shared semantics, dense QR reference, dense reuse, Sparse, and DAQP.

Five additional fresh-solver calls per passing default-setting combination.
Bars show medians and ranges on a logarithmic time axis. Missing bars failed
the original-input certificate. Setup and initialization are included.
```

| Default setting | Dense reference | Dense reuse | Sparse |
| --- | ---: | ---: | ---: |
| QUADCMPC1 | 2.175 s | 0.583 s | 0.110 s |
| QUADCMPC2 | 0.324 s | 0.158 s | 0.060 s |
| QUADCMPC3 | 0.039 s | 0.030 s | 0.011 s |
| QUADCMPC4 | 0.051 s | 0.050 s | 0.023 s |

Reuse reduces repeated work but Sparse remains faster on these four cases.
QUADCMPC1 makes 19 working solves with one objective QR and three working-basis
rebuilds; two rebuilds exchange a released basis row while redundant rows
remain. The downloadable report includes counters, reasons, setup/solve
breakdowns, and all accuracy settings. Local diagnostic writes are included
in the timed calls; instrumentation is excluded from the reviewed source.

### DAQP regularization and accuracy

DAQP is a **dual active-set solver using recursive LDLᵀ updates**. It updates
its factorization as constraints enter or leave the working set. This
comparison pins [DAQP 0.8.7](https://pypi.org/project/daqp/0.8.7/), matching
the published benchmark, and uses the unchanged `qpsolvers==4.13.0` adapter.
DAQP runs on all 64 MPC problems and is excluded from G1.

DAQP uses proximal iterations to handle QUAD's singular Hessians. The
versioned source adds `eps_prox` to diagonal values or Cholesky pivots at or
below `zero_tol`, records those directions, and recenters their linear terms
on subsequent solves. Healthy curvature directions retain their original
values. The QUAD Hessians are diagonal, so their zero diagonal entries
receive proximal regularization of `1e-6` in this experiment.

<!-- qp-daqp-settings:start -->

| Setting | primal_tol | dual_tol | eps_prox | eta_prox |
| --- | ---: | ---: | ---: | ---: |
| default | 1e-06 | 1e-12 | 1e-06 | 1e-06 |
| low_accuracy | 0.001 | 0.001 | 1e-06 | 1e-06 |
| mid_accuracy | 1e-06 | 1e-06 | 1e-06 | 1e-06 |
| high_accuracy | 1e-09 | 1e-09 | 1e-06 | 1e-06 |

<!-- qp-daqp-settings:end -->

These are the actual configured settings, read from the installed 0.8.7
interface and checked against its published source archive. The upstream
accuracy settings change only `primal_tol` and `dual_tol`; `eps_prox` and
`eta_prox` remain `1e-6` at every setting. Other retained defaults include
`progress_tol = 1e-14`, `iter_limit = 10000`, and native `time_limit = 0`.
The external 10-second watchdog supplies the deadline. Complete stopping
settings, source and binary checksums, and settings-audit details are in
the downloadable provenance.

<!-- qp-daqp-results:start -->

| Setting | All MPC /64 | Dense-Hessian cases /60 | QUAD /4 |
| --- | ---: | ---: | ---: |
| default | 64 | 60 | 4 |
| low_accuracy | 64 | 60 | 4 |
| mid_accuracy | 60 | 60 | 0 |
| high_accuracy | 60 | 60 | 0 |

<!-- qp-daqp-results:end -->

DAQP passes **248/256** combinations: all 240 dense-Hessian combinations and
QUAD at default and low accuracy. All four QUAD cases return native success
at mid and high accuracy but miss the original-input gap check; high
accuracy also misses stationarity. The proximal and stagnation stopping
rules remain unchanged as primal and dual tolerances tighten. The report
retains these eight failed certificates without tuning the defaults.

DAQP is **not running without regularization on QUAD**. A separate untimed
audit with `eps_prox = 0` rejects all four QUAD inputs during setup with
native exit flag `-5`. Those probes are excluded from benchmark timings and
performance profiles. With the benchmark settings, 24 audited calls match
the official solution and multiplier arrays exactly. The audit covers one
problem from each small family and all four QUAD cases at all four settings.
The source archive and diagnostic results remain local; scalar settings,
checksums, and outcomes are recorded in the report.

Published-style success and supplemental dual feasibility are distinct
columns. Approximate multipliers can pass primal, stationarity, and gap
checks while missing a sign tolerance. These outcomes remain in the data;
they are not reported as native solver exceptions. Native ProxQP backend
selection is preserved: dense for these dense MPC arrays, sparse for the
G1 CSC inputs.

<!-- qp-alm-results:start -->

### GTSAM augmented Lagrangian results

GTSAM ALM uses native BCL multiplier/penalty updates and LM inner solves,
with a persistent nonlinear multifrontal workspace. It starts at zero;
it is separate from the active-set solver and its initialization options.

| Accuracy | Native convergence /64 | Independent pass /64 | QUAD pass /4 | Inner stalls |
| --- | ---: | ---: | ---: | ---: |
| default | 64 | 64 | 4 | 0 |
| low_accuracy | 64 | 41 | 3 | 0 |
| mid_accuracy | 62 | 53 | 0 | 2 |
| high_accuracy | 25 | 20 | 0 | 39 |

Default keeps every native parameter unchanged. Other settings change only
absolute violation and stationarity tolerances. ALM retains 20 outer iterations,
100 LM iterations per inner solve, initial penalty 10 with growth 100,
initial LM damping 1e-5, and LM relative/absolute error tolerances 1e-5.
No additional objective regularization, backend fallback, or tolerance tuning
is introduced. Native LM damping remains part of ALM itself.

ALM passes **178/256** combinations: 41 stop with an unconverged inner solve;
37 return native convergence but miss the independent original-input checks.
No MPC call times out or throws a numerical exception. Returned candidates
and native multipliers are checked unchanged: 35 of these miss the gap and
two QUADCMPC3 calls miss stationarity with the returned multipliers. ALM tests
the augmented gradient and feasibility, but does not test the gap. A BCL
penalty update can retain the previous multipliers; their original-QP
stationarity is then distinct from the augmented-gradient check. No
multiplier repair is applied. Five additional fresh calls
follow each initially passing combination. Status, penalties, residuals,
effective settings and iteration counts are in the HTML and JSON.

G1 uses 1e-7 outer tolerances, eight CPUs, and a 300-second diagnostic budget.
The native BCL/LM settings and original captured damping are retained.

| Capture | Native status | Certified | Setup (s) | Solve (s) | Outer / total LM iterations | Failure / check |
| --- | --- | --- | ---: | ---: | --- | --- |
| G1SP20 | converged | no | 0.2176 | 42.08 | 6 / 16 | absolute_gap |
| G1CMCTWO | converged | yes | 1.259 | 11.22 | 4 / 9 |  |
| G1SPFULL | watchdog_timeout | no | 2.272 | — | — / — | Explicit 300-second solve watchdog |

G1SP20 returns native convergence in 42.302 s but misses the independent
absolute gap check (2.00e-7 versus 1e-7), despite stationarity 3.41e-9 and
primal violation 3.76e-10. It receives no extra timings because it did not
certify. G1CMCTWO certifies in 12.477 s, including 1.259 s of setup.
G1SPFULL reaches the 300-second watchdog without a returned candidate;
this is a timeout, not a reported numerical or iteration failure.

<!-- qp-alm-results:end -->

## Captured G1 kinodynamic QPs

These fixed QPs come from a humanoid motion application, after one accepted
nonlinear step. The second global QP was captured because the first has no
inequality rows.

- **G1SP20** selects 171 contact-support slices from a 1,000-sample,
  78–98 s motion clip.
- **G1CMCTWO** uses all 1,000 samples of that clip and is a distinct,
  more densely sampled QP.
- **G1SPFULL** selects 1,780 slices from a 0–219.94 s recording.

The motion windows are not solver runtime limits. No full trajectory is
run for this comparison, and Dense and DAQP are excluded from G1.

The two GTSAM active-set G1 entries use the same solver with different
variable-elimination orderings. An elimination ordering determines which
variable blocks are solved out first and changes the sizes of intermediate
systems. The original Sparse entry uses COLAMD, an automatic ordering that
seeks to keep intermediate matrices sparse.
Chronological eliminates state blocks in increasing time and retires each
shared contact variable after its last observing slice; the IMU bias stays
through its last use. The ordering includes every potential constraint,
is verified as a complete key permutation, and is set through the existing
`setOrdering()` option. Ordering construction is included in setup time.

Both GTSAM active-set entries use **`FeasibleStepThenProjectedGradient`**, incremental
sparse elimination, and **zero additional regularization**. They retain
the same factors, zero initial values, tolerances, thread settings, and
watchdog. Multiplier recovery keeps its separate internal ordering.

| Capture | Variables | Least-squares residual rows | Inequality rows |
| --- | ---: | ---: | ---: |
| G1SP20 | 18,549 | 58,051 | 230 |
| G1CMCTWO | 106,836 | 338,455 | 378 |
| G1SPFULL | 193,566 | 607,666 | 2,437 |

These are fixed captured QPs with their original LM damping. No trajectory
is run. GTSAM reconstructs the original Jacobian factors and diagonal
sigmas. The other solvers receive sparse normal-equation Hessians;
conversion time is reported separately and also included in the second
timing panel. All certification uses the original weighted Jacobian and
inequality rows, with the objective constant restored.

```{iframe} /qp-plots/g1-times.html
:width: 100%
:class: qp-chart
:title: Setup-and-solve and conversion-inclusive runtimes for each captured G1 QP, with uncertified or censored outcomes marked.

G1 timing boundaries. Hatched bars did not certify the original-input QP;
limit markers indicate censored calls rather than completed solves.
```

<!-- qp-g1-summary:start -->

| Capture | GTSAM Sparse | GTSAM Chronological | GTSAM Slice COLAMD | Clarabel | OSQP | ProxQP | GTSAM ALM |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| G1SP20 | 7.329 s | 9.033 s | 8.928 s | 2.555 s | 5.584 s | 1.002 s | 42.302 s (checks failed) |
| G1CMCTWO | 165.703 s | 122.811 s | 122.147 s | 14.489 s | 22.504 s | 300 s limit | 12.477 s |
| G1SPFULL | 300 s limit | 300 s limit | 300 s limit | 42.293 s | 101.755 s | 300 s limit | 300 s limit |

These are first cold-call setup-and-solve times, including initialization.
The three GTSAM active-set ordering variants were rerun with the current
library; baseline and ALM measurements retain their recorded campaigns.
The larger captures have one primary attempt per method. Completed
active-set calls are checked against the original inputs; watchdog limits
remain explicit rather than being reported as completed solves.

The five additional G1SP20 calls have median times of 7.283 s, 9.059 s, 8.974 s, 2.548 s, 5.407 s, 1.002 s,
in the method order excluding ALM. Ranges, objectives, residuals, and iteration counts
are in the downloadable report. Baseline Hessian conversion adds about
0.18 s, 1.06–1.07 s, and 1.83–1.84 s for the three captures.

<!-- qp-g1-summary:end -->

The common G1 certificate requires absolute primal feasibility,
stationarity, dual feasibility, complementarity, and gap no greater than
`1e-7`. Every solver has an explicit 300-second diagnostic call budget.
Preparation has a separate excluded 180-second budget; worker RSS is
limited to 16 GiB. Eight CPUs are available; realized threading and
CPU-seconds are reported because the solvers have different native
parallel implementations. Five additional timings are collected on
certified G1SP20 combinations; the larger captures have one primary
attempt per solver.

G1 coefficients, recordings, and solution vectors remain local. The
downloads publish numeric outcomes and input fingerprints. They do not
provide an independent reference optimum or release the captured data.

## Reproducibility and interpretation

The report records the GTSAM revision, MPC dataset commit and file hashes,
solver versions, backend selection, settings, CPU, affinity, thread
budgets, construction and solve timing boundaries, and numerical
residuals. MPC uses the pinned upstream settings from `qpbenchmark 2.7.0`
and `qpsolvers 4.13.0`; all entries run on the same host. The current Sparse campaign uses a rebuilt Release/TBB library and the same
MPC inputs, initializer, regularization, thread settings and timing boundaries
as the saved batch/proximal campaign. Dense is retimed after each implementation
stage. The current refresh reruns Sparse on MPC and all three active-set
G1 orderings; Dense, ALM and baseline entries retain their recorded campaigns
and historical library/adapter checksums. Absolute
timings from the upstream website's different hardware are not reused.

Timing includes initialization and solver setup. Input loading and sparse
Hessian conversion are excluded from the solver-call measurement;
conversion-inclusive G1 times make that additional work visible. The
first MPC GTSAM call includes native-module import. Subsequent fresh-solver
timings keep loaded modules and filesystem caches, but reuse no solver
state or supplied warm start.

The supplemental certificate does not weaken native full-optimization
requirements. A feasible step is never counted as a converged QP solve.
Watchdog limits, iteration exhaustion, numerical failures, resource limits,
native convergence, and checked certificates are separate outcomes. The
results describe these inputs and settings; initialization strategies
retain workload-dependent costs.
