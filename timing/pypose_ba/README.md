# PyPose/BAE BAL Timing Harness

This folder is an optional comparison harness for `timing/timeSFMBAL.cpp`.
It is intentionally kept outside the C++ build: PyPose/BAE are Python/PyTorch
dependencies, and the standalone `bae` package may need a CUDA toolkit to build
its extensions.

## Environment

Create the environment from the repository root:

```bash
timing/pypose_ba/setup_env.sh
```

Activate it:

```bash
conda activate pypose-bae
```

The setup script installs PyTorch with a CUDA wheel, PyPose `>=0.9.5`, CUDA
`nvcc`, NVIDIA cuDSS, and `bae==0.2` from `sair-lab/bae`. The BAE build uses
native CUDA extensions, so it expects a CUDA-capable Linux machine and a host
compiler supported by the CUDA toolkit. Override `CC`, `CXX`, or
`CUDAHOSTCXX` before running the script if your default compiler is too new for
CUDA.

## Quick Run

Build GTSAM's timing executable first:

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_TIMING_ALWAYS=ON
cmake --build build --target timeSFMBAL -j
```

Run PyPose sparse LM on the small checked-in BAL file and include GTSAM output:

```bash
python timing/pypose_ba/time_bal.py \
  --bal-file examples/Data/dubrovnik-3-7-pre.txt \
  --backend pypose-sparse \
  --warmup-steps 1 \
  --steps 20 \
  --gtsam-exe build/timing/timeSFMBAL
```

For the large workflow dataset used by the existing `timeSFMBAL` benchmark:

```bash
curl --fail --location --show-error \
  --output examples/Data/problem-135-90642-pre.txt.bz2 \
  https://grail.cs.washington.edu/projects/bal/data/dubrovnik/problem-135-90642-pre.txt.bz2
bzip2 -dc examples/Data/problem-135-90642-pre.txt.bz2 \
  > examples/Data/dubrovnik-135-90642-pre.txt

python timing/pypose_ba/time_bal.py \
  --bal-file examples/Data/dubrovnik-135-90642-pre.txt \
  --backend pypose-sparse \
  --warmup-steps 1 \
  --steps 20 \
  --gtsam-exe build/timing/timeSFMBAL
```

## Full Benchmark

`run_benchmark.py` drives `time_bal.py` over the three Dubrovnik datasets used
by `timeSFMBAL` (16-22106, 88-64298, 135-90642), repeats the PyPose/BAE solve
for a stable min/median wall time, runs the GTSAM executable once per dataset,
and prints a Markdown table. Download the datasets first (see above), then:

```bash
python timing/pypose_ba/run_benchmark.py \
  --backend pypose-sparse \
  --repeats 3 \
  --steps 50 \
  --output-json timing/pypose_ba/benchmark-pypose.json
```

Both sides use GTSAM's relative-decrease stopping rule (`--steps` is the cap),
so the reported errors converge to the same level and the timings reflect equal
optimization work. Example results on a machine with an NVIDIA A100 80GB GPU
and a 32-core CPU (PyPose/BAE on the GPU; GTSAM on the CPU, built with TBB and
observed peaking near 9 cores — it is not single-threaded) — these are
solver-family numbers, not a like-for-like replacement, because the linear
solvers differ (PCG vs. direct multifrontal) and the hardware differs:

| Dataset | Obs | PyPose iters | PyPose s (min) | GTSAM Cholesky s | GTSAM Solver s | rel.err.diff |
| --- | --- | --- | --- | --- | --- | --- |
| dubrovnik-16-22106 | 83718 | 6 | 0.60 | 2.3 | 1.2 | 0.00% |
| dubrovnik-88-64298 | 383937 | 4 | 0.62 | 7.5 | 5.2 | 0.28% |
| dubrovnik-135-90642 | 553336 | 3 | 0.59 | 7.8 | 7.4 | 1.35% |

The growing error difference on the larger problems comes from the looser 1%
relative-decrease tolerance stopping after very few outer iterations: each side
takes a slightly different last step. Tighten `--relative-decrease-tol` and
raise `--steps` to drive both to a common, lower error if you need closer
final-error agreement.

## Notes

- This is not part of the official GTSAM timing target.
- `timeSFMBAL` drops tracks observed by fewer than two cameras. This harness
  applies the same observation filter before running PyPose/BAE.
- GTSAM's `timeSFMBAL` reports nonlinear error as `0.5 * ||r||^2`. This harness
  reports both raw sum-of-squares and `gtsam_style_error` values.
- When `--gtsam-exe` is given, the harness parses GTSAM's initial error and the
  `Final error: ..., iterations: ...` lines from `timeSFMBAL` stdout and emits
  `initial_error_comparison` / `final_error_comparison` blocks so timings can
  be normalized by the error level each side actually reached.
- `--initial-damping` (default `1e-4`) sets the TrustRegion initial damping to
  match GTSAM's Ceres-style `lambdaInitial`. Both apply multiplicative diagonal
  damping, so the first LM step solves a comparable system.
- The JSON records thread and device environment (`torch` thread counts,
  `OMP_NUM_THREADS`, CUDA device) since these materially affect both sides.
- GTSAM here is built with TBB (`GTSAM_WITH_TBB=ON`) and parallelizes factor
  linearization and the elimination across cores; `timeSFMBAL` does not cap the
  thread count, so it uses as many cores as TBB schedules (observed ~9 of 32 on
  the reference machine). It is not a single-core baseline. The recorded
  `OMP_NUM_THREADS`/`torch` thread counts describe the PyTorch side, not GTSAM's
  TBB pool.
- GTSAM's `timeSFMBAL` runs Levenberg-Marquardt to its configured termination
  criteria with direct multifrontal linear solvers. By default the PyPose/BAE
  side now applies the same stopping rule (`--relative-decrease-tol 0.01`, the
  value `timeSFMBAL` passes to `setRelativeErrorTol`), with `--steps` as the
  iteration cap, so both sides do comparable optimization work. Pass a negative
  tolerance to revert to a fixed number of LM steps. The linear solvers still
  differ (direct multifrontal vs. PCG), so treat timings as solver-family
  comparisons rather than apples-to-apples replacement timings.
- Use `--warmup-steps` for CUDA/Warp setup before timing. Warmup uses a separate
  model so the timed solve still starts from the BAL initialization.
- Raw BAL observations use the BAL/OpenGL convention. GTSAM converts them to
  its camera convention internally; this harness stays in the raw BAL convention
  to match the PyPose/BAE example projection. Squared residual norms are
  equivalent, but residual vectors differ by the convention transform.
- Per-step losses are not recorded by default. With the convergence check
  enabled the scalar loss is already copied to CPU each step (GTSAM also
  evaluates its error every iteration), so `--record-losses` adds no extra
  synchronization; in fixed-step mode (negative tolerance) it does.
