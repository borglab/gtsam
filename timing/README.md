# Timing Benchmarks

This directory contains timing executables and helper scripts for GTSAM.

## RangeFactor Plaza2 Benchmark

This benchmark isolates the current range-only Plaza2 incremental SLAM workload
used by `RangeFactor<Pose2, Point2>`. It exists to support before/after
performance comparisons when changing the `RangeFactor` implementation.

### Build

From the build directory:

```bash
make -j6 timeRangeFactorPlaza2
```

### Run the benchmark executable

Run from `build/`:

```bash
./timing/timeRangeFactorPlaza2 --warmup 1 --repeats 5 \
  --output ../timing/results/range_factor_plaza2.csv
```

This prints aggregate timing statistics and writes one CSV row per measured run.

### Run the helper script

Run from the repository root with the `py312` conda environment:

```bash
conda run -n py312 python timing/benchmark_range_factor_plaza2.py \
  --build-dir build \
  --warmup 1 \
  --repeats 5 \
  --output timing/results/range_factor_plaza2.csv
```

The helper script runs the benchmark executable, preserves the CSV, and prints a
short summary that can be copied into a PR description.

## Bayes-Tree Covariance Results

The Bayes-tree covariance paper uses generated benchmark output rather than
checked-in CSV files. The files under `timing/results/` can be regenerated from
the commands below and do not need to be committed.

### Build

From the build directory:

```bash
make -j6 timeBayesTreeCovariance
make -j6 timeISAM2Covariance
make -j6 exportBayesTreeCovarianceVisuals
```

### Generate benchmark CSVs

Run from `build/`:

```bash
./timing/timeBayesTreeCovariance \
  --datasets w100.graph,w10000.graph,w20000.txt \
  --repeats 10 \
  --output-dir ../timing/results/bayes_tree_covariance
```

This writes:

- `timing/results/bayes_tree_covariance/raw.csv`
- `timing/results/bayes_tree_covariance/per_query.csv`
- `timing/results/bayes_tree_covariance/summary.csv`

The generated CSVs now include the small-query families used to benchmark the
legacy common cases:

- `single_pose` for `Q = 1`
- `pair_pose` for `Q = 2`

These are timed with the same benchmark executable and appear alongside the
larger `local_window`, `wide_separated`, `overlap_window`, and
`selected_cross` workloads.

The local query families are now sampled across the full valid trajectory range
rather than from only the earliest windows. This makes the `w10000` versus
`w20000` comparison reflect trajectory-wide local supports rather than a prefix
of the trajectory.

The fixed linearization point for these batch covariance timings is now obtained
from a sequential `ISAM2` solve rather than from a one-shot batch optimizer.
This avoids the poor large-loop initializations that can occur when the dataset
provides only odometric seed poses.

### Generate incremental `ISAM2` covariance CSV

Run from `build/`:

```bash
./timing/timeISAM2Covariance \
  --dataset w20000.txt \
  --query-repeats 5 \
  --output ../timing/results/bayes_tree_covariance/isam2_w20000.csv \
  --snapshot-dir ../timing/results/bayes_tree_covariance/isam2_support_snapshots
```

This writes:

- `timing/results/bayes_tree_covariance/isam2_w20000.csv`
- `timing/results/bayes_tree_covariance/isam2_support_snapshots/snapshots.csv`
- `timing/results/bayes_tree_covariance/isam2_support_snapshots/*.dot`

The incremental CSV contains one row per update step, recording both the
`ISAM2.update(...)` time and the repeated query time for the pairwise joint
covariance on `{x0, xt}`.
The snapshot directory stores compressed-support Graphviz views for five
representative incremental steps.

### Export `w100` visual data

Run from `build/`:

```bash
./timing/exportBayesTreeCovarianceVisuals \
  --dataset w100.graph \
  --output-dir ../timing/results/bayes_tree_covariance/visuals
```

This writes:

- the `w100` pose/query/covariance CSVs used for the geometric query figures
- `w10000_cliques.csv`
- `w20000_cliques.csv`

The latter two files store the final optimized trajectories with one clique-size value
per pose, so the report can compare the elimination geometry of the two larger datasets
on a shared color scale.

### Generate figures

Run from the repository root:

```bash
python3 timing/plot_bayes_tree_covariance.py \
  --input timing/results/bayes_tree_covariance/summary.csv \
  --per-query-input timing/results/bayes_tree_covariance/per_query.csv \
  --incremental-input timing/results/bayes_tree_covariance/isam2_w20000.csv \
  --incremental-snapshot-dir timing/results/bayes_tree_covariance/isam2_support_snapshots \
  --output-dir ../BayesTreeCovariance/figures/generated \
  --copy-csv-dir ../BayesTreeCovariance/data \
  --visual-data-dir timing/results/bayes_tree_covariance/visuals
```

This generates:

- `results-smallq.pdf`
- `results-ablation.pdf`
- `results-ordering.pdf`
- `results-structure.pdf`
- `results-local-diagnostics.pdf`
- `results-clique-sizes.pdf`
- `results-cross.pdf`
- `results-isam2-support.pdf`
- `results-isam2-w20000.pdf`
- `results-w100-queries.pdf`
- `results-w100-covariance.pdf`

## Notes

- The benchmark timings measure covariance-query work after obtaining a final
  estimate with sequential `ISAM2` and then linearizing once at that estimate.
- Each distinct query is run once as an untimed warmup and then `--repeats`
  times as measured repetitions.
- `raw.csv` stores one row per measured repetition.
- `per_query.csv` stores one row per distinct query, aggregating repeated
  timings by per-query means.
- `summary.csv` stores one row per query family bucket, aggregating the
  per-query means and structural statistics by medians across the sampled
  queries.
- The batch CSVs now also record support-width diagnostics:
  - `max_frontal_dim`
  - `max_separator_dim`
- `isam2_w20000.csv` stores one row per incremental update step for the
  sequential `ISAM2` experiment on `w20000`.
- The `results-smallq.pdf` figure summarizes the `Q = 1` and `Q = 2` query
  families; the larger performance figures focus on `Q > 2`, where Steiner
  localization changes the asymptotic behavior.
- The benchmark compares four variants:
  - `legacy_dense`
  - `steiner_dense`
  - `legacy_solve`
  - `steiner_solve`
- If the generated results become stale, it is safe to delete
  `timing/results/bayes_tree_covariance/` and regenerate it.
