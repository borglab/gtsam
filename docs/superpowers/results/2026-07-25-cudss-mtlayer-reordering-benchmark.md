# cuDSS Multithreaded Reordering (P1a) Benchmark

## Outcome: negative result — the threading layer does not help these workloads

Enabling cuDSS's host threading layer (`libcudss_mtlayer_gomp.so.0`,
`CUDSS_CONFIG_HOST_NTHREADS = 30`) produced **no meaningful change** in
end-to-end GPU wall time on any of the four workloads. The reason, found by
splitting `CUDSS_PHASE_ANALYSIS` into its two sub-phases, is that the
assumption behind P1a was wrong for these systems: **analysis time is
dominated by symbolic factorization, not reordering**, and the threading
layer parallelizes only the reordering.

### Analysis-phase split (1 run, A100, `GTSAM_CUDSS_SPLIT_ANALYSIS=1`)

| Workload | Reordering (1 thread) | Reordering (30 threads) | Symbolic factorization |
| --- | ---: | ---: | ---: |
| Dubrovnik-135 | 0.580 s | 0.521 s | 1.24–1.25 s (unchanged) |
| Dubrovnik-16 | 0.107 s | 0.124 s | 0.178–0.180 s (unchanged) |

Even for Dubrovnik-135 the 30-thread reordering saved only ~0.06 s of a
3.5 s run; for Dubrovnik-16 it was slightly slower. cuDSS 0.8's log confirms
the layer was active ("Using 30 threads on host for the reordering") and the
reordering algorithm was `CUDSS_REORDERING_ALG_NESTED_DISSECTION`.

### Five-run medians, baseline vs threading layer

| Workload | GPU wall (baseline) | GPU wall (mtlayer) | cudssAnalysis (baseline) | cudssAnalysis (mtlayer) |
| --- | ---: | ---: | ---: | ---: |
| bal16 | 0.663 s | 0.689 s | 0.269 s | 0.269 s |
| bal135 | 3.559 s | 3.619 s | 1.811 s | 1.831 s |
| pose2 | 0.367 s | 0.344 s | 0.040 s | 0.033 s |
| pose3 | 0.731 s | 0.704 s | 0.037 s | 0.032 s |

Differences are within run-to-run noise (baseline stddev on bal135 was
~0.11 s in the 2026-07-23 benchmark). All runs passed the harness's
objective-parity validation (max diff ≤ 8.7e-8 relative to multi-million
objectives).

## Consequences for the optimization plan

1. **P1a is not worth pursuing further on A100-class hosts for these
   problem sizes.** The parallelizable fraction (reordering) is ~30% of
   analysis and it barely speeds up at this scale.
2. **P1b (user-supplied permutation) has a hard ceiling of ~0.5 s on
   bal135** — it can only remove the reordering sub-phase. The symbolic
   factorization (~1.25 s) remains.
3. The symbolic-factorization cost is a function of the full 273k×273k,
   30.7M-nnz normal system. The ways to remove it are structural:
   **P1c (Schur elimination of landmarks — hand cuDSS a ~1.2k×1.2k system)**,
   **P3 (reuse the analyzed solver across same-topology solves, e.g. GNC)**,
   or **P2 (PCG, which has no analysis phase at all)**.

## Code changes (this worktree, uncommitted)

- `CudssThreadingOptions` (`threadingLayer`, `hostThreads`) added to
  `gtsam/nonlinear/cuda/CudssLinearSolver.h`; applied in
  `CudssSpdSolver::Impl::analyze` via `cudssSetThreadingLayer` +
  `CUDSS_CONFIG_HOST_NTHREADS`. Default off (empty layer = old behavior).
- Plumbed through `DeviceSparseJacobianNormalEquations::analyze` and
  `CudaSparseLevenbergMarquardtParams`
  (`cudssThreadingLayer`, `cudssHostThreads`).
- `timeCudaSparseLM` gained `--cudss-threading-layer LIB` and
  `--cudss-host-threads N`.
- Env-gated diagnostic `GTSAM_CUDSS_SPLIT_ANALYSIS=1` times
  `CUDSS_PHASE_REORDERING` and `CUDSS_PHASE_SYMBOLIC_FACTORIZATION`
  separately and prints one stderr line per analysis.

## Environment

Same host and build as the 2026-07-23 benchmark (A100 80GB PCIe, cuDSS
0.8.0.10, CUDA 13.0, 32-core EPYC-Milan, Release build). NVIDIA kernel
modules were reloaded before the runs to fix a driver/userspace version
mismatch (580.159.03 module vs 580.173.02 userspace).

Commands:

```text
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --json timing/cuda_sparse/results/2026-07-25-mtlayer/baseline.json \
  --csv  timing/cuda_sparse/results/2026-07-25-mtlayer/baseline.csv

# plus the same with:
#   --cudss-threading-layer /usr/lib/x86_64-linux-gnu/libcudss/13/libcudss_mtlayer_gomp.so.0 \
#   --cudss-host-threads 30
# writing mtlayer.json / mtlayer.csv
```

Artifacts: `timing/cuda_sparse/results/2026-07-25-mtlayer/`
(baseline/mtlayer JSON, CSV, and console logs).
