# Shared CUDA solver matrix smoke benchmark

Measured on an NVIDIA A100 80 GB PCIe with CUDA 13.0 and a Release build.
Dataset: `examples/Data/dubrovnik-3-7-pre.txt` (3 cameras, 7 points). Each row
was warmed once in its own process. This tiny problem is a correctness and
instrumentation smoke test, not a scalability claim.

| Configuration | Wall (ms) | Final objective | Analyze | Factor | Solve | PCG iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Schur dense | 5.28 | 0.020060046542 | 1 | 16 | 16 | 0 |
| Schur cuDSS auto | 22.74 | 0.020060046544 | 1 | 16 | 16 | 0 |
| Schur cuDSS GTSAM | 22.37 | 0.020060046544 | 1 | 16 | 16 | 0 |
| Schur PCG | 124.23 | 0.020060048828 | 1 | 0 | 16 | 2,980 |
| Full normal cuDSS auto | 18.20 | 0.020060046542 | 1 | 16 | 16 | 0 |
| Full normal cuDSS GTSAM | 18.09 | 0.020060046539 | 1 | 16 | 16 | 0 |
| Full normal PCG | 500.06 | 0.020060037211 | 1 | 0 | 16 | 14,310 |

All configurations reduced the same initial objective (2764.21998442218) to
the same solution basin. Both explicit cuDSS variants performed one symbolic
analysis and reused it for all 16 numerical attempts. The explicit GTSAM
ordering changes neither correctness nor cost meaningfully at this size; its
intended benefit must be evaluated on the large BAL datasets. Strict PCG
settings make both iterative rows converge to direct-solver accuracy, but the
tiny system strongly favors dense Schur. The raw machine-readable records are
in `results.jsonl`.
