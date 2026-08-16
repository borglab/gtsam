# CUDA SFM solver matrix

One warm-up and one measured run on BAL `problem-16-22106-pre.txt`
(16 cameras, 22,106 points, 83,718 observations), NVIDIA A100 80 GB PCIe,
CUDA 13.0, Release build. The dense Schur row is the objective reference.

| Configuration | Representation | Wall (s) | Final objective | Analyze (s) | Factor (s) | Solve (s) | PCG iterations |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Schur + dense Cholesky | 144×144 dense | 0.026 | 18033.917684774 | 0 | 0.011 | 0.001 | 0 |
| Schur + cuDSS auto | 10,440-entry CSR | 0.135 | 18033.917684775 | 0.011 | 0.003 | 0.001 | 0 |
| Schur + cuDSS + GTSAM ordering | 10,440-entry CSR | 0.137 | 18033.917684771 | 0.011 | 0.003 | 0.001 | 0 |
| Schur + PCG | matrix-free | 0.532 | 18033.917684774 | 0 | 0 | 0.515 | 1,090 |
| Full normal + cuDSS auto | 2,393,742-entry CSR | 0.324 | 18033.917684773 | 0.276 | 0.011 | 0.004 | 0 |
| Full normal + cuDSS + GTSAM ordering | 2,393,742-entry CSR | 0.312 | 18033.917684774 | 0.263 | 0.013 | 0.004 | 0 |
| Full normal + PCG | matrix-free | 0.723 | 18033.917686206 | 0 | 0 | 0.698 | 2,130 |

Every row passed the automated objective check; both PCG rows also reported
convergence at relative residual tolerance `1e-10`. Both GTSAM-ordering rows
reported that the scalar permutation was applied by cuDSS.

This problem has only 16 cameras, so its reduced system is tiny and dense Schur
is the clear winner. The result still demonstrates the intended choice points:
cuDSS can solve either explicit system, while PCG consumes an operator without
building CSR. Matrix-free full-normal PCG transferred 2.65 MB host-to-device,
versus 12.50 MB for explicit full-normal cuDSS. Its camera/point block-Jacobi
preconditioner reduced the large-problem run from the 8,000-iteration cap with
no convergence to 2,130 converged iterations.

Persisting the Schur `U`, `V`, `W`, and gradient blocks also removed Jacobian
product recomputation from every damping attempt. Relative to the immediately
preceding run on the same machine, Schur PCG wall time fell from 0.970 s to
0.532 s (45%). The direct Schur rows moved by -1% to +7%, within the expected
noise/overhead range for this 144-scalar reduced system. Capturing and restoring
the explicit full-normal diagonal reduced its cuDSS rows by 3% and preserves
the same objective and analyze-once lifecycle.

The complete machine-readable records are under
`benchmark_logs/20260816_shared_solver_matrix_16_22106_persistent_final/`. A seven-row
smoke matrix for `dubrovnik-3-7-pre.txt` is under
`benchmark_logs/20260816_shared_solver_matrix_persistent_final/`.
