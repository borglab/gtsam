# CUDA SFM solver matrix

One warm-up and one measured run on BAL `problem-16-22106-pre.txt`
(16 cameras, 22,106 points, 83,718 observations), NVIDIA A100 80 GB PCIe,
CUDA 13.0, Release build. The dense Schur row is the objective reference.

| Configuration | Representation | Wall (s) | Final objective | Analyze (s) | Factor (s) | Solve (s) | PCG iterations |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Schur + dense Cholesky | 144×144 dense | 0.025 | 18033.917684772 | 0 | 0.010 | 0.001 | 0 |
| Schur + cuDSS auto | 10,440-entry CSR | 0.136 | 18033.917684770 | 0.011 | 0.003 | 0.001 | 0 |
| Schur + cuDSS + GTSAM ordering | 10,440-entry CSR | 0.134 | 18033.917684775 | 0.011 | 0.003 | 0.001 | 0 |
| Schur + PCG | matrix-free | 0.970 | 18033.917684773 | 0 | 0 | 0.951 | 1,070 |
| Full normal + cuDSS auto | 2,393,742-entry CSR | 0.333 | 18033.917684772 | 0.280 | 0.011 | 0.004 | 0 |
| Full normal + cuDSS + GTSAM ordering | 2,393,742-entry CSR | 0.322 | 18033.917684773 | 0.268 | 0.013 | 0.004 | 0 |
| Full normal + PCG | matrix-free | 0.723 | 18033.917686214 | 0 | 0 | 0.699 | 2,130 |

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

The complete machine-readable records are under
`benchmark_logs/20260816_shared_solver_matrix_16_22106_final/`. A seven-row
smoke matrix for `dubrovnik-3-7-pre.txt` is under
`benchmark_logs/20260816_shared_solver_matrix_final/`.
