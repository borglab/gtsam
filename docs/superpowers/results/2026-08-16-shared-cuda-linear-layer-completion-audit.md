# Shared CUDA linear layer completion audit

Date: 2026-08-16

This audit maps the complete implementation plan to code, regression tests,
and release evidence. The final ownership boundary is a prepared linear system:
the general and SFM frontends produce systems and retain LM semantics, while
`gtsam/linear/cuda` owns backend dispatch and numerical solves.

## Requirement inventory

| # | Requirement | Implementation and proof |
| ---: | --- | --- |
| 1 | Common contracts and capability validation | `CudaLinearSystem.h`, `CudaLinearSolver.h/.cpp`; `CudaLinearSolver.CapabilityMatrix`, representation/order rejection tests. |
| 2 | GTSAM key ordering to scalar permutation | `CudaBlockOrdering.h/.cpp`; valid, duplicate, incomplete, unknown-key, and invalid-layout tests. |
| 3 | Shared sparse SPD storage | `DeviceSparseSpdSystem.h/.cu`; diagonal capture/restore tests. Compatibility aliases remain in `gtsam/nonlinear/cuda`. |
| 4 | Shared dense Cholesky | `CudaDenseCholeskySolver.h/.cu`; two-by-two solve test and dense SFM parity tests. |
| 5 | Shared cuDSS with user permutation | `CudssSpdSolver.h/.cpp` sets `CUDSS_DATA_USER_PERM`, disables additional reordering, reads the result back, and verifies it; ordered-solve tests pass. |
| 6 | Formulation-independent PCG recurrence | `CudaPcgSolver.h/.cu` consumes operator/preconditioner interfaces; generic operator and warm-start invalidation tests pass. |
| 7 | Common session dispatch and statistics | `CudaLinearSolverSession` dispatches dense, sparse, and operator systems and reports `CudaLinearSolveStats`; all three dispatch tests pass. |
| 8 | General CUDA LM on shared layer | `CudaSparseLevenbergMarquardt.cpp` owns a shared session for cuDSS/PCG and compiles optional ordering; general trajectory, fallback, PCG, cuDSS, and ordering regressions pass. |
| 9 | Persistent SFM Schur producer | `CudaSfmSchurProblem.h/.cu` builds projection linearization and undamped `U`, `V`, `W`, `gc`, and `gp` once per outer iteration. Tests poison the source residual/Jacobian arrays and prove dense assembly and point recovery use the retained blocks. |
| 10 | Stable reduced-camera CSR plan | `CudaSfmReducedCsrPlan.h/.cpp`; deterministic camera-only co-visibility structure and offset tests pass. |
| 11 | Sparse Schur through cuDSS | `CudaSfmSparseSchur.h/.cu` scatters retained blocks into stable upper CSR. A poisoned-Jacobian test matches dense Schur; auto and GTSAM-ordered optimizer tests pass. |
| 12 | Implicit Schur and camera-block PCG | `CudaSfmSchurOperator.h/.cu` applies `U-WV^-1W^T` and builds the 9-by-9 camera preconditioner from retained blocks. Operator/RHS parity, recovered-point, convergence, and objective tests pass. |
| 13 | Full normal through common core | `CudaSfmFullNormalProblem.h/.cu` owns both explicit CSR and matrix-free forms. Explicit mode accumulates once, captures the undamped diagonal, and restores it per retry; cuDSS and PCG use shared sessions. |
| 14 | Independent formulation/backend parameters and LM semantics | `CudaSfmLevenbergMarquardtParams` inherits GTSAM LM parameters and separates `formulation` from `linear.backend`; capability and compatibility-alias tests pass. |
| 15 | Common stats and frontend profiles | Both optimizers export common solve statistics. Full-normal construction is attributed once per outer iteration; damping remains per attempt. The profiling reconciliation test passes. |
| 16 | Complete benchmark matrices | `timeCudaSparseLM`, `timeCudaSFMBAL`, and `run_cuda_sfm_bal_benchmarks.sh` expose stable configurations and correctness-gated JSON/CSV records. |
| 17 | Operational documentation | `docs/CUDA_LINEAR_SOLVERS.md` documents ownership, capability matrix, ordering semantics, lifecycle, statistics, build/test commands, and benchmark commands. |
| 18 | Release verification and audit | Evidence is recorded below and in `timing/sfm_ba/cuda_solver_matrix_bal.md`. |

## Final verification

| Gate | Result |
| --- | --- |
| CUDA + cuDSS build | `testCudaLinearSolver`, `testCudaSparseJacobian`, `testCudaSparseLevenbergMarquardt`, `testCudaSfm`, `timeCudaSparseLM`, and `timeCudaSFMBAL` built successfully. |
| CUDA + cuDSS tests | 4/4 selected CTest suites passed. |
| CUDA without cuDSS build/tests | 3/3 selected CTest suites passed; dense Schur, Schur PCG, and full-normal PCG ran; requesting cuDSS failed explicitly. |
| CPU-only build | `gtsam` Release target built successfully. |
| Compute Sanitizer memcheck | `testCudaLinearSolver` and `testCudaSfm`: zero errors. |
| Compute Sanitizer racecheck | cuDSS-disabled `testCudaSfm`: zero project-kernel hazards. The cuDSS-enabled run reports races only inside NVIDIA `cudss::*` kernels, not project code. |
| Documentation contract | All required architecture, backend, ordering, and benchmark terms are present. |
| Diff hygiene | `git diff --check` passes. |
| Independent review | No Critical or Important issue. Two Minor findings (outer-iteration timing attribution and signed large-input bounds) were fixed and reverified. |

## Benchmark result

On BAL `problem-16-22106-pre.txt` (16 cameras, 22,106 points, 83,718
observations), every one of the seven configurations passed its objective and
metadata gate. Wall times were 0.026 s dense Schur, 0.135/0.137 s Schur cuDSS
auto/GTSAM, 0.532 s Schur PCG, 0.324/0.312 s full-normal cuDSS auto/GTSAM,
and 0.723 s full-normal PCG. Persisting Schur normal blocks reduced Schur PCG
from 0.970 s to 0.532 s (45%) on the same machine. Complete records are in
`timing/sfm_ba/benchmark_logs/20260816_shared_solver_matrix_16_22106_persistent_final/results.jsonl`.
