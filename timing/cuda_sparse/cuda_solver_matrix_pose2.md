# General CUDA solver matrix: Pose2 smoke benchmark

One warm-up and one measured run on `w10000.graph` (10,000 poses, 64,312
factors), NVIDIA A100 80 GB PCIe, CUDA 13.0, Release build. The build used the
correct `BetweenFactor` Jacobians. Initialization was the raw dataset, not
FastSync. CPU and GPU reached the same objective basin (CPU 162.033721791224).

| Configuration | GPU wall (s) | CPU/GPU | GPU objective | Analyze | Factor | Solve | PCG iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cuDSS auto | 1.111 | 3.83× | 162.033721791232 | 1 | 34 | 34 | 0 |
| cuDSS + GTSAM COLAMD | 1.533 | 2.78× | 162.033721791215 | 1 | 34 | 34 | 0 |
| PCG (`1e-10`, max 5000) | 3.474 | 1.26× | 162.033720946589 | 1 | 0 | 34 | 33,970 |

On this pose graph, cuDSS automatic ordering is the winner. Supplying GTSAM's
COLAMD ordering is correct but 38% slower end to end. Strict PCG converged in
every inner solve without reaching its cap, but was 3.13× slower than cuDSS
auto. The result does not contradict ordering wins on other sparsity patterns:
ordering is graph-dependent, which is why both policies are exposed and
benchmarked instead of hard-coding one. Each backend reuses one
analysis/session across every LM attempt.
