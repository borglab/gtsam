# General CUDA solver matrix: Pose2 smoke benchmark

One warm-up and one measured run on `w10000.graph` (10,000 poses, 64,312
factors), NVIDIA A100 80 GB PCIe, CUDA 13.0, Release build. The build used the
correct `BetweenFactor` Jacobians. Initialization was the raw dataset, not
FastSync. CPU and GPU reached the same objective basin (CPU 162.033721791224).

| Configuration | GPU wall (s) | CPU/GPU | GPU objective | Analyze | Factor | Solve | PCG iterations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cuDSS auto | 1.127 | 3.95× | 162.033721791213 | 1 | 34 | 34 | 0 |
| cuDSS + GTSAM COLAMD | 1.568 | 2.79× | 162.033721791208 | 1 | 34 | 34 | 0 |
| PCG (`1e-10`, max 1000) | 1.948 | 2.31× | 162.069600223272 | 1 | 0 | 33 | 13,100 |

On this pose graph, cuDSS automatic ordering is the winner. Supplying GTSAM's
COLAMD ordering is correct but 39% slower end to end, and strict PCG is 73%
slower than cuDSS auto. The result does not contradict ordering wins on other
sparsity patterns: ordering is graph-dependent, which is why both policies are
exposed and benchmarked instead of hard-coding one. Each backend reuses one
analysis/session across every LM attempt.
