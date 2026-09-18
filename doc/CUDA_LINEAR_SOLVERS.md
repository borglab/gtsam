# CUDA linear solvers

CUDA optimizers prepare a linear system and hand it to
`gtsam::cuda::LinearSolverSession`. The session owns reusable solver state,
validates the selected backend, and reports backend-independent statistics.

## Public API

The installed CUDA solver API consists of:

- `LinearSolverSession`, `LinearSolverOptions`, `PcgOptions`, and solver stats;
- `SparseLevenbergMarquardtOptimizer` for supported general factor graphs; and
- `SfmLevenbergMarquardtOptimizer` for resident BAL-style bundle adjustment.

Assembly plans, CUDA kernels, Schur operators, and concrete solver
implementations are internal details, live under `cuda/internal/`, and are not
installed as public headers.

Walkthrough notebooks for each optimizer:

- [SparseLevenbergMarquardtOptimizer](../gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb)
- [CudaSfmLevenbergMarquardtOptimizer](../gtsam/sfm/doc/CudaSfmLevenbergMarquardtOptimizer.ipynb)
- [CudaSfmGncOptimizer](../gtsam/sfm/doc/CudaSfmGncOptimizer.ipynb)

## Backends

The general sparse optimizer supports cuDSS and matrix-free PCG. The SFM
optimizer eliminates points with a Schur complement and supports:

| Backend | Reduced system |
|---|---|
| `DenseCholesky` | dense camera system solved with cuSOLVER |
| `Cudss` | sparse camera system with reusable symbolic analysis |
| `Pcg` | implicit Schur operator with block-Jacobi preconditioning |

The SFM default is `DenseCholesky`. The general optimizer defaults to cuDSS.
cuDSS requires `GTSAM_ENABLE_CUDSS` and the separate install described below;
the other backends require only CUDA.

## Installing cuDSS

cuDSS is NVIDIA's direct sparse solver. It is **not** part of the CUDA toolkit
and GTSAM does not bundle it, so `GTSAM_ENABLE_CUDSS=ON` needs it installed
first. It is optional: `GTSAM_ENABLE_CUDSS` defaults to `OFF`, and the dense
Cholesky and PCG backends need nothing beyond CUDA itself.

**GTSAM requires cuDSS 0.8.0 or newer**, which is where `CUDSS_R_64F`,
`cudssReorderingAlg_t`, and `CUDSS_STATUS_IR_FAILED` arrived. An older cuDSS
fails during configuration, naming the version it found. Configuring reports what
was selected:

```
-- GTSAM cuDSS: 0.8.0 (/usr/lib/x86_64-linux-gnu/libcudss/13/libcudss.so)
```

Some distributions still serve 0.7.x, and an active conda or virtual environment
is searched before the system install, so check that line rather than assuming.

Install it from [developer.nvidia.com/cudss](https://developer.nvidia.com/cudss):

```bash
# Debian and Ubuntu, once NVIDIA's CUDA repository is configured
sudo apt-get install libcudss0-cuda-13 libcudss0-dev-cuda-13

# conda
conda install -c conda-forge libcudss-dev

# tarball unpacked anywhere
cmake -S . -B build-cuda -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=ON \
  -DCUDSS_ROOT=/opt/nvidia/libcudss
```

`CUDSS_ROOT` is also how to override an install that the search would otherwise
find first. The pip wheels (`nvidia-cudss-cu12`, `nvidia-cudss-cu13`) carry only
the versioned runtime libraries without a development symlink, so they can run
against a prebuilt GTSAM but cannot build one.

Projects consuming an installed GTSAM locate cuDSS the same way through
`GTSAMConfig.cmake`, so `CUDSS_ROOT` applies to them too.

## Ordering

Set the inherited optional `Ordering` when using cuDSS. The frontend validates
that every required key appears once, expands key ordering to scalar indices,
and supplies that permutation during symbolic analysis. SFM ordering contains
camera keys because its linear system is the reduced camera system. Ordering is
rejected for dense and PCG backends instead of being silently ignored.

## Lifecycle

Each optimizer creates one solver session. A stable sparse pattern is analyzed
once, while factorization and solve run for each LM damping attempt. PCG keeps
its allocations across attempts and can warm-start until the frontend
invalidates the state. `LinearSolveStats` records analyses, solves, convergence,
iteration counts, transfer costs, and backend timings.

## C++ selection

```cpp
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
#include <gtsam/slam/cuda/SfmLevenbergMarquardt.h>

gtsam::cuda::SparseLevenbergMarquardtParams sparseParams;
sparseParams.linear.backend = gtsam::cuda::LinearSolverType::Pcg;
sparseParams.pcg.relativeTolerance = 1e-6;

gtsam::cuda::SfmLevenbergMarquardtParams sfmParams;
sfmParams.setLinearSolver(gtsam::cuda::LinearSolverType::Cudss);
sfmParams.ordering = cameraOrdering;
```

## Python bindings

CUDA-enabled Python builds expose both optimizers under `gtsam.cuda`. The
namespace is intentionally absent when GTSAM is built without
`GTSAM_ENABLE_CUDA=ON`; see the [Python wrapper README](../python/README.md) for
source-build instructions.

The general optimizer accepts ordinary `gtsam.NonlinearFactorGraph` and
`gtsam.Values` objects as `graph` and `initial`:

```python
import gtsam

cuda = gtsam.cuda

linear = cuda.LinearSolverOptions()
linear.backend = cuda.LinearSolverType.Pcg

pcg = cuda.PcgOptions()
pcg.maxIterations = 250
pcg.relativeTolerance = 1e-6

params = cuda.SparseLevenbergMarquardtParams()
params.linear = linear
params.pcg = pcg
params.fallbackOnUnsupported = False

optimizer = cuda.SparseLevenbergMarquardtOptimizer(graph, initial, params)
values = optimizer.optimize()
diagnostics = optimizer.result()
print(diagnostics.backend, diagnostics.finalError)
```

Setting `fallbackOnUnsupported = False` is useful when execution must stay on
the GPU. The default permits the optimizer to continue with CPU LM when the
runtime or graph is unsupported; `diagnostics.fallbackReason` and
`diagnostics.fallbackDetail` report why. The
[General CUDA LM notebook](../gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb)
contains a complete runnable graph.

For CUDA SFM, pass a `gtsam.SfmData` as `data`:

```python
params = cuda.SfmLevenbergMarquardtParams.ceresDefaults()
params.setLinearSolver(cuda.LinearSolverType.DenseCholesky)

result = cuda.optimizeSfm(data, params)
print(result.finalError, result.iterations)
values = result.optimizedValues
```

`DenseCholesky` is available only to CUDA SFM. General CUDA LM accepts `Pcg`
or `Cudss`; selecting `Cudss` requires a build with
`GTSAM_ENABLE_CUDSS=ON` and a separate cuDSS installation.

## Build and test

The kernels accumulate into double-precision buffers with `atomicAdd`, so they
require compute capability 6.0 or newer. `GTSAM_ENABLE_CUDA=ON` therefore
defaults `CMAKE_CUDA_ARCHITECTURES` to `native`, which builds only for the GPUs
present in the configuring machine; name the architectures explicitly when
producing binaries for other machines, for example
`-DCMAKE_CUDA_ARCHITECTURES=80;120`. Configuring below 6.0 is a hard error
rather than a compile failure inside the kernels.

A toolkit older than the GPU cannot build for it — CUDA 12.0 rejects
`compute_120` even where the driver supports Blackwell — and CMake by itself
takes whichever `nvcc` comes first on `PATH`. When `CMAKE_CUDA_COMPILER` and
`CUDACXX` are both unset, the configuration therefore tries `CUDAToolkit_ROOT`,
`CUDA_HOME`, `CUDA_PATH`, `/usr/local/cuda`, `PATH`, and the newest
`/usr/local/cuda-*` in turn, and takes the first `nvcc` that compiles for the
requested architectures. Configuring reports the choice:

```
-- GTSAM CUDA compiler: /usr/local/cuda-13.0/bin/nvcc
-- GTSAM CUDA architectures: native
```

A compiler that cannot target the requested architectures fails during
configuration, naming the alternatives that were tried, instead of failing part
way through the build.

A source file is named `.cu` if and only if it contains device code: a
`__global__` or `__device__` function, or a `<<<>>>` launch. Everything else is
`.cpp` and is built by the host compiler, including files that are wall-to-wall
CUDA library calls — `cudss*`, `cusolver*`, `cusparse*`, `cudaMalloc*`, and
`cudaEvent*` are ordinary host functions and need no `nvcc`. Following the rule
keeps files out of the slower compile path; `CudssSpdSolver.cpp` is the clearest
example. Sources are globbed, so renaming a file across the boundary needs no
CMake change.

```bash
cmake -S . -B build-cuda -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=ON
cmake --build build-cuda -j6 --target testCudaLinearSolver \
  testCudaSparseJacobian testCudaSparseLevenbergMarquardt testCudaSfm
ctest --test-dir build-cuda --output-on-failure \
  -R 'testCudaLinearSolver|testCudaSparseJacobian|testCudaSparseLevenbergMarquardt|testCudaSfm'
```

The timing executables retain production comparisons and machine-readable
output without embedding benchmark results in the source tree:

```bash
./build-cuda/timing/cuda_sparse/timeCudaSparseLM --help
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --list-configurations
./build-cuda/timing/sfm_ba/timeCudaSFMBAL --cuda-lm \
  --configuration schur-cudss-gtsam --output-format json problem.txt
```
