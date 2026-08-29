# General CUDA LM Python Binding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose the practical configuration, status, result, and optimizer API for general CUDA Levenberg-Marquardt under `gtsam.cuda`.

**Architecture:** Put shared solver declarations in `linear/cuda`, general optimizer declarations in `nonlinear/cuda`, and retain SFM declarations in `sfm/cuda`. CMake concatenates them in dependency order for CUDA-enabled Python builds, producing one shared `gtsam.cuda.LinearSolverType` registration.

**Tech Stack:** C++17, CUDA, GTSAM, gtwrap interface files, pybind11, CMake, Python unittest/pytest.

---

### Task 1: Add the failing Python contract test

**Files:**
- Create: `python/gtsam/tests/test_CudaSparseLevenbergMarquardt.py`
- Modify: `python/CMakeLists.txt:212-220`

- [ ] **Step 1: Write the failing test**

Create a CUDA-conditional `unittest.TestCase` that constructs the desired API.
Use this fixture and these assertions:

```python
import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X

cuda = getattr(gtsam, "cuda", None)


def _make_pose2_problem():
    graph = gtsam.NonlinearFactorGraph()
    prior = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.15, 0.15, 0.1]))
    odometry = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.15]))
    graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(), prior))
    graph.add(gtsam.BetweenFactorPose2(
        X(0), X(1), gtsam.Pose2(2.0, 0.2, 0.1), odometry))
    graph.add(gtsam.BetweenFactorPose2(
        X(1), X(2), gtsam.Pose2(1.5, -0.1, -0.08), odometry))
    initial = gtsam.Values()
    initial.insert(X(0), gtsam.Pose2(0.35, -0.25, 0.18))
    initial.insert(X(1), gtsam.Pose2(2.45, -0.35, -0.05))
    initial.insert(X(2), gtsam.Pose2(3.15, 0.45, 0.2))
    return graph, initial


@unittest.skipIf(cuda is None, "GTSAM was not built with CUDA")
class TestCudaSparseLevenbergMarquardt(unittest.TestCase):

    def _run_or_skip_unavailable_runtime(self, function):
        try:
            return function()
        except RuntimeError as exception:
            message = str(exception)
            unavailable_fragments = (
                "no CUDA-capable device",
                "CUDA driver version is insufficient",
                "initialization error",
                "not initialized",
                "CUDA toolkit",
            )
            if any(fragment in message for fragment in unavailable_fragments):
                self.skipTest(f"CUDA runtime is unavailable: {message}")
            raise

    def test_options_and_status_are_wrapped(self):
        linear = cuda.LinearSolverOptions()
        linear.backend = cuda.LinearSolverType.Pcg
        pcg = cuda.PcgOptions()
        pcg.maxIterations = 80
        pcg.relativeTolerance = 1e-9
        pcg.warmStart = False
        pcg.convergenceCheckInterval = 4
        params = cuda.SparseLevenbergMarquardtParams()
        params.linear = linear
        params.pcg = pcg
        params.fallbackOnUnsupported = False
        params.collectTiming = True
        params.collectAttemptTrace = True
        params.validateStructureEveryIteration = True
        self.assertEqual(cuda.LinearSolverType.Pcg, params.linear.backend)
        self.assertEqual(80, params.pcg.maxIterations)

        status = cuda.DirectJacobianStatus()
        status.failure = cuda.DirectJacobianFailure.StructuralMismatch
        status.factorIndex = 3
        status.detail = "row layout changed"
        self.assertFalse(status.ok())
        result = cuda.SparseLevenbergMarquardtResult()
        result.backend = cuda.SparseLevenbergMarquardtBackend.CpuFallback
        result.fallbackReason = (
            cuda.SparseLevenbergMarquardtFallbackReason.PlanIncompatible)
        result.fallbackStatus = status
        result.termination = (
            cuda.SparseLevenbergMarquardtTerminationReason.MaxIterations)
        result.iterations = 2
        result.initialError = 10.0
        result.finalError = 1.0
        self.assertEqual(3, result.fallbackStatus.factorIndex)
        self.assertLess(result.finalError, result.initialError)

    def test_zero_iterations_runs_without_cuda_setup(self):
        graph, initial = _make_pose2_problem()
        params = cuda.SparseLevenbergMarquardtParams()
        params.setMaxIterations(0)
        params.setErrorTol(0.0)
        params.fallbackOnUnsupported = False
        optimizer = cuda.SparseLevenbergMarquardtOptimizer(
            graph, initial, params)
        actual = optimizer.optimize()
        result = optimizer.result()
        self.assertTrue(initial.equals(actual, 1e-12))
        self.assertEqual(
            cuda.SparseLevenbergMarquardtTerminationReason.MaxIterations,
            result.termination)
        self.assertEqual(0, result.outerLinearizations)
        self.assertEqual(0, result.lambdaAttempts)
        self.assertAlmostEqual(result.initialError, result.finalError)
        self.assertAlmostEqual(result.finalError, optimizer.error())

    def test_general_cuda_pcg_improves_pose2_error(self):
        graph, initial = _make_pose2_problem()
        params = cuda.SparseLevenbergMarquardtParams()
        params.setMaxIterations(2)
        params.fallbackOnUnsupported = False
        linear = cuda.LinearSolverOptions()
        linear.backend = cuda.LinearSolverType.Pcg
        params.linear = linear
        pcg = cuda.PcgOptions()
        pcg.maxIterations = 100
        pcg.relativeTolerance = 1e-10
        params.pcg = pcg
        optimizer = self._run_or_skip_unavailable_runtime(
            lambda: cuda.SparseLevenbergMarquardtOptimizer(
                graph, initial, params))
        actual = self._run_or_skip_unavailable_runtime(optimizer.optimize)
        self.assertLess(graph.error(actual), graph.error(initial))
        self.assertEqual(
            cuda.SparseLevenbergMarquardtBackend.Device,
            optimizer.result().backend)

    def test_sfm_uses_shared_linear_solver_enum(self):
        params = cuda.SfmLevenbergMarquardtParams()
        params.setLinearSolver(cuda.LinearSolverType.Pcg)
        self.assertEqual(cuda.LinearSolverType.Pcg, params.getLinearSolver())
```

- [ ] **Step 2: Exclude the test from non-CUDA packages**

Replace the two single-test exclusions in `python/CMakeLists.txt` with:

```cmake
if(NOT GTSAM_ENABLE_CUDA)
    file(REMOVE
        "${GTSAM_MODULE_PATH}/tests/test_CudaSfm.py"
        "${GTSAM_MODULE_PATH}/tests/test_CudaSparseLevenbergMarquardt.py")
endif()

file(GLOB GTSAM_PYTHON_TEST_FILES "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/tests/*.py")
if(NOT GTSAM_ENABLE_CUDA)
    list(FILTER GTSAM_PYTHON_TEST_FILES EXCLUDE REGEX
        ".*/test_Cuda(Sfm|SparseLevenbergMarquardt)\\.py$")
endif()
```

- [ ] **Step 3: Configure and build the pre-change CUDA Python module**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=80 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-13.0/bin/nvcc \
  -DGTSAM_BUILD_TESTS=ON -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=ON \
  -DPYTHON_EXECUTABLE=/home/ubuntu/miniconda3/bin/python3
cmake --build build --target gtsam_py -j6
```

Expected: configuration and the existing wrapper build succeed.

- [ ] **Step 4: Verify RED**

Run:

```bash
PYTHONPATH=build/python /home/ubuntu/miniconda3/bin/python3 -m pytest -q \
  python/gtsam/tests/test_CudaSparseLevenbergMarquardt.py
```

Expected: FAIL because `gtsam.cuda.LinearSolverOptions` and
`SparseLevenbergMarquardtParams` do not exist.

- [ ] **Step 5: Commit the red test**

```bash
git add python/CMakeLists.txt python/gtsam/tests/test_CudaSparseLevenbergMarquardt.py
git commit -m "test: specify general CUDA LM Python API"
```

### Task 2: Implement the shared and general CUDA wrappers

**Files:**
- Create: `gtsam/linear/cuda/cuda_linear.i`
- Create: `gtsam/nonlinear/cuda/cuda_nonlinear.i`
- Create: `python/gtsam/preamble/cuda_linear.h`
- Create: `python/gtsam/preamble/cuda_nonlinear.h`
- Create: `python/gtsam/specializations/cuda_linear.h`
- Create: `python/gtsam/specializations/cuda_nonlinear.h`
- Modify: `gtsam/sfm/cuda/cuda_sfm.i:12-20`
- Modify: `python/CMakeLists.txt:136-142`

- [ ] **Step 1: Declare shared CUDA solver configuration**

Create `gtsam/linear/cuda/cuda_linear.i`:

```cpp
namespace gtsam {
namespace cuda {
#include <gtsam/linear/cuda/LinearSolver.h>
enum class LinearSolverType { DenseCholesky, Cudss, Pcg };
class LinearSolverOptions {
  LinearSolverOptions();
  gtsam::cuda::LinearSolverType backend;
};
class PcgOptions {
  PcgOptions();
  int maxIterations;
  double relativeTolerance;
  bool warmStart;
  int convergenceCheckInterval;
};
}  // namespace cuda
}  // namespace gtsam
```

- [ ] **Step 2: Add required empty gtwrap support headers**

Create the four dependency files with these exact contents:

```cpp
/* No cuda_linear-module preamble customizations are required. */
```

```cpp
/* No cuda_nonlinear-module preamble customizations are required. */
```

```cpp
/* No cuda_linear-module specializations are required. */
```

```cpp
/* No cuda_nonlinear-module specializations are required. */
```

- [ ] **Step 3: Declare the practical general CUDA LM API**

Create `gtsam/nonlinear/cuda/cuda_nonlinear.i`:

```cpp
namespace gtsam {
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
namespace cuda {
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
class SparseLevenbergMarquardtParams : gtsam::LevenbergMarquardtParams {
  SparseLevenbergMarquardtParams();
  bool fallbackOnUnsupported;
  bool collectTiming;
  bool collectAttemptTrace;
  bool validateStructureEveryIteration;
  gtsam::cuda::LinearSolverOptions linear;
  gtsam::cuda::PcgOptions pcg;
};
enum class SparseLevenbergMarquardtBackend { Device, CpuFallback };
enum class SparseLevenbergMarquardtTerminationReason {
  None, ErrorThreshold, Converged, MaxIterations, SmallCostChange,
  LambdaUpperBound
};
enum class SparseLevenbergMarquardtFallbackReason {
  None, RuntimeUnavailable, ToolkitUnsupported, CudssUnavailable,
  PlanIncompatible, DirectJacobianUnsupported
};
enum class DirectJacobianFailure {
  None, StructuralMismatch, UnsupportedGaussianFactor, ConstrainedFactor,
  NonFiniteValues
};
class DirectJacobianStatus {
  DirectJacobianStatus();
  gtsam::cuda::DirectJacobianFailure failure;
  size_t factorIndex;
  string detail;
  bool ok() const;
};
class SparseLevenbergMarquardtResult {
  SparseLevenbergMarquardtResult();
  gtsam::cuda::SparseLevenbergMarquardtBackend backend;
  gtsam::cuda::SparseLevenbergMarquardtFallbackReason fallbackReason;
  gtsam::cuda::DirectJacobianStatus fallbackStatus;
  string fallbackDetail;
  gtsam::cuda::SparseLevenbergMarquardtTerminationReason termination;
  size_t outerLinearizations;
  size_t iterations;
  size_t lambdaAttempts;
  size_t acceptedSteps;
  size_t cudssAnalyses;
  size_t pcgIterationsTotal;
  size_t pcgSolves;
  size_t pcgMaxIterationHits;
  double initialError;
  double finalError;
  double finalLambda;
};
class SparseLevenbergMarquardtOptimizer {
  SparseLevenbergMarquardtOptimizer(
      const gtsam::NonlinearFactorGraph& graph,
      const gtsam::Values& initialValues,
      const gtsam::cuda::SparseLevenbergMarquardtParams& params =
          gtsam::cuda::SparseLevenbergMarquardtParams());
  const gtsam::Values& optimize();
  const gtsam::Values& values() const;
  double error() const;
  const gtsam::cuda::SparseLevenbergMarquardtParams& params() const;
  const gtsam::cuda::SparseLevenbergMarquardtResult& result() const;
};
}  // namespace cuda
}  // namespace gtsam
```

- [ ] **Step 4: Share the enum and wire interfaces**

Delete the `LinearSolverType` block from `cuda_sfm.i`. Change the CUDA CMake
list to:

```cmake
if(GTSAM_ENABLE_CUDA)
    list(APPEND interface_headers
        ${PROJECT_SOURCE_DIR}/gtsam/linear/cuda/cuda_linear.i
        ${PROJECT_SOURCE_DIR}/gtsam/nonlinear/cuda/cuda_nonlinear.i
        ${PROJECT_SOURCE_DIR}/gtsam/sfm/cuda/cuda_sfm.i
    )
endif()
```

- [ ] **Step 5: Build and verify GREEN**

Run:

```bash
cmake --build build --target gtsam_py -j6
PYTHONPATH=build/python /home/ubuntu/miniconda3/bin/python3 -m pytest -q \
  python/gtsam/tests/test_CudaSparseLevenbergMarquardt.py \
  python/gtsam/tests/test_CudaSfm.py
```

Expected: wrapper generation and compilation succeed; both focused test files
pass, with only runtime-dependent skips.

- [ ] **Step 6: Commit the implementation**

```bash
git add gtsam/linear/cuda/cuda_linear.i \
  gtsam/nonlinear/cuda/cuda_nonlinear.i gtsam/sfm/cuda/cuda_sfm.i \
  python/gtsam/preamble/cuda_linear.h \
  python/gtsam/preamble/cuda_nonlinear.h \
  python/gtsam/specializations/cuda_linear.h \
  python/gtsam/specializations/cuda_nonlinear.h python/CMakeLists.txt
git commit -m "feat: bind general CUDA LM to Python"
```

### Task 3: Verify integration

**Files:**
- Verify only.

- [ ] **Step 1: Run wrapper-generator tests**

```bash
/home/ubuntu/miniconda3/bin/python3 -m pytest -q -c /dev/null \
  -p no:cacheprovider wrap/tests/test_interface_parser.py \
  wrap/tests/test_template_instantiator.py wrap/tests/test_pybind_wrapper.py
```

Expected: 82 tests pass.

- [ ] **Step 2: Build the non-CUDA Python module**

```bash
cmake -S . -B build-no-cuda -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_ENABLE_CUDA=OFF \
  -DPYTHON_EXECUTABLE=/home/ubuntu/miniconda3/bin/python3
cmake --build build-no-cuda --target gtsam_py -j6
test ! -e build-no-cuda/python/gtsam/tests/test_CudaSfm.py
test ! -e build-no-cuda/python/gtsam/tests/test_CudaSparseLevenbergMarquardt.py
```

Expected: CPU-only wrapper builds and neither CUDA test is packaged.

- [ ] **Step 3: Run repository checks**

```bash
git diff --check upstream/develop...HEAD
git status --short --branch
git log --oneline --decorate upstream/develop..HEAD
```

Expected: no whitespace errors, only intended commits, and a clean worktree.
