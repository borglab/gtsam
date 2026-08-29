# General CUDA LM Python Documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the new General CUDA LM Python binding discoverable, explain both CUDA optimizer APIs, and provide one executable General LM tutorial.

**Architecture:** Keep each document focused: `python/README.md` is the build and discovery entry point, `docs/CUDA_LINEAR_SOLVERS.md` is the canonical CUDA Python API guide, and the sparse-LM notebook is the executable tutorial. The examples use the already-tested `gtsam.cuda` API and do not duplicate the full C++ reference.

**Tech Stack:** Markdown, Jupyter Notebook JSON, Python, GTSAM, CUDA, CMake.

---

### Task 1: Add CUDA Binding Discovery to the Python README

**Files:**
- Modify: `python/README.md:35-55`

- [ ] **Step 1: Add the CUDA bindings section**

Insert this section after the Python installation instructions and before the
Windows installation section:

```markdown
## CUDA Bindings

The optional CUDA optimizers are exposed under `gtsam.cuda` only when the
Python wrapper is built from source with CUDA enabled. Configure and build the
module with:

```bash
cmake -S . -B build-cuda -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_ENABLE_CUDA=ON
cmake --build build-cuda --target gtsam_py -j6
```

This is sufficient for the matrix-free PCG backend and CUDA SFM dense
Cholesky. Add `-DGTSAM_ENABLE_CUDSS=ON` to enable the cuDSS sparse direct
backend; cuDSS must be installed separately.

When CUDA is disabled, `gtsam.cuda` is intentionally absent. See the
[CUDA linear solver guide](../docs/CUDA_LINEAR_SOLVERS.md) for the General LM
and SFM Python APIs, backend selection, and examples.
```

- [ ] **Step 2: Check the README link and formatting**

Run:

```bash
test -f python/../docs/CUDA_LINEAR_SOLVERS.md
git diff --check -- python/README.md
```

Expected: both commands exit successfully.

### Task 2: Document the General and SFM Python APIs

**Files:**
- Modify: `docs/CUDA_LINEAR_SOLVERS.md:97-112`

- [ ] **Step 1: Add the Python bindings overview and General LM example**

After the C++ selection section, add a `## Python bindings` section stating
that the namespace exists only in CUDA-enabled builds, then include this
General LM example:

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

Explain that `graph` and `initial` are ordinary `gtsam.NonlinearFactorGraph`
and `gtsam.Values` objects. State that `fallbackOnUnsupported = False` is useful
when GPU execution is required, while the default permits CPU fallback and
reports the reason through `diagnostics.fallbackReason` and
`diagnostics.fallbackDetail`.

- [ ] **Step 2: Add the CUDA SFM example and backend constraints**

Continue the same section with:

```python
params = cuda.SfmLevenbergMarquardtParams.ceresDefaults()
params.setLinearSolver(cuda.LinearSolverType.DenseCholesky)

result = cuda.optimizeSfm(data, params)
print(result.finalError, result.iterations)
values = result.optimizedValues
```

Explain that `data` is a `gtsam.SfmData`, `DenseCholesky` is available only to
CUDA SFM, General CUDA LM accepts `Pcg` or `Cudss`, and `Cudss` requires a
cuDSS-enabled build. Link back to `../python/README.md` for build instructions
and to the General LM notebook for a complete runnable graph.

- [ ] **Step 3: Validate local links and Markdown formatting**

Run:

```bash
test -f docs/../python/README.md
test -f gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb
git diff --check -- docs/CUDA_LINEAR_SOLVERS.md
```

Expected: all commands exit successfully.

### Task 3: Add the Executable General CUDA LM Notebook Example

**Files:**
- Modify: `gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb`

- [ ] **Step 1: Normalize the notebook preamble**

Arrange the first five cells in the repository-required order:

1. The existing title and feature introduction Markdown.
2. The copyright Markdown with `metadata.tags` set to `["remove-cell"]`.
3. The existing Colab badge Markdown.
4. The Colab installation cell with `metadata.tags` set to
   `["remove-cell"]`.
5. An imports cell containing:

```python
import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
```

- [ ] **Step 2: Add a Python usage explanation**

Add a Markdown cell explaining that the example uses the PCG backend because
it needs only `GTSAM_ENABLE_CUDA=ON`, that the Python module must be built from
source with CUDA enabled, and that the same graph and values types used by CPU
optimizers are accepted.

- [ ] **Step 3: Add the runnable Pose2 optimization**

Add one code cell with the complete example:

```python
if not hasattr(gtsam, "cuda"):
    raise RuntimeError(
        "This example requires a source build with GTSAM_ENABLE_CUDA=ON")

cuda = gtsam.cuda
graph = gtsam.NonlinearFactorGraph()
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.15, 0.15, 0.1]))
odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.2, 0.2, 0.15]))
graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(), prior_noise))
graph.add(gtsam.BetweenFactorPose2(
    X(0), X(1), gtsam.Pose2(2.0, 0.2, 0.1), odometry_noise))
graph.add(gtsam.BetweenFactorPose2(
    X(1), X(2), gtsam.Pose2(1.5, -0.1, -0.08), odometry_noise))

initial = gtsam.Values()
initial.insert(X(0), gtsam.Pose2(0.35, -0.25, 0.18))
initial.insert(X(1), gtsam.Pose2(2.45, -0.35, -0.05))
initial.insert(X(2), gtsam.Pose2(3.15, 0.45, 0.2))

linear = cuda.LinearSolverOptions()
linear.backend = cuda.LinearSolverType.Pcg
pcg = cuda.PcgOptions()
pcg.maxIterations = 100
pcg.relativeTolerance = 1e-10

params = cuda.SparseLevenbergMarquardtParams()
params.linear = linear
params.pcg = pcg
params.fallbackOnUnsupported = False
params.setMaxIterations(5)

optimizer = cuda.SparseLevenbergMarquardtOptimizer(graph, initial, params)
values = optimizer.optimize()
diagnostics = optimizer.result()

print(f"initial error: {diagnostics.initialError:.6g}")
print(f"final error:   {diagnostics.finalError:.6g}")
print(f"iterations:    {diagnostics.iterations}")
assert diagnostics.backend == cuda.SparseLevenbergMarquardtBackend.Device
assert graph.error(values) < graph.error(initial)
```

- [ ] **Step 4: Validate notebook structure and execute the example**

Run:

```bash
jq empty gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb
jq -e '
  .cells[0].cell_type == "markdown" and
  .cells[1].cell_type == "markdown" and
  (.cells[1].metadata.tags | index("remove-cell")) != null and
  .cells[2].cell_type == "markdown" and
  .cells[3].cell_type == "code" and
  (.cells[3].metadata.tags | index("remove-cell")) != null and
  .cells[4].cell_type == "code"
' gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb
PYTHONPATH=/dev/shm/gtsam-general-cuda-python-binding-build/python \
  /home/ubuntu/miniconda3/bin/python3 -c '
import json
path = "gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb"
with open(path, encoding="utf-8") as stream:
    notebook = json.load(stream)
namespace = {}
for index in (4, 6):
    exec("".join(notebook["cells"][index]["source"]), namespace)
'
```

Expected: the notebook parses, the preamble assertion succeeds, and the CUDA
PCG example prints a lower final error without raising an assertion.

### Task 4: Review and Commit the Documentation

**Files:**
- Modify: `python/README.md`
- Modify: `docs/CUDA_LINEAR_SOLVERS.md`
- Modify: `gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb`

- [ ] **Step 1: Review the rendered source and final diff**

Run:

```bash
git diff -- python/README.md docs/CUDA_LINEAR_SOLVERS.md \
  gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb
git diff --check
```

Expected: only the approved documentation changes appear and no whitespace
errors are reported.

- [ ] **Step 2: Commit the documentation**

```bash
git add python/README.md docs/CUDA_LINEAR_SOLVERS.md \
  gtsam/nonlinear/doc/SparseLevenbergMarquardtOptimizer.ipynb
git commit -m "docs: explain CUDA LM Python bindings"
```

- [ ] **Step 3: Verify the branch is clean**

Run:

```bash
git status --short --branch
git log --oneline --decorate upstream/develop..HEAD
```

Expected: the worktree is clean and the documentation commit is the branch
tip.
