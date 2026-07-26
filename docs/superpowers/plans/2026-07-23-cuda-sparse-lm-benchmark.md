# CUDA Sparse LM Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add and run one reproducible benchmark that compares ordinary CPU GTSAM LM with the generic direct-sparse CUDA LM on Dubrovnik-16, Dubrovnik-135, `w10000` Pose2, and `sphere_smallnoise` Pose3, while preserving full per-run GPU stage profiles.

**Architecture:** A CUDA/cuDSS-gated timing executable owns dataset loading, anchoring, fair paired execution, validation, statistics, and JSON/CSV/text reporting. The existing `CudaSparseLevenbergMarquardtResult` remains the source of GPU system, transfer, attempt, and detailed stage data; production optimizer code is unchanged.

**Tech Stack:** C++17, GTSAM nonlinear/slam/SFM APIs, CUDA 13, cuDSS, CMake, standard-library JSON/CSV serialization.

---

### Task 1: Add benchmark support and its self-test

**Files:**
- Create: `timing/cuda_sparse/CMakeLists.txt`
- Create: `timing/cuda_sparse/timeCudaSparseLM.cpp`
- Modify: `timing/CMakeLists.txt`

- [ ] **Step 1: Add the CUDA-gated timing target**

Add this line to `timing/CMakeLists.txt`:

```cmake
add_subdirectory(cuda_sparse)
```

Create `timing/cuda_sparse/CMakeLists.txt`:

```cmake
if(GTSAM_ENABLE_CUDA AND GTSAM_ENABLE_CUDSS)
  gtsamAddTimingGlob("*.cpp" "" "gtsam" ${GTSAM_BUILD_TIMING_ALWAYS})
endif()
```

- [ ] **Step 2: Write a failing benchmark self-test entry point**

Create `timeCudaSparseLM.cpp` with an initial `--self-test` path that calls, but does not yet define, these focused helpers:

```cpp
RunOptions ParseOptions(int argc, char** argv);
SummaryStatistics Summarize(const std::vector<double>& samples);
void ValidateObjective(double reference, double candidate, double tolerance);
std::string CsvEscape(const std::string& value);
std::string JsonEscape(const std::string& value);
int RunSelfTest();
```

`RunSelfTest()` must cover:

```cpp
Summarize({1.0, 2.0, 3.0, 4.0, 5.0});
ValidateObjective(100.0, 100.0 + 5e-7, 1e-8);
CsvEscape("a,\"b\"");
JsonEscape("a\n\"b\"");
ParseOptions({"timeCudaSparseLM", "--warmups", "1", "--repeats", "5",
              "--datasets", "bal16,bal135,pose2,pose3",
              "--json", "results.json", "--csv", "results.csv"});
```

- [ ] **Step 3: Configure and build to verify the self-test target fails**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target timeCudaSparseLM -j2
```

Expected: compilation fails because the benchmark helpers are not defined.

- [ ] **Step 4: Implement CLI parsing, statistics, escaping, and self-test**

Implement:

```cpp
struct RunOptions {
  size_t warmups = 1;
  size_t repeats = 5;
  std::vector<std::string> datasets{"bal16", "bal135", "pose2", "pose3"};
  std::string jsonPath = "cuda-sparse-lm-benchmark.json";
  std::string csvPath = "cuda-sparse-lm-benchmark.csv";
  bool selfTest = false;
};

struct SummaryStatistics {
  double median = 0.0;
  double mean = 0.0;
  double standardDeviation = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
};
```

Reject zero repeats, malformed integers, unknown dataset names, duplicate dataset names, missing option values, and unknown options. Use population standard deviation and the arithmetic mean of the two middle samples for an even-sized median.

`ValidateObjective` must use:

```cpp
const double allowed =
    tolerance * std::max({1.0, std::abs(reference), std::abs(candidate)});
```

and throw when the absolute difference exceeds `allowed`.

- [ ] **Step 5: Build and run the self-test**

Run:

```bash
cmake --build build-cuda-cudss-on --target timeCudaSparseLM -j2
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM --self-test
```

Expected:

```text
timeCudaSparseLM self-test passed
```

### Task 2: Implement common workload loading and fair paired runs

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`

- [ ] **Step 1: Add workload descriptions**

Implement:

```cpp
enum class WorkloadKind { Bal, Pose2, Pose3 };

struct WorkloadSpec {
  std::string name;
  WorkloadKind kind;
  std::string defaultPath;
};

struct LoadedWorkload {
  WorkloadSpec spec;
  NonlinearFactorGraph graph;
  Values initial;
};
```

Use:

```text
bal16  -> dubrovnik-16-22106-pre.txt
bal135 -> dubrovnik-135-90642-pre.txt
pose2  -> w10000.graph
pose3  -> sphere_smallnoise.graph
```

Resolve bundled paths with `findExampleDataFile`; allow `--data-dir` to override the root.

- [ ] **Step 2: Load and anchor every workload**

For BAL, reuse the same camera/value and factor construction conventions as `timeCudaSFMBAL`.

For Pose2:

```cpp
auto [graph, initial] = load2D(path);
graph->addPrior(0, initial->at<Pose2>(0), noiseModel::Unit::Create(3));
```

For Pose3:

```cpp
auto [graph, initial] = load3D(path);
graph->addPrior(0, initial->at<Pose3>(0), noiseModel::Unit::Create(6));
```

Reject empty graphs, empty values, missing key 0, or non-finite initial errors.

- [ ] **Step 3: Implement identical nonlinear LM parameters**

Start from:

```cpp
LevenbergMarquardtParams::SetCeresDefaults(&params);
params.setVerbosityLM("SILENT");
```

Apply the existing BAL convergence settings for BAL workloads. Use the same nonlinear damping and convergence fields for CPU and GPU. Do not apply CPU ordering settings to the GPU interpretation; cuDSS retains its internal reordering.

- [ ] **Step 4: Implement CPU and GPU runs**

Measure construction plus `optimize()` with `std::chrono::steady_clock`.

CPU:

```cpp
LevenbergMarquardtOptimizer optimizer(graph, initial, params);
Values values = optimizer.optimize();
```

GPU:

```cpp
CudaSparseLevenbergMarquardtParams gpuParams;
// Copy the shared LM fields.
gpuParams.fallbackOnUnsupported = false;
gpuParams.collectTiming = true;
gpuParams.collectAttemptTrace = true;
CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, gpuParams);
Values values = optimizer.optimize();
```

Require the GPU backend to remain `Cuda`, termination to be non-`None`, objectives and deltas to be finite, and the final objective to match the first measured CPU objective within relative tolerance `1e-8`.

- [ ] **Step 5: Implement warm-up and paired ordering**

For each workload:

```text
CPU warm-up
GPU warm-up
repeat 0: CPU then GPU
repeat 1: GPU then CPU
repeat 2: CPU then GPU
repeat 3: GPU then CPU
repeat 4: CPU then GPU
```

Each run receives a fresh optimizer and an unchanged copy of the same initial `Values`.

- [ ] **Step 6: Build and run one-repeat smoke tests**

Run:

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 0 --repeats 1 --datasets pose2 \
  --json /tmp/cuda-sparse-pose2-smoke.json \
  --csv /tmp/cuda-sparse-pose2-smoke.csv

./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 0 --repeats 1 --datasets pose3 \
  --json /tmp/cuda-sparse-pose3-smoke.json \
  --csv /tmp/cuda-sparse-pose3-smoke.csv
```

Expected: both use the CUDA backend, complete normally, and pass CPU/GPU objective parity.

### Task 3: Preserve detailed profiles and summary statistics

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`

- [ ] **Step 1: Define raw records**

Store for every run:

```cpp
struct StageValues {
  // One member for every CudaSparseLmStageTimings field.
};

struct RawRun {
  std::string dataset;
  std::string backend;
  size_t repetition = 0;
  double externalWall = 0.0;
  double initialError = 0.0;
  double finalError = 0.0;
  size_t iterations = 0;
  size_t outerLinearizations = 0;
  size_t lambdaAttempts = 0;
  size_t acceptedSteps = 0;
  size_t cudssAnalyses = 0;
  CudaSparseLmSystemSize systemSize;
  CudaSparseLmTransferCounts transfers;
  CudaSparseLmStageTimings timings;
  std::vector<CudaSparseLmAttemptRecord> attempts;
};
```

- [ ] **Step 2: Aggregate each scalar independently**

For CPU and GPU external wall time, and every detailed GPU timing field, compute median, mean, population standard deviation, minimum, and maximum across the five measured runs.

Do not sum overlapping timing fields. Label `factorLinearizationCpuSum` and `csrPackingCpuSum` as worker CPU sums.

- [ ] **Step 3: Print human-readable results**

Print:

```text
dataset and graph/system sizes
initial/final objective
CPU and GPU five-run statistics
median CPU/GPU speedup
iterations, attempts, analyses
median detailed GPU stages
median transfer time and byte counts
objective-parity tolerance and maximum observed difference
```

### Task 4: Implement durable JSON and CSV output

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`

- [ ] **Step 1: Write JSON after every completed workload**

Write to `<output>.tmp`, flush and close it, then atomically rename to the requested JSON path. Include:

```text
schema_version
UTC timestamp
git commit and dirty flag
build type
CUDA runtime/toolkit and driver
GPU name, compute capability, and memory
command-line configuration
dataset paths
raw CPU/GPU runs
attempt traces
per-field summaries
parity checks
```

Rewriting after each workload preserves completed results if a later workload fails.

- [ ] **Step 2: Write summary CSV**

Write one row per dataset/backend with:

```text
dataset, backend, factors, values, rows, columns, J.nnz, H.nnz,
warmups, repeats, external median/mean/stddev/min/max,
initial error, final error, iterations, attempts, analyses,
CPU/GPU speedup, objective difference
```

Add GPU timing medians and transfer bytes as additional columns.

- [ ] **Step 3: Validate outputs with the standard library and Python**

Run:

```bash
python3 -m json.tool /tmp/cuda-sparse-pose2-smoke.json >/dev/null
python3 - <<'PY'
import csv
with open("/tmp/cuda-sparse-pose2-smoke.csv", newline="") as stream:
    rows = list(csv.DictReader(stream))
assert rows
PY
```

Expected: both commands succeed.

### Task 5: Run the full benchmark and record results

**Files:**
- Create: `timing/cuda_sparse/results/2026-07-23-a100/benchmark.json`
- Create: `timing/cuda_sparse/results/2026-07-23-a100/summary.csv`
- Create: `timing/cuda_sparse/results/2026-07-23-a100/console.log`
- Create: `docs/superpowers/results/2026-07-23-cuda-sparse-lm-multidomain-benchmark.md`

- [ ] **Step 1: Verify the Release build and GPU environment**

Run:

```bash
rg 'CMAKE_BUILD_TYPE:STRING=Release' build-cuda-cudss-on/CMakeCache.txt
nvidia-smi --query-gpu=name,memory.total,driver_version,compute_cap --format=csv
nvcc --version
```

- [ ] **Step 2: Run one warm-up plus five measured runs**

Run:

```bash
mkdir -p timing/cuda_sparse/results/2026-07-23-a100
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 \
  --datasets bal16,bal135,pose2,pose3 \
  --data-dir /home/ubuntu/cu-gtsam/examples/Data \
  --json timing/cuda_sparse/results/2026-07-23-a100/benchmark.json \
  --csv timing/cuda_sparse/results/2026-07-23-a100/summary.csv \
  2>&1 | tee timing/cuda_sparse/results/2026-07-23-a100/console.log
```

- [ ] **Step 3: Validate the full artifacts**

Run:

```bash
python3 -m json.tool \
  timing/cuda_sparse/results/2026-07-23-a100/benchmark.json >/dev/null
python3 - <<'PY'
import csv
path = "timing/cuda_sparse/results/2026-07-23-a100/summary.csv"
with open(path, newline="") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 8, len(rows)
assert {row["dataset"] for row in rows} == {
    "bal16", "bal135", "pose2", "pose3"
}
assert {row["backend"] for row in rows} == {"cpu", "gpu"}
PY
```

- [ ] **Step 4: Write the result report**

Document hardware, build, commands, graph sizes, parity, five-run statistics, detailed median GPU stages, transfers, and interpretation. Clearly distinguish overlapping stage timers from exclusive wall time.

- [ ] **Step 5: Run final verification**

Run:

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM --self-test
git diff --check
```

Expected: self-test passes and `git diff --check` is silent.
