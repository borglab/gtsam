# Stereo Visual Odometry CUDA Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three native stereo visual-odometry workloads to the paired CPU/CUDA LM harness and produce verified five-repeat results for every CUDA configuration.

**Architecture:** Keep all new behavior in `timeCudaSparseLM.cpp`: a stereo workload specification resolves one calibration, pose, and factor file; a loader constructs the same `GenericStereoFactor<Pose3, Point3>` graph as GTSAM's example but uses a finite prior compatible with the CUDA Jacobian frontend. The existing optimizer, timing protocol, serializers, and solver matrix remain unchanged.

**Tech Stack:** C++17, GTSAM nonlinear factors and `GenericStereoFactor`, CUDA general sparse LM, cuDSS, PCG, CMake, JSON/CSV benchmark artifacts.

---

### Task 1: Register the stereo workload surface test-first

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`

- [ ] **Step 1: Write the failing CLI self-test**

Extend `RunSelfTest()` with a parse case for all three names:

```cpp
const char* stereoArguments[] = {
    "timeCudaSparseLM", "--datasets", "stereo26,stereo77,stereo135"};
const RunOptions stereoOptions =
    ParseOptions(3, const_cast<char**>(stereoArguments));
if (stereoOptions.datasets !=
    std::vector<std::string>({"stereo26", "stereo77", "stereo135"})) {
  throw std::runtime_error("stereo workload option parsing self-test failed");
}
```

- [ ] **Step 2: Verify the new test fails for the intended reason**

Run:

```bash
cmake --build build-cuda-cudss-on -j6 --target timeCudaSparseLM
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM --self-test
```

Expected: the self-test exits nonzero with `unknown dataset: stereo26`.

- [ ] **Step 3: Register the names and workload metadata**

Add the required includes:

```cpp
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/StereoFactor.h>
```

Extend the kind and specification without changing existing aggregate
initializers:

```cpp
enum class WorkloadKind { Bal, Pose2, Pose3, Stereo };

struct WorkloadSpec {
  std::string name;
  WorkloadKind kind;
  std::string filename;
  std::string exampleDataName;
  std::string poseFilename;
  std::string calibrationFilename;
};
```

Add `stereo26`, `stereo77`, and `stereo135` to `SplitDatasets()` and
`LookupWorkload()` with factor files `VO_stereo_factors_large.txt`,
`VO_stereo_factors00s.txt`, and `VO_stereo_factors00.txt`; pose files
`VO_camera_poses_large.txt`, `VO_camera_poses00s.txt`, and
`VO_camera_poses00.txt`; and calibration files `VO_calibration.txt`,
`VO_calibration00s.txt`, and `VO_calibration00.txt`.

Update `Usage()` so its dataset list contains the three new names.

- [ ] **Step 4: Verify the self-test passes**

Run the build and `--self-test` commands from Step 2.

Expected: `timeCudaSparseLM self-test passed` and exit code zero.

- [ ] **Step 5: Commit the registered interface**

```bash
git add timing/cuda_sparse/timeCudaSparseLM.cpp
git commit -m "test: register stereo VO CUDA workloads"
```

### Task 2: Construct stereo graphs with strict input validation

**Files:**
- Modify: `timing/cuda_sparse/timeCudaSparseLM.cpp`

- [ ] **Step 1: Write a failing loader self-test**

Add a pure stream-based helper declaration and exercise one calibration, two
poses, and two observations from string streams. Assert two pose keys, two
landmark keys, three total factors including the prior, and a point-first
ordering ending with the two pose keys. Add rejection checks for an observation
whose pose is absent and for a truncated factor row.

The core positive fixture is:

```cpp
std::istringstream calibration("100 100 0 50 40 0.2");
std::istringstream poses(
    "0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n"
    "1 1 0 0 1 0 1 0 0 0 0 1 0 0 0 0 1\n");
std::istringstream factors(
    "0 7 60 58 40 1 0 10\n"
    "1 8 62 60 40 2 0 10\n");
```

- [ ] **Step 2: Verify the helper test does not compile**

Run:

```bash
cmake --build build-cuda-cudss-on -j6 --target timeCudaSparseLM
```

Expected: compilation fails because the stereo loader helper is not defined.

- [ ] **Step 3: Implement the minimal validated loader**

Implement a local helper returning graph, initial values, and ordering. Parse
exactly six finite calibration scalars and reject trailing tokens. Parse pose
rows as a unique integer pose ID plus sixteen finite row-major matrix values.
Parse factor rows as pose ID, landmark ID, `uL,uR,v,X,Y,Z`; reject missing
poses, incomplete rows, trailing tokens, nonfinite values, and an empty input.

For every observation add:

```cpp
graph.emplace_shared<gtsam::GenericStereoFactor<Pose3, gtsam::Point3>>(
    gtsam::StereoPoint2(uL, uR, v), measurementModel,
    gtsam::Symbol('x', poseId), gtsam::Symbol('l', landmarkId), calibration);
```

On the first observation of a landmark initialize it with:

```cpp
const Pose3& cameraPose = initial.at<Pose3>(gtsam::Symbol('x', poseId));
initial.insert(gtsam::Symbol('l', landmarkId),
               cameraPose.transformFrom(gtsam::Point3(X, Y, Z)));
```

Anchor the smallest pose key using:

```cpp
graph.emplace_shared<gtsam::PriorFactor<Pose3>>(
    firstPoseKey, initial.at<Pose3>(firstPoseKey),
    gtsam::noiseModel::Unit::Create(6));
```

Build an `Ordering` containing every `l` key in key order followed by every
`x` key in key order. Return the ordering as `cpuOrdering`.

- [ ] **Step 4: Connect file resolution and workload loading**

For stereo specifications, resolve the factor path through existing
`ResolvePath()` semantics and resolve pose/calibration files beside it when
`--data-dir` is present, otherwise through `findExampleDataFile`. Open all
three streams with explicit path-bearing errors and populate `LoadedWorkload`.
Keep `workload.path` as the factor file so JSON provenance remains meaningful.

- [ ] **Step 5: Verify the self-test and dry-run**

```bash
cmake --build build-cuda-cudss-on -j6 --target timeCudaSparseLM
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM --self-test
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --datasets stereo26,stereo77,stereo135 --all-cuda-configurations --dry-run
```

Expected: self-test passes and dry-run prints all three configurations without
an unknown-dataset error.

- [ ] **Step 6: Commit the loader**

```bash
git add timing/cuda_sparse/timeCudaSparseLM.cpp
git commit -m "bench: add stereo visual odometry workloads"
```

### Task 3: Validate live CPU/CUDA behavior

**Files:**
- Create: `timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/smoke.*`

- [ ] **Step 1: Run cuDSS-auto smoke parity**

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 0 --repeats 1 --datasets stereo26 \
  --data-dir examples/Data --configuration cudss-auto \
  --json timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/smoke.auto.json \
  --csv timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/smoke.auto.csv
```

Expected: CUDA backend, no fallback exception, and objective difference within
`1e-8` relative tolerance.

- [ ] **Step 2: Run GTSAM-ordering smoke parity**

Repeat Step 1 with `--configuration cudss-gtsam` and `.gtsam` output names.
Expected: direct parity passes, one cuDSS analysis, and nonempty applied
permutation evidence from source/session statistics.

- [ ] **Step 3: Run PCG smoke parity**

Repeat Step 1 with `--configuration pcg` and `.pcg` output names.
Expected: objective difference within the recorded `1e-3` tolerance, no
iteration-cap hit or breakdown, at least one solve, and the last solve
converged.

- [ ] **Step 4: Inspect machine artifacts**

Use `jq` to verify every smoke JSON has one workload and one measured CPU/GPU
run, finite positive wall times, the expected factor/value counts, and the
backend-specific solver gates above.

### Task 4: Run and audit the complete stereo matrix

**Files:**
- Create: `timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/results.*`
- Create: `docs/superpowers/results/2026-08-16-stereo-vo-cuda-benchmark.md`

- [ ] **Step 1: Run all nine benchmark combinations**

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM \
  --warmups 1 --repeats 5 \
  --datasets stereo26,stereo77,stereo135 \
  --data-dir examples/Data --all-cuda-configurations \
  --json timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/results.json \
  --csv timing/cuda_sparse/benchmark_logs/20260816_stereo_vo/results.csv
```

Expected suffix artifacts are `results.cudss-auto.*`,
`results.cudss-gtsam.*`, and `results.pcg.*`.

- [ ] **Step 2: Audit all artifacts**

Verify with `jq`/`awk`:

- three workloads per configuration and five raw CPU/GPU repetitions each;
- exact factor/value counts from the design;
- finite positive median wall times;
- direct objective tolerance `1e-8` and PCG tolerance `1e-3` respected;
- no fallback (the harness would already abort), PCG cap hit, or breakdown;
- cuDSS analysis count one per direct run and PCG convergence true.

- [ ] **Step 3: Write the report-ready result record**

Create the result Markdown with revision/build/GPU/protocol metadata, a table
of CPU and GPU medians and speedups for all nine combinations, endpoint and
iteration evidence, stage attribution, and these caveats: hybrid CPU
linearization/retract/error, finite instead of equality anchor, full-normal
rather than explicit Schur, and overlapping GPU stage timers.

- [ ] **Step 4: Run final verification**

```bash
./build-cuda-cudss-on/timing/cuda_sparse/timeCudaSparseLM --self-test
git diff --check HEAD~2
git status --short
```

Also rerun the artifact audit from Step 2 and inspect the final source diff.

- [ ] **Step 5: Commit implementation and report artifacts**

```bash
git add timing/cuda_sparse/timeCudaSparseLM.cpp \
  timing/cuda_sparse/benchmark_logs/20260816_stereo_vo \
  docs/superpowers/results/2026-08-16-stereo-vo-cuda-benchmark.md
git commit -m "bench: report stereo VO CUDA solver matrix"
```
