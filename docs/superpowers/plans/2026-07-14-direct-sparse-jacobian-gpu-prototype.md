# Direct Sparse Jacobian GPU Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in batch Levenberg-Marquardt optimizer that accepts an ordinary `NonlinearFactorGraph` and `Values`, streams supported per-factor `JacobianFactor` results into a cached scalar CSR Jacobian, forms normal equations with cuSPARSE, and solves them with the existing cuDSS wrapper.

**Architecture:** Keep the public API to parameters, result/profile data, and the optimizer. Internally, separate immutable CPU structure (`SparseJacobianColumnLayout` and `SparseJacobianPlan`), mutable pinned host numbers (`HostSparseJacobian`), CPU/TBB factor evaluation and CSR packing (`StreamingSparseJacobianLinearizer`), and persistent GPU normal-equation state (`DeviceSparseJacobianNormalEquations`). Unsupported factor semantics restart ordinary CPU LM from the original initial values; CUDA execution failures throw with stage context.

**Tech Stack:** C++17, Eigen/GTSAM `Values`, `VectorValues`, `NonlinearFactorGraph`, and `JacobianFactor`; Intel TBB through `GTSAM_USE_TBB`; CUDA runtime; cuSPARSE CSR-to-CSC, fixed-pattern SpGEMM-reuse, and SpMV; CUB device reduction; existing `CudaDeviceArray`, `DeviceSparseNormalEquations`, and `CudssSpdSolver`; CppUnitLite tests.

**Design reference:** `docs/superpowers/specs/2026-07-10-direct-sparse-jacobian-gpu-prototype-design.md`

---

## File map

Create these focused implementation units:

- `gtsam/base/cuda/CudaPinnedHostArray.h`: move-only RAII storage backed by `cudaMallocHost`.
- `gtsam/nonlinear/cuda/SparseJacobianPlan.h/.cpp`: key-to-column layout, factor/block write plans, scalar CSR structure, compatibility validation, and flat-delta conversion.
- `gtsam/nonlinear/cuda/HostSparseJacobian.h`: pinned mutable `J.values` and `b` buffers.
- `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h/.cpp`: factor scheduling, temporary `JacobianFactor` validation, whitening, and numeric CSR writes.
- `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h/.cu`: persistent device `J`, `J^T`, `H`, `g`, damping, model-error reduction, and cuDSS solve state.
- `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h/.cpp`: public parameters, profiles, result, fallback policy, and LM control flow.
- `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`: symbolic plan, host storage, streaming linearizer, and device algebra tests.
- `gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp`: optimizer parity and fallback tests.

Modify these existing files:

- `gtsam/CMakeLists.txt`: link `CUDA::cusparse` when CUDA is enabled.
- `gtsam/nonlinear/cuda/CudssLinearSolver.h/.cpp`: surface cuDSS numerical-factorization information before solve while preserving reusable analysis.
- `gtsam/nonlinear/tests/CMakeLists.txt`: exclude the two CUDA sparse tests from non-CUDA builds.
- `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`: retain successful cuDSS solves and cover non-SPD factorization information.
- `timing/sfm_ba/timeCudaSFMBAL.cpp`: add generic sparse-Jacobian LM and diagnostic Gaussian-graph packing benchmark modes.

The `gtsam/CMakeLists.txt` recursive source glob and `gtsam/nonlinear/CMakeLists.txt` CUDA-header install glob automatically discover the new implementation files and headers.

---

### Task 1: Implement the immutable column layout and CSR assembly plan

**Files:**
- Create: `gtsam/nonlinear/cuda/SparseJacobianPlan.h`
- Create: `gtsam/nonlinear/cuda/SparseJacobianPlan.cpp`
- Create: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`
- Modify: `gtsam/nonlinear/tests/CMakeLists.txt`

- [ ] **Step 1: Add the CUDA-only test exclusion and a failing layout test**

Add both new test filenames to the existing non-CUDA exclusion branch:

```cmake
if(NOT GTSAM_ENABLE_CUDA)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaDeviceValues.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaDeviceValues.cpp")
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaSparseJacobian.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaSparseJacobian.cpp")
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaSparseLevenbergMarquardt.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaSparseLevenbergMarquardt.cpp")
  endif()
endif()
```

Start `testCudaSparseJacobian.cpp` with a deliberately reversed local-key factor so the test proves that local block order and global column order are independent:

```cpp
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {
constexpr Key kPose = 10;
constexpr Key kPoint = 20;

class PointPoseFactor : public NoiseModelFactorN<Point2, Pose2> {
 public:
  PointPoseFactor(Key point, Key pose)
      : NoiseModelFactorN(noiseModel::Unit::Create(2), point, pose) {}

  Vector evaluateError(const Point2& point, const Pose2& pose,
                       Matrix* Hpoint, Matrix* Hpose) const override {
    if (Hpoint) *Hpoint = Matrix::Identity(2, 2);
    if (Hpose) {
      *Hpose = Matrix::Zero(2, 3);
      Hpose->block<2, 2>(0, 0) = -Matrix::Identity(2, 2);
    }
    return point - pose.translation();
  }
};

Values MakeValues() {
  Values values;
  values.insert(kPose, Pose2(1.0, 2.0, 0.1));
  values.insert(kPoint, Point2(4.0, 6.0));
  return values;
}

NonlinearFactorGraph MakeGraph() {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      kPose, Pose2(), noiseModel::Unit::Create(3));
  graph.emplace_shared<PointPoseFactor>(kPoint, kPose);
  return graph;
}
}  // namespace

TEST(SparseJacobianColumnLayout, AssignsNaturalScalarColumns) {
  const SparseJacobianColumnLayout layout(MakeValues());
  EXPECT_LONGS_EQUAL(5, layout.totalColumns());
  EXPECT_LONGS_EQUAL(0, layout.at(kPose).columnBegin);
  EXPECT_LONGS_EQUAL(3, layout.at(kPose).dimension);
  EXPECT_LONGS_EQUAL(3, layout.at(kPoint).columnBegin);
  EXPECT_LONGS_EQUAL(2, layout.at(kPoint).dimension);
}
```

- [ ] **Step 2: Reconfigure, then verify the missing type fails the build**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian -j2
```

Expected: compilation fails because `gtsam/nonlinear/cuda/SparseJacobianPlan.h` does not exist.

- [ ] **Step 3: Define the exact symbolic types**

Create `SparseJacobianPlan.h` with these public internal contracts:

```cpp
#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gtsam::cuda {

struct SparseJacobianColumnBlock {
  Key key = 0;
  int dimension = 0;
  int columnBegin = 0;
};

class SparseJacobianColumnLayout {
 public:
  explicit SparseJacobianColumnLayout(const Values& values);

  const SparseJacobianColumnBlock& at(Key key) const;
  const std::vector<SparseJacobianColumnBlock>& blocks() const;
  int totalColumns() const;
  bool matches(const Values& values) const;
  VectorValues toVectorValues(const Vector& flatDelta) const;

 private:
  std::vector<SparseJacobianColumnBlock> blocks_;
  FastMap<Key, size_t> keyToBlock_;
  int totalColumns_ = 0;
};

struct SparseJacobianBlockWritePlan {
  Key key = 0;
  size_t localBlockIndex = 0;
  int width = 0;
  int globalColumnBegin = 0;
  int valueOffsetWithinRow = 0;
};

struct SparseJacobianFactorWritePlan {
  int rowBegin = 0;
  int rowCount = 0;
  int nonzerosPerRow = 0;
  bool sendable = true;
  std::vector<SparseJacobianBlockWritePlan> blocks;
};

class SparseJacobianPlan {
 public:
  SparseJacobianPlan(const NonlinearFactorGraph& graph,
                     const SparseJacobianColumnLayout& columns);

  int rows() const;
  int columns() const;
  int nonzeros() const;
  const std::vector<int>& rowPointers() const;
  const std::vector<int>& columnIndices() const;
  const SparseJacobianFactorWritePlan& factor(size_t index) const;
  const std::vector<SparseJacobianFactorWritePlan>& factors() const;
  uint64_t structuralFingerprint() const;
  bool matches(const NonlinearFactorGraph& graph,
               const SparseJacobianColumnLayout& columns) const;

 private:
  int rows_ = 0;
  int columns_ = 0;
  std::vector<int> rowPointers_{0};
  std::vector<int> columnIndices_;
  std::vector<SparseJacobianFactorWritePlan> factors_;
  uint64_t structuralFingerprint_ = 0;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Implement checked column construction and flat-delta conversion**

In `SparseJacobianPlan.cpp`, iterate over `values.dims()` in its natural key order. Reject zero dimensions, duplicate insertion, scalar offsets beyond `INT_MAX`, missing flat-delta entries, and unknown keys. Implement conversion without consulting nonlinear value types:

```cpp
VectorValues SparseJacobianColumnLayout::toVectorValues(
    const Vector& flatDelta) const {
  if (flatDelta.size() != totalColumns_) {
    throw std::invalid_argument(
        "SparseJacobianColumnLayout delta dimension mismatch");
  }
  VectorValues result;
  for (const auto& block : blocks_) {
    result.insert(block.key,
                  flatDelta.segment(block.columnBegin, block.dimension));
  }
  return result;
}
```

Implement `matches()` by comparing the complete ordered `(key, dimension)` sequence, not just total dimension.

- [ ] **Step 5: Implement factor and block write-plan construction**

For each graph slot, assign an exclusive row range. For non-null factors, build one block per key, reject repeated or missing keys, sort block indices by `globalColumnBegin`, assign `valueOffsetWithinRow`, append sorted scalar columns once per residual row, and check every count before converting to `int`.

Track whether every scalar column belongs to at least one non-null factor that emits at least one row. Mark a factor's block columns as covered only when `factorPlan.rowCount > 0`; merely mentioning a key in a zero-row factor emits no CSR entries. The first prototype must reject a layout containing an uncovered variable with `std::invalid_argument` naming its key. Otherwise `J^T J` has no structural diagonal for that variable and the existing cuDSS wrapper cannot apply LM damping without first augmenting the Hessian pattern. The optimizer catches this setup error as `PlanIncompatible` and restarts CPU LM; augmenting `pattern(J^T J)` with the identity is a documented follow-up.

The core construction must follow this shape:

```cpp
for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
  SparseJacobianFactorWritePlan factorPlan;
  factorPlan.rowBegin = rows_;
  const auto& factor = graph[factorIndex];
  if (!factor) {
    factors_.push_back(std::move(factorPlan));
    continue;
  }

  factorPlan.rowCount = CheckedInt(factor->dim(), "factor rows");
  factorPlan.sendable = factor->sendable();

  FastSet<Key> seen;
  for (size_t local = 0; local < factor->size(); ++local) {
    const Key key = factor->keys()[local];
    if (!seen.insert(key).second) {
      throw std::invalid_argument("SparseJacobianPlan repeated factor key");
    }
    const auto& column = columns.at(key);
    factorPlan.blocks.push_back({key, local, column.dimension,
                                 column.columnBegin, 0});
  }

  std::vector<size_t> globalOrder(factorPlan.blocks.size());
  std::iota(globalOrder.begin(), globalOrder.end(), 0);
  std::sort(globalOrder.begin(), globalOrder.end(), [&](size_t a, size_t b) {
    return factorPlan.blocks[a].globalColumnBegin <
           factorPlan.blocks[b].globalColumnBegin;
  });

  int offset = 0;
  for (size_t blockIndex : globalOrder) {
    auto& block = factorPlan.blocks[blockIndex];
    block.valueOffsetWithinRow = offset;
    offset = CheckedAdd(offset, block.width, "factor row nonzeros");
  }
  factorPlan.nonzerosPerRow = offset;

  for (int row = 0; row < factorPlan.rowCount; ++row) {
    for (size_t blockIndex : globalOrder) {
      const auto& block = factorPlan.blocks[blockIndex];
      for (int c = 0; c < block.width; ++c) {
        columnIndices_.push_back(block.globalColumnBegin + c);
      }
    }
    rowPointers_.push_back(CheckedInt(columnIndices_.size(), "Jacobian nnz"));
  }

  rows_ = CheckedAdd(rows_, factorPlan.rowCount, "Jacobian rows");
  factors_.push_back(std::move(factorPlan));
}
```

Use a 64-bit FNV-1a helper over column blocks, graph null markers, factor row counts, keys, dimensions, and `sendable()` to populate `structuralFingerprint_`. `matches()` uses the fingerprint as a fast rejection and then directly compares every graph slot's nullness, row count, key sequence, key dimension, and sendable flag; the hash is never the only compatibility check.

- [ ] **Step 6: Add exact CSR and block-routing assertions**

Extend the test with:

```cpp
TEST(SparseJacobianPlan, AssignsRowsAndReordersLocalBlocks) {
  const Values values = MakeValues();
  const NonlinearFactorGraph graph = MakeGraph();
  const SparseJacobianColumnLayout layout(values);
  const SparseJacobianPlan plan(graph, layout);

  EXPECT_LONGS_EQUAL(5, plan.rows());
  EXPECT_LONGS_EQUAL(5, plan.columns());
  EXPECT_LONGS_EQUAL(19, plan.nonzeros());
  CHECK(plan.rowPointers() == std::vector<int>({0, 3, 6, 9, 14, 19}));
  CHECK(plan.columnIndices() == std::vector<int>({
      0, 1, 2, 0, 1, 2, 0, 1, 2,
      0, 1, 2, 3, 4, 0, 1, 2, 3, 4}));

  const auto& mixed = plan.factor(1);
  EXPECT_LONGS_EQUAL(3, mixed.rowBegin);
  EXPECT_LONGS_EQUAL(2, mixed.rowCount);
  EXPECT_LONGS_EQUAL(5, mixed.nonzerosPerRow);
  EXPECT_LONGS_EQUAL(3, mixed.blocks.at(0).valueOffsetWithinRow);
  EXPECT_LONGS_EQUAL(0, mixed.blocks.at(1).valueOffsetWithinRow);
}
```

Also test missing factor keys, repeated factor keys, an uncovered value key, a key mentioned only by a zero-row factor, changed value dimensions, flat delta of the wrong length, and a null graph slot with zero reserved rows.

- [ ] **Step 7: Build and run the symbolic tests**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run -j2
```

Expected: reconfiguration discovers `SparseJacobianPlan.cpp`; every `SparseJacobianColumnLayout` and `SparseJacobianPlan` test passes.

- [ ] **Step 8: Commit the symbolic layer**

```bash
git add gtsam/nonlinear/cuda/SparseJacobianPlan.h \
        gtsam/nonlinear/cuda/SparseJacobianPlan.cpp \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp \
        gtsam/nonlinear/tests/CMakeLists.txt
git commit -m "feat: add sparse Jacobian assembly plan"
```

---

### Task 2: Add stable pinned host numerical storage

**Files:**
- Create: `gtsam/base/cuda/CudaPinnedHostArray.h`
- Create: `gtsam/nonlinear/cuda/HostSparseJacobian.h`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Write failing move, zeroing, and stable-address tests**

Add tests that construct a plan, create host storage, write nonzero values, save both pointers, call `clear()`, and verify size, zero contents, and unchanged addresses:

```cpp
TEST(HostSparseJacobian, ClearsWithoutChangingStorage) {
  const Values values = MakeValues();
  const NonlinearFactorGraph graph = MakeGraph();
  const SparseJacobianPlan plan(graph, SparseJacobianColumnLayout(values));
  HostSparseJacobian host(plan);

  std::fill(host.valuesData(), host.valuesData() + host.valuesSize(), 3.0);
  std::fill(host.rhsData(), host.rhsData() + host.rhsSize(), 4.0);
  double* valuesAddress = host.valuesData();
  double* rhsAddress = host.rhsData();

  host.clear();

  CHECK(valuesAddress == host.valuesData());
  CHECK(rhsAddress == host.rhsData());
  for (size_t i = 0; i < host.valuesSize(); ++i)
    DOUBLES_EQUAL(0.0, host.valuesData()[i], 0.0);
  for (size_t i = 0; i < host.rhsSize(); ++i)
    DOUBLES_EQUAL(0.0, host.rhsData()[i], 0.0);
}
```

- [ ] **Step 2: Verify the missing host types fail the build**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian -j2
```

Expected: compilation fails because `HostSparseJacobian` and `CudaPinnedHostArray` are undefined.

- [ ] **Step 3: Implement move-only pinned allocation**

Model `CudaPinnedHostArray<T>` after `CudaDeviceArray<T>`, using `cudaMallocHost` and `cudaFreeHost`. Provide only the operations required by this prototype:

```cpp
template <typename T>
class CudaPinnedHostArray {
 public:
  CudaPinnedHostArray() = default;
  explicit CudaPinnedHostArray(size_t size) { resize(size); }
  ~CudaPinnedHostArray() { resetUnchecked(); }

  CudaPinnedHostArray(const CudaPinnedHostArray&) = delete;
  CudaPinnedHostArray& operator=(const CudaPinnedHostArray&) = delete;

  CudaPinnedHostArray(CudaPinnedHostArray&& other) noexcept
      : data_(other.data_), size_(other.size_) {
    other.data_ = nullptr;
    other.size_ = 0;
  }

  CudaPinnedHostArray& operator=(CudaPinnedHostArray&& other) noexcept {
    if (this == &other) return *this;
    resetUnchecked();
    data_ = other.data_;
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  void resize(size_t size);
  void clear() {
    if (size_ != 0) std::fill(data_, data_ + size_, T{});
  }
  T* data() { return data_; }
  const T* data() const { return data_; }
  size_t size() const { return size_; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;
  void resetUnchecked() noexcept;
};
```

`resize()` must allocate the replacement before releasing the old pointer so allocation failure leaves the object valid. Reject `sizeof(T) * size` overflow.

- [ ] **Step 4: Implement the two-buffer host object**

Create `HostSparseJacobian.h` as a small header-only owner:

```cpp
class HostSparseJacobian {
 public:
  explicit HostSparseJacobian(const SparseJacobianPlan& plan)
      : values_(static_cast<size_t>(plan.nonzeros())),
        rhs_(static_cast<size_t>(plan.rows())) {
    clear();
  }

  void clear() {
    values_.clear();
    rhs_.clear();
  }

  double* valuesData() { return values_.data(); }
  const double* valuesData() const { return values_.data(); }
  double* rhsData() { return rhs_.data(); }
  const double* rhsData() const { return rhs_.data(); }
  size_t valuesSize() const { return values_.size(); }
  size_t rhsSize() const { return rhs_.size(); }

 private:
  CudaPinnedHostArray<double> values_;
  CudaPinnedHostArray<double> rhs_;
};
```

- [ ] **Step 5: Run the host-storage tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run -j2
```

Expected: symbolic and host-storage tests pass; no CUDA memory leak is reported by process teardown.

- [ ] **Step 6: Commit host storage**

```bash
git add gtsam/base/cuda/CudaPinnedHostArray.h \
        gtsam/nonlinear/cuda/HostSparseJacobian.h \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp
git commit -m "feat: add pinned sparse Jacobian host buffers"
```

---

### Task 3: Implement serial streaming from temporary Jacobian factors

**Files:**
- Create: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h`
- Create: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Write a failing dense-reference test**

Add a test helper that converts CSR to a dense matrix using only `rowPointers`, `columnIndices`, and host numeric values. Compare the streamed result with an independently assembled reference from `graph.linearize(values)`, each returned factor's keys, and the column layout:

```cpp
TEST(StreamingSparseJacobianLinearizer, MatchesGaussianGraphReference) {
  const Values values = MakeValues();
  const NonlinearFactorGraph graph = MakeGraph();
  const SparseJacobianColumnLayout layout(values);
  const SparseJacobianPlan plan(graph, layout);
  HostSparseJacobian host(plan);

  host.clear();
  const DirectJacobianStatus status =
      StreamingSparseJacobianLinearizer().linearize(
          graph, values, layout, plan, &host);
  CHECK(status.ok());

  const Matrix actualJ = DenseFromCsr(plan, host);
  const Vector actualB = VectorFromHostRhs(host);
  const auto [expectedJ, expectedB] =
      DenseReferenceFromGaussianGraph(graph, values, layout);
  EXPECT(assert_equal(expectedJ, actualJ, 1e-12));
  EXPECT(assert_equal(expectedB, actualB, 1e-12));
}
```

Make one test noise model diagonal and one robust so the reference proves that existing `NoiseModelFactor::linearize()` whitening semantics are retained.

- [ ] **Step 2: Verify the missing linearizer fails the build**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian -j2
```

Expected: compilation fails because the streaming-linearizer header and status types do not exist.

- [ ] **Step 3: Define expected compatibility outcomes**

Create `StreamingSparseJacobianLinearizer.h` with:

```cpp
enum class DirectJacobianFailure {
  None,
  StructuralMismatch,
  UnsupportedGaussianFactor,
  ConstrainedFactor,
  NonFiniteValues,
};

struct DirectJacobianStatus {
  DirectJacobianFailure failure = DirectJacobianFailure::None;
  size_t factorIndex = std::numeric_limits<size_t>::max();
  std::string detail;
  bool ok() const { return failure == DirectJacobianFailure::None; }
};

class StreamingSparseJacobianLinearizer {
 public:
  DirectJacobianStatus linearize(
      const NonlinearFactorGraph& graph, const Values& values,
      const SparseJacobianColumnLayout& columns,
      const SparseJacobianPlan& plan, HostSparseJacobian* output,
      bool validateStructure = true) const;

  DirectJacobianStatus packGaussianFactorGraph(
      const GaussianFactorGraph& linear,
      const SparseJacobianPlan& plan,
      HostSparseJacobian* output) const;
};
```

`packGaussianFactorGraph()` exists only for reference tests and the diagnostic benchmark; it must call the same private scatter helper as the streaming path.

- [ ] **Step 4: Implement one-factor validation and copying**

Implement a private `ScatterOneFactor` that receives the graph index and a `shared_ptr<GaussianFactor>`. Its required behavior is:

```cpp
if (!gaussian) return Success();

const auto* source = dynamic_cast<const JacobianFactor*>(gaussian.get());
if (!source)
  return Failure(UnsupportedGaussianFactor, factorIndex,
                 "linearize() did not return JacobianFactor");
if (source->isConstrained())
  return Failure(ConstrainedFactor, factorIndex,
                 "constrained Jacobian rows require QR semantics");

std::optional<JacobianFactor> whitened;
if (source->get_model() && !source->get_model()->isUnit()) {
  whitened.emplace(source->whiten());
  source = &*whitened;
}
```

Validate row count, key count, key at every `localBlockIndex`, block width, block rows, RHS size, and every coefficient's finiteness in a first pass. Only after the complete factor passes validation, copy in a second pass with the precomputed destination formula:

```cpp
for (const auto& block : factorPlan.blocks) {
  const auto A = source->getA(source->begin() + block.localBlockIndex);
  if (source->keys()[block.localBlockIndex] != block.key ||
      A.rows() != factorPlan.rowCount || A.cols() != block.width) {
    return Failure(DirectJacobianFailure::StructuralMismatch, factorIndex,
                   "Jacobian block does not match symbolic plan");
  }
  if (!A.allFinite()) {
    return Failure(DirectJacobianFailure::NonFiniteValues, factorIndex,
                   "Jacobian block contains non-finite values");
  }
}
const auto b = source->getb();
if (b.size() != factorPlan.rowCount) {
  return Failure(DirectJacobianFailure::StructuralMismatch, factorIndex,
                 "Jacobian RHS does not match symbolic plan");
}
if (!b.allFinite()) {
  return Failure(DirectJacobianFailure::NonFiniteValues, factorIndex,
                 "Jacobian RHS contains non-finite values");
}

for (const auto& block : factorPlan.blocks) {
  const auto A = source->getA(source->begin() + block.localBlockIndex);
  for (int localRow = 0; localRow < factorPlan.rowCount; ++localRow) {
    const int globalRow = factorPlan.rowBegin + localRow;
    const int destination = plan.rowPointers()[globalRow] +
                            block.valueOffsetWithinRow;
    for (int c = 0; c < block.width; ++c)
      output->valuesData()[destination + c] = A(localRow, c);
  }
}
for (int r = 0; r < factorPlan.rowCount; ++r)
  output->rhsData()[factorPlan.rowBegin + r] = b(r);
```

Do all validation before writing the factor's rows, so a structural failure cannot leave a partially updated factor range.

- [ ] **Step 5: Implement the initial serial graph traversal**

`linearize()` must reject a null or incorrectly sized output and process graph slots in index order. Add a final `bool validateStructure = true` argument. When it is true, require both `columns.matches(values)` and `plan.matches(graph, columns)` before evaluating factors. When it is false, skip that deep topology/dimension scan; do not reconstruct a column layout internally. Validation of every returned Gaussian factor's row count, key order, block dimensions, and finite coefficients remains mandatory in both modes. The caller must call `output->clear()` immediately before this method; keeping zeroing outside the linearizer makes `hostZero` independently measurable and avoids accidentally zeroing twice.

```cpp
for (size_t i = 0; i < graph.size(); ++i) {
  if (!graph[i]) continue;
  const auto gaussian = graph[i]->linearize(values);
  const auto status = ScatterOneFactor(i, gaussian, plan, output);
  if (!status.ok()) return status;
}
return {};
```

An inactive factor returns null; its reserved rows remain zero because every call site clears the complete host buffer first. Tests must explicitly call `host.clear()` immediately before both `linearize()` and `packGaussianFactorGraph()`.

- [ ] **Step 6: Add inactive and malformed-result tests**

Define test-only factors whose `active()` returns false and whose `linearize()` returns an intentionally wrong row count. Assert inactive rows are zero and malformed results return `StructuralMismatch` with the correct factor index without throwing.

- [ ] **Step 7: Run the streaming tests**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run -j2
```

Expected: reconfiguration discovers `StreamingSparseJacobianLinearizer.cpp`; streamed dense `J,b` matches the ordinary `GaussianFactorGraph` reference for unit, diagonal, and robust noise; inactive and malformed tests pass.

- [ ] **Step 8: Commit serial streaming**

```bash
git add gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h \
        gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp
git commit -m "feat: stream Jacobian factors into cached CSR"
```

---

### Task 4: Add TBB scheduling and complete compatibility fallback detection

**Files:**
- Modify: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h`
- Modify: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Write failing scheduling-classification and non-sendable execution tests**

Add test-only factor classes that override `linearize()` to record call count and thread ID. Add at least 512 sendable factors and one `sendable() == false` factor. Add an optional `StreamingLinearizationStats*` output containing `sendableFactors` and `nonSendableFactors`; assert every factor is evaluated once, classification counts are exact, and the non-sendable factor runs on the calling thread. Do not assert that a worker thread must appear, because a valid one-core TBB runtime may execute all sendable work on the caller.

The essential assertions are:

```cpp
const std::thread::id caller = std::this_thread::get_id();
StreamingLinearizationStats stats;
const DirectJacobianStatus status = linearizer.linearize(
    graph, values, layout, plan, &host, &stats);
CHECK(status.ok());
EXPECT_LONGS_EQUAL(512, stats.sendableFactors);
EXPECT_LONGS_EQUAL(1, stats.nonSendableFactors);
EXPECT_LONGS_EQUAL(1, nonSendable->linearizeCalls());
CHECK(nonSendable->linearizeThread() == caller);
```

- [ ] **Step 2: Run the focused test and verify the missing stats API fails**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run -j2
```

Expected: compilation fails because `StreamingLinearizationStats` and the stats argument do not exist.

- [ ] **Step 3: Mirror `NonlinearFactorGraph::linearize()` scheduling**

Define `StreamingLinearizationStats { size_t sendableFactors = 0; size_t nonSendableFactors = 0; }` and evolve the signature to:

```cpp
DirectJacobianStatus linearize(
    const NonlinearFactorGraph& graph, const Values& values,
    const SparseJacobianColumnLayout& columns,
    const SparseJacobianPlan& plan, HostSparseJacobian* output,
    StreamingLinearizationStats* stats = nullptr,
    bool validateStructure = true) const;
```

Compute the counts deterministically from the plan before launching work. Under `GTSAM_USE_TBB`, allocate one status slot per graph factor and run sendable slots through `tbb::parallel_for`. Each task writes only its factor's status slot and preassigned numerical rows. Process all non-sendable factors serially in graph order, then scan all status slots from index zero and return the lowest-index failure deterministically.

Use the same mixed-thread limiter as GTSAM:

```cpp
TbbOpenMPMixedScope threadLimiter;
tbb::parallel_for(tbb::blocked_range<size_t>(0, graph.size()),
                  [&](const tbb::blocked_range<size_t>& range) {
  for (size_t i = range.begin(); i != range.end(); ++i) {
    if (!graph[i] || !plan.factor(i).sendable) continue;
    auto gaussian = graph[i]->linearize(values);
    statuses[i] = ScatterOneFactor(i, gaussian, plan, output);
  }
});
```

When TBB is unavailable, retain the serial traversal and identical output.

- [ ] **Step 4: Add explicit unsupported-factor tests**

Add a `NonlinearFactor` that returns a `HessianFactor`, and a constrained factor that returns a constrained `JacobianFactor`. Assert:

```text
Hessian result     → UnsupportedGaussianFactor
constrained result → ConstrainedFactor
NaN in A or b      → NonFiniteValues
```

Also assert `packGaussianFactorGraph()` reports the same statuses as streaming the corresponding graph.

- [ ] **Step 5: Run with TBB and run the existing nonlinear graph tests**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run testNonlinearFactorGraph.run -j2
```

Expected: all tests pass and the non-sendable factor runs exactly once on the caller thread.

- [ ] **Step 6: Commit parallel streaming and fallback detection**

```bash
git add gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h \
        gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp
git commit -m "feat: parallelize sparse Jacobian streaming"
```

---

### Task 5: Form undamped normal equations with cuSPARSE

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h`
- Create: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu`
- Modify: `gtsam/CMakeLists.txt`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`

- [ ] **Step 1: Write all failing device normal-equation behavior tests**

Before changing CMake or adding the device class, write the complete Task 5 behavior suite. Use `MakeGraph()`, the streaming linearizer, and Eigen for two distinct numerical states with one device initialization:

```cpp
TEST(DeviceSparseJacobianNormalEquations,
     RepeatedFormsMatchEigenAndPreserveFinalStorage) {
  CudaContext context;
  const Values first = MakeValues();
  Values second = first;
  second.update(kPose, Pose2(1.2, 1.8, -0.2));
  second.update(kPoint, Point2(4.5, 5.5));
  const NonlinearFactorGraph graph = MakeGraph();
  const SparseJacobianColumnLayout layout(first);
  const SparseJacobianPlan plan(graph, layout);
  HostSparseJacobian host(plan);

  DeviceSparseJacobianNormalEquations device;
  device.initialize(plan, context.stream());

  const int* const rowAddress = device.system().rowPointers().data();
  const int* const colAddress = device.system().colIndices().data();
  const double* const valueAddress = device.system().values().data();
  const double* const rhsAddress = device.system().rhs().data();
  const auto initialPattern =
      DownloadCsrPattern(device.system(), context.stream());

  for (const Values* numericalValues : {&first, &second}) {
    host.clear();
    CHECK(StreamingSparseJacobianLinearizer()
              .linearize(graph, *numericalValues, layout, plan, &host)
              .ok());
    const Matrix J = DenseFromCsr(plan, host);
    const Vector b = VectorFromHostRhs(host);

    device.uploadNumerics(host, context.stream());
    device.formUndampedSystem(context.stream());
    const auto [actualH, actualG] =
        DownloadNormalSystem(device.system(), context.stream());

    EXPECT(assert_equal(J.transpose() * J, actualH, 1e-10));
    EXPECT(assert_equal(J.transpose() * b, actualG, 1e-10));
    CHECK(rowAddress == device.system().rowPointers().data());
    CHECK(colAddress == device.system().colIndices().data());
    CHECK(valueAddress == device.system().values().data());
    CHECK(rhsAddress == device.system().rhs().data());
    CHECK(initialPattern ==
          DownloadCsrPattern(device.system(), context.stream()));
  }
}
```

Define `DownloadCsrPattern()` in the test file to download both row offsets and column indices, synchronize the supplied stream, and return `std::pair<std::vector<int>, std::vector<int>>`. Add `RejectsEmptyRowsColumnsAndNonzeros`, which builds the empty graph/empty-`Values` plan and requires `initialize()` to throw `std::invalid_argument` whose diagnostic identifies that rows, columns, and nnz must all be positive.

Also add `ReportsConfiguredSpGemmCapability` before implementation:

```cpp
TEST(DeviceSparseJacobianNormalEquations, ReportsConfiguredSpGemmCapability) {
  const auto capability =
      DeviceSparseJacobianNormalEquations::preflightCapability();
#if GTSAM_TEST_EXPECT_SPGEMM_REUSE
  CHECK(capability.supported);
#else
  CHECK(!capability.supported);
  CHECK(!capability.detail.empty());
#endif
}
```

The normal installed build defines `GTSAM_TEST_EXPECT_SPGEMM_REUSE=1`. A build where symbol detection or the fallback version guard disables reuse defines it as `0`, and this same pre-implementation test source proves capability false without entering setup.

- [ ] **Step 2: Run the complete behavior suite and observe RED**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian -j2
```

Expected: compilation fails because `DeviceSparseJacobianNormalEquations`, `preflightCapability()`, and the device normal-equation API do not exist. This is the RED result for repeated numerical parity, stable final storage/pattern, empty-input rejection, and both configured capability branches.

- [ ] **Step 3: Link cuSPARSE explicitly**

Inside `if(GTSAM_ENABLE_CUDA)` in `gtsam/CMakeLists.txt`, add:

```cmake
target_link_libraries(gtsam PUBLIC CUDA::cusparse)
```

Keep cuDSS conditional exactly as it is. Prefer a CMake compile/symbol probe that includes `<cusparse.h>` and takes the address of `cusparseSpGEMMreuse_workEstimation`; use its result to define the private implementation capability `GTSAM_CUSPARSE_HAS_SPGEMM_REUSE` and the matching test-target definition `GTSAM_TEST_EXPECT_SPGEMM_REUSE`. A symbol probe is preferable to a version comparison because SpGEMM-reuse is deprecated and a future cuSPARSE can remove the declarations while retaining a higher version number.

If the build cannot use a symbol probe, the `.cu` fallback guard must be based on the cuSPARSE header version, never `CUDA_VERSION`, with this minimum declaration guard:

```cpp
#if defined(CUSPARSE_VERSION) && CUSPARSE_VERSION >= 11600
#define GTSAM_CUSPARSE_HAS_SPGEMM_REUSE 1
#else
#define GTSAM_CUSPARSE_HAS_SPGEMM_REUSE 0
#endif
```

The installed CUDA 13.0.3 environment has cuSPARSE 12.6.3 (`CUSPARSE_VERSION == 12603`), so the capability is available here.

- [ ] **Step 4: Define the move-only PIMPL interface**

Create this internal device-component interface and capability result:

```cpp
struct DeviceSparseNormalEquationCapability {
  bool supported = false;
  std::string detail;
};

class DeviceSparseJacobianNormalEquations {
 public:
  DeviceSparseJacobianNormalEquations();
  ~DeviceSparseJacobianNormalEquations();
  DeviceSparseJacobianNormalEquations(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations& operator=(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations(
      DeviceSparseJacobianNormalEquations&&) noexcept;
  DeviceSparseJacobianNormalEquations& operator=(
      DeviceSparseJacobianNormalEquations&&) noexcept;

  static DeviceSparseNormalEquationCapability preflightCapability();

  void initialize(const SparseJacobianPlan& plan,
                  cudaStream_t stream = nullptr);
  void uploadNumerics(const HostSparseJacobian& host,
                      cudaStream_t stream = nullptr);
  void formUndampedSystem(cudaStream_t stream = nullptr);

  const DeviceSparseNormalEquations& system() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
```

`preflightCapability()` reports whether this compiled device component can form fixed-pattern normal equations and supplies an internal diagnostic. Task 7 maps an unsupported result to `CudaToolkitUnsupported`; do not add a cuSPARSE strategy selector, capability flag, or reuse-specific type to the public optimizer parameters or result.

- [ ] **Step 5: Implement RAII, one-stream ownership, and persistent `J`/`J^T`**

In the `.cu` PIMPL, own a `cusparseHandle_t`, sparse descriptors for `J`, `J^T`, and the stable final `H`, dense-vector descriptors for `b` and `g`, one persistent final `cusparseSpGEMMDescr_t`, and five SpGEMM-reuse workspaces in `CudaDeviceArray<std::byte>`. Retain a separate CSR-to-CSC workspace and a separate SpMV workspace; neither may alias any of the five reuse buffers. The one SpMV workspace is sized to the maximum buffer-size query for the `J^T * b` operation here and, after Task 6 extends initialization, the `J * delta` operation. Every destructor must tolerate partially constructed state.

`initialize()` rejects zero or negative Jacobian row/column dimensions and zero `J.nnz` before making a CUDA call. It fixes the component to its supplied stream, calls `cusparseSetStream(handle, stream)`, calls `cusparseSetPointerMode(handle, CUSPARSE_POINTER_MODE_HOST)`, and stores that stream. Every later public method rejects a different stream. Consequently, asynchronous uploads, CSR-to-CSC, SpGEMM, SpMV, CUB, custom kernels, cuDSS, events, downloads, the cuDSS factorization-info boundary added in Task 6, and final result downloads are ordered on the same stream. Host stack `alpha` and `beta` scalars are valid only with this explicit host pointer mode.

`initialize()` must:

1. Upload `J.rowPointers` and `J.columnIndices` once.
2. Allocate `J.values`, `b`, `J^T.rowPointers`, `J^T.columnIndices`, and `J^T.values` once.
3. Bitwise-zero `J.values` before the setup conversion and create `J` as `rows × columns` CSR.
4. Query and allocate the dedicated CSR-to-CSC workspace, then call `cusparseCsr2cscEx2` once to establish the transpose structure.
5. Treat CSC arrays of `J` as CSR arrays of `J^T` with shape `columns × rows` and query/allocate the dedicated SpMV workspace.
6. Capture every persistent `J`, `J^T`, and final-system device pointer after setup; no later numeric path may resize or replace those allocations.

`uploadNumerics()` must verify host sizes and use two `cudaMemcpyAsync` operations into the persistent `J.values` and `b` allocations. It must not resize device arrays after initialization.

Any path that has queued work and is about to destroy a descriptor or workspace because of an exception must synchronize the owned stream first. Structure temporary setup state so its catch handler still owns that state, preserve the original exception, perform a best-effort `cudaStreamSynchronize(stream)`, destroy/reset the queued descriptors and workspaces only after that synchronization, and then rethrow the original exception. Apply the same rule to a failed public numeric operation before unwinding can destroy the PIMPL; destructors remain non-throwing.

- [ ] **Step 6: Establish stable `H` storage with the documented SpGEMM-reuse lifecycle**

Use two complete, documented SpGEMM-reuse lifecycles for `J^T * J`; never run copy against one allocation and then rebind that consumed output descriptor to another allocation.

Reference the [CUDA 13.0 cuSPARSE SpGEMM-reuse API](https://docs.nvidia.com/cuda/archive/13.0.0/cusparse/index.html#cusparse-generic-function-spgemm-reuse) and the [official NVIDIA reuse sample](https://github.com/NVIDIA/CUDALibrarySamples/blob/main/cuSPARSE/spgemm_reuse/spgemm_reuse_example.c) while implementing this lifecycle.

1. **Discovery lifecycle:** create a temporary `H` descriptor and temporary reuse descriptor. Run `cusparseSpGEMMreuse_workEstimation()` in query/execute form, `cusparseSpGEMMreuse_nnz()` in query/execute form, query `H.nnz` as `int64_t`, bind temporary column/value storage, and run `cusparseSpGEMMreuse_copy()` in query/execute form.
2. Reject `H.nnz <= 0` and reject `H.nnz > std::numeric_limits<int>::max()` before converting it for `DeviceSparseNormalEquations`. Download the discovered `H.rowPointers` and `H.columnIndices`, synchronize the setup stream, validate the square CSR pattern and every scalar diagonal, and initialize `system_` with the host pattern. Only after that synchronization may the temporary discovery descriptor, reuse descriptor, and discovery workspaces be destroyed.
3. **Stable final lifecycle:** create a fresh `H` CSR descriptor with `nnz = 0`, `system_.rowPointers().data()` supplied from the start, and null column/value pointers, plus a fresh reuse descriptor. Run a fresh query/execute `cusparseSpGEMMreuse_workEstimation()` and a fresh query/execute `cusparseSpGEMMreuse_nnz()` against this descriptor. Query its `int64_t` nnz and require exact equality with the discovered, int-bounded `system_.nonzeros()`.
4. Before the second lifecycle's first `cusparseSpGEMMreuse_copy()` call, bind all three stable pointers at once with `cusparseCsrSetPointers(finalH, system_.rowPointers().data(), system_.colIndices().data(), system_.values().data())`. Then run `cusparseSpGEMMreuse_copy()` in query/execute form. There is no pointer rebinding after this copy.
5. Download the final row offsets and column indices, synchronize, and require exact equality with the discovered host pattern. Record the stable row-offset, column-index, value, and RHS addresses. Retain this second reuse descriptor plus its workspaces 4 and 5 for every repeated compute; release workspaces 1--3 only after the verification synchronization.

Put both lifecycles behind `GTSAM_CUSPARSE_HAS_SPGEMM_REUSE`. When it is false, `preflightCapability()` returns an unsupported result with a precise configured-cuSPARSE diagnostic and setup is not attempted. Do not fall back to an unverified ordinary-SpGEMM descriptor lifecycle.

The two setup lifecycles and pattern round trips are one-time setup timing, not per-iteration H2D/D2H timing. The stable output descriptor is born with `system_` row offsets, receives all final pointers before its copy, and is never rebound afterward.

- [ ] **Step 7: Implement repeated numeric `J^T`, `H`, and `g` updates**

For every `formUndampedSystem()`:

1. Verify on the host that every captured persistent pointer still matches before queuing work.
2. Run numeric `cusparseCsr2cscEx2` into the same stable transpose arrays. The numeric conversion rewrites `J^T.rowPointers`, `J^T.columnIndices`, and `J^T.values`; it does not update values alone.
3. With host pointer mode already set, create host stack scalars `alpha = 1` and `beta = 0`, then call only `cusparseSpGEMMreuse_compute()` for `J^T * J` into stable `system_.values()`.
4. Run `cusparseSpMV` for `J^T * b` into stable `system_.rhs()` using the dedicated SpMV workspace.
5. Check launch/API status without a normal-path internal synchronization; the caller's stage boundary owns normal synchronization. If an exception will unwind and destroy queued resources, use the exceptional-path synchronization rule from Step 5 first.

Thus every repeated form has exactly this operation order: numeric CSR-to-CSC (structure and values), reuse-compute, then SpMV. Do not call ordinary `cusparseSpGEMM_compute/copy` repeatedly, and do not call `cusparseCsrSetPointers()` on the final `H` after its setup copy.

- [ ] **Step 8: Re-run the Task 5 behavior suite**

Do not add or extend behavioral tests here; Step 1 already contains the two-state Eigen parity, final pointer/pattern stability, empty-input rejection, and configured capability cases. Re-run that unchanged suite against the implementation.

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run -j2
```

Expected: reconfiguration discovers the new `.cu` source and cuSPARSE link; every Step 1 test passes unchanged. Both repeated normal systems match Eigen within `1e-10`, the final CSR pattern and all four final pointers remain stable, empty input is rejected, and the installed capability branch reports supported. When the configured build permits the unsupported variant, the same prewritten capability test reports false.

- [ ] **Step 9: Commit cuSPARSE normal equations**

```bash
git add gtsam/CMakeLists.txt \
        gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h \
        gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp
git commit -m "feat: form sparse normal equations with cuSPARSE"
```

---

### Task 6: Add reusable damping, cuDSS solving, and model-error evaluation

**Files:**
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu`
- Modify: `gtsam/nonlinear/cuda/CudssLinearSolver.h`
- Modify: `gtsam/nonlinear/cuda/CudssLinearSolver.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseJacobian.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Write a failing non-SPD cuDSS factorization-info test**

The existing `CudssSpdSolver` checks only the host return status from `cudssExecute`; a successful host status does not prove that numerical factorization was positive definite. In `testCudaDeviceValues.cpp`, retain `SolvesSmallSpdSystem` and `ReusesAnalysisForChangedValues` unchanged and add an indefinite upper-CSR case:

```cpp
TEST(CudssSpdSolver, SurfacesIndefiniteFactorizationInfo) {
  CudaContext context;
  DeviceSparseNormalEquations system;
  system.uploadPattern(2, std::vector<int>{0, 2, 3},
                       std::vector<int>{0, 1, 1}, context.stream());
  system.values().upload(std::vector<double>{1.0, 2.0, 1.0},
                         context.stream());
  system.rhs().upload(std::vector<double>{1.0, 1.0}, context.stream());

  CudaDeviceArray<double> solution;
  CudssSpdSolver solver;
  solver.analyze(system, &solution, context.stream());
  try {
    solver.solve(system, &solution, context.stream());
    CHECK(false);
  } catch (const std::runtime_error& e) {
    const std::string message = e.what();
    CHECK(message.find("CUDSS_DATA_INFO") != std::string::npos);
    CHECK(message.find("first non-positive minor") != std::string::npos);
  }
}
```

- [ ] **Step 2: Run the new wrapper test and verify it fails before changing the wrapper**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaDeviceValues.run -j2
```

Expected: the new indefinite test fails because the current wrapper does not query or surface `CUDSS_DATA_INFO`; the two existing successful solve tests remain passing.

- [ ] **Step 3: Query and enforce cuDSS numerical factorization info**

Keep the `CudssSpdSolver` public signatures unchanged, and document in `CudssLinearSolver.h` that `solve()` throws when numerical factorization reports a non-positive minor. In `CudssLinearSolver.cpp`, immediately after the successful `CUDSS_PHASE_FACTORIZATION` `cudssExecute()` call and before any `CUDSS_PHASE_SOLVE` call, add the cuDSS 0.8.0 correctness query:

```cpp
int info = 0;
size_t bytesWritten = 0;
GTSAM_CUDSS_CHECK(cudssDataGet(handle.value, data.value, CUDSS_DATA_INFO,
                               &info, sizeof(info), &bytesWritten));
if (bytesWritten != sizeof(info)) {
  throw std::runtime_error(
      "cuDSS CUDSS_DATA_INFO returned an unexpected byte count");
}
if (info != 0) {
  std::ostringstream os;
  os << "cuDSS factorization CUDSS_DATA_INFO=" << info
     << " (reordered 1-based first non-positive minor)";
  throw std::runtime_error(os.str());
}
```

Execute `CUDSS_PHASE_SOLVE` only after `bytesWritten == sizeof(info)` and `info == 0`. A nonzero value is the reordered, 1-based first non-positive minor and must be surfaced even when the factorization execute call returned `CUDSS_STATUS_SUCCESS`.

With cuDSS 0.8.0, this host `cudssDataGet()` correctness query introduces a factorization-info/synchronization boundary before solve. Preserve the final combined delta/errors D2H synchronization later in this task, but do not promise only one synchronization per attempt.

Re-run:

```bash
cmake --build build-cuda-cudss-on --target testCudaDeviceValues.run -j2
```

Expected: the indefinite test now passes by observing `CUDSS_DATA_INFO`, and both existing SPD solve tests still pass.

- [ ] **Step 4: Write failing solve, validation, and repeated-lambda tests**

After forming the reference (H,g), solve two lambdas without rebuilding `J` or `H`:

```cpp
device.prepareDamping(false, 1e-6, 1e32, context.stream());
device.analyze(context.stream());

for (double lambda : {0.1, 1.0}) {
  device.solveAndEvaluate(lambda, context.stream());
  const DeviceSparseJacobianAttemptResult attempt =
      device.downloadAttemptResult(context.stream());
  const Vector& actual = attempt.delta;
  const Vector expected =
      (H + lambda * Matrix::Identity(H.rows(), H.cols()))
          .llt().solve(g);
  EXPECT(assert_equal(expected, actual, 1e-9));
}
```

Add a diagonal-damping case using `diag(H).cwiseMax(min).cwiseMin(max)` and assert the second lambda does not contain damping accumulated from the first. Call `analyze()` twice before the repeated solves and require `analysisCount() == 1`.

Add explicit invalid-input tests for an uninitialized/empty system, non-finite `minDiagonal` or `maxDiagonal`, `minDiagonal > maxDiagonal`, and negative, NaN, or infinite lambda. The damping limits must both be finite and ordered; each lambda must be finite and nonnegative.

- [ ] **Step 5: Run the test and verify the missing device solve API fails**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian -j2
```

Expected: compilation fails because damping, analysis, solve, and delta download methods are absent.

- [ ] **Step 6: Add validated persistent damping and absolute writes**

During first-H setup, build and upload one `hDiagonalOffsets` entry per scalar row. Add device arrays for undamped diagonal and damping diagonal. After each SpGEMM update, gather the new undamped diagonal and compute:

```text
identity damping: D[i] = 1
diagonal damping: D[i] = clamp(H[i,i], minDiagonal, maxDiagonal)
```

For each lambda, launch a kernel that writes absolute values:

```cpp
H.values[hDiagonalOffsets[i]] =
    undampedDiagonal[i] + lambda * dampingDiagonal[i];
```

Never use additive damping across attempts; this prevents lambda accumulation.

Before preparing damping, require a nonempty initialized system (`rows > 0` and `nonzeros > 0`), `std::isfinite(minDiagonal)`, `std::isfinite(maxDiagonal)`, and `minDiagonal <= maxDiagonal`. Before launching each absolute-write kernel, require `std::isfinite(lambda) && lambda >= 0.0`. Throw `std::invalid_argument` before queuing device work for every violation.

- [ ] **Step 7: Compose the corrected reusable cuDSS solver**

Add `CudssSpdSolver solver_`, `CudaDeviceArray<double> delta_`, `bool analyzed_`, and `size_t analysisCount_` to the PIMPL. `analyze()` validates the nonempty stable system, returns immediately when `analyzed_` is already true for the same captured pointers, otherwise calls the existing solver exactly once and increments `analysisCount_` only after successful analysis. `solveAndEvaluate()` requires analysis and calls the corrected factorization-info-plus-solve operation; if `CUDSS_DATA_INFO` throws, it must not queue solve or model-error work. A second `initialize()` resets descriptors, solver state, `analyzed_`, and `analysisCount_` together.

Expose:

```cpp
void prepareDamping(bool diagonalDamping, double minDiagonal,
                    double maxDiagonal, cudaStream_t stream = nullptr);
void analyze(cudaStream_t stream = nullptr);
void solveAndEvaluate(double lambda, cudaStream_t stream = nullptr);
size_t analysisCount() const;
```

`solveAndEvaluate()` validates finite nonnegative lambda, performs the absolute damping write, cuDSS factorization, `CUDSS_DATA_INFO` query, and solve, then queues the linearized-model evaluation described in the next step. Analysis remains one-time across all outer linearizations because the system structure and every captured pointer remain stable.

- [ ] **Step 8: Implement linearized old/new error without downloading `J`**

Add persistent vectors for `J * delta`, squared row terms, separate `oldErrorDevice_` and `newErrorDevice_` scalars, and CUB reduction workspace. Extend the dedicated SpMV workspace sizing during initialization to the maximum required by both `J^T * b` and `J * delta`; it remains distinct from CSR-to-CSC and all five SpGEMM-reuse buffers. At `formUndampedSystem()`, reduce `b_i^2` into `oldErrorDevice_` and store:

```text
oldLinearizedError = 0.5 * sum_i b_i^2
```

After each cuDSS solve inside `solveAndEvaluate()`:

1. Run cuSPARSE SpMV `residual = J * delta`.
2. Launch a pointwise kernel `squared[i] = (residual[i] - b[i])^2`.
3. Reduce `squared` into `newErrorDevice_` with `cub::DeviceReduce::Sum`; never overwrite the persistent old-error scalar.
4. Copy the two scalar errors to host at the same synchronization boundary as delta.

Expose one combined result and download method:

```cpp
struct LinearizedModelErrors {
  double oldError = 0.0;
  double newError = 0.0;
  double change() const { return oldError - newError; }
};

struct DeviceSparseJacobianAttemptResult {
  Vector delta;
  LinearizedModelErrors model;
};

DeviceSparseJacobianAttemptResult downloadAttemptResult(
    cudaStream_t stream = nullptr) const;
```

`downloadAttemptResult()` queues delta and both scalar copies together, performs the final combined stream synchronization, and then constructs the Eigen/result objects. This remains the single combined result D2H boundary, but the earlier cuDSS 0.8.0 `CUDSS_DATA_INFO` query is a separate factorization correctness/synchronization boundary.

- [ ] **Step 9: Compare model errors and run cuDSS-enabled verification**

For the same graph, delta, and linearization point, assert old error, new error, and change agree with the ordinary linear graph within `1e-9`.

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseJacobian.run testCudaDeviceValues.run -j2
```

Expected: factorization-info, successful solve, one-time analysis, validation, non-accumulating damping, model-error, existing cuDSS, and existing device-normal-equation tests all pass.

- [ ] **Step 10: Verify the no-cuDSS build remains explicit**

Run:

```bash
cmake -S . -B build-cuda-no-cudss
cmake --build build-cuda-no-cudss --target testCudaDeviceValues.run -j2
```

Expected: the existing `ThrowsWhenCudssDisabled` test passes with `requires cuDSS`; DATA_INFO code and the indefinite test remain excluded by `GTSAM_ENABLE_CUDSS`, and no cuDSS symbol is required by this build.

- [ ] **Step 11: Commit the reusable GPU solve layer**

```bash
git add gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h \
        gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu \
        gtsam/nonlinear/cuda/CudssLinearSolver.h \
        gtsam/nonlinear/cuda/CudssLinearSolver.cpp \
        gtsam/nonlinear/tests/testCudaSparseJacobian.cpp \
        gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: solve damped sparse Jacobian systems on GPU"
```

---

### Task 7: Implement the public CUDA sparse LM optimizer

**Files:**
- Create: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`
- Create: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp`
- Create: `gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp`

- [ ] **Step 1: Write failing public-API, parity, preflight, and fallback tests**

Build a well-constrained nonlinear `Pose2` graph with one prior and two between factors. Run CPU and CUDA LM with the same Ceres defaults and assert final errors and values:

```cpp
TEST(CudaSparseLevenbergMarquardt, MatchesCpuPose2Result) {
  const auto [graph, initial] = MakePose2LmProblem();

  CudaSparseLevenbergMarquardtParams params;
  LevenbergMarquardtParams::SetCeresDefaults(&params);
  params.maxIterations = 20;

  const Values expected =
      LevenbergMarquardtOptimizer(graph, initial, params).optimize();
  CudaSparseLevenbergMarquardtOptimizer optimizer(graph, initial, params);
  const Values actual = optimizer.optimize();

  DOUBLES_EQUAL(graph.error(expected), graph.error(actual), 1e-8);
  EXPECT(assert_equal(expected, actual, 1e-7));
  CHECK(optimizer.result().backend == CudaSparseLmBackend::Cuda);
}
```

In the same pre-implementation step, add a `CheckCpuFallback(graph, initial, expectedReason)` test helper. It runs ordinary CPU LM from the original initial values, runs the CUDA optimizer with fallback enabled, checks value/error parity, `CpuFallback`, and the exact reason, then repeats with `fallbackOnUnsupported = false` and requires a `std::runtime_error` containing the fallback detail (and factor index/type for semantic failures).

Use that helper in tests written now for every fallback path that Task 7 implements:

```cpp
TEST(CudaSparseLevenbergMarquardt, FallsBackForPlanIncompatible) {
  auto [graph, initial] = MakePose2LmProblem();
  initial.insert(Symbol('u', 0), Point2(3.0, -2.0));
  CheckCpuFallback(graph, initial,
                   CudaSparseLmFallbackReason::PlanIncompatible);
}

TEST(CudaSparseLevenbergMarquardt,
     FallsBackForDirectJacobianSemanticFailure) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      kPose, Pose2(), noiseModel::Constrained::All(3));
  Values initial;
  initial.insert(kPose, Pose2(0.2, -0.1, 0.05));
  CheckCpuFallback(graph, initial,
                   CudaSparseLmFallbackReason::DirectJacobianUnsupported);
}

TEST(CudaSparseLevenbergMarquardt, FallsBackWhenToolkitUnsupported) {
  if (DeviceSparseJacobianNormalEquations::preflightCapability().supported)
    return;  // Installed supported build: source exists but branch is skipped.
  const auto [graph, initial] = MakePose2LmProblem();
  CheckCpuFallback(graph, initial,
                   CudaSparseLmFallbackReason::CudaToolkitUnsupported);
}

#if !GTSAM_ENABLE_CUDSS
TEST(CudaSparseLevenbergMarquardt, FallsBackWhenCudssUnavailable) {
  const auto [graph, initial] = MakePose2LmProblem();
  CheckCpuFallback(graph, initial,
                   CudaSparseLmFallbackReason::CudssUnavailable);
}
#endif
```

Do not add a production parameter, public test hook, or capability override to force environment-specific branches. The installed supported/cuDSS build conditionally skips those two environment branches; the configured unsupported-capability and no-cuDSS builds execute the same prewritten test source. The `fallbackOnUnsupported = false` half of `CheckCpuFallback` supplies the throw expectation for every branch that executes.

Also write the CPU-side `tryLambda()` reference harness and deterministic trace/termination assertions now, before the state machine exists. Include a multi-linearization case that requires `result().cudssAnalyses == 1`. Compare accepted count, attempt count, lambda before each attempt, acceptance, hook arguments, initial `errorTol` exit, small-cost termination, and lambda-upper-bound termination.

- [ ] **Step 2: Verify the missing optimizer fails the build**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseLevenbergMarquardt -j2
```

Expected: compilation fails because the optimizer, fallback enums/result, and fallback behavior API do not exist. This is RED for parity plus `CudaToolkitUnsupported`, `CudssUnavailable`, `PlanIncompatible`, `DirectJacobianUnsupported`, and disabled-fallback throwing before any optimizer implementation.

- [ ] **Step 3: Define the minimal public API and profiles**

Forward-declare `CudaSparseLevenbergMarquardtOptimizer`, then define `CudaSparseLevenbergMarquardtParams` as a public subclass of `LevenbergMarquardtParams` with only:

```cpp
using OptimizerType = CudaSparseLevenbergMarquardtOptimizer;
bool fallbackOnUnsupported = true;
bool collectTiming = true;
bool collectAttemptTrace = false;
bool validateStructureEveryIteration = false;
```

Define:

```cpp
enum class CudaSparseLmBackend { Cuda, CpuFallback };

enum class CudaSparseLmTerminationReason {
  None,
  ErrorThreshold,
  Converged,
  MaxIterations,
  SmallCostChange,
  LambdaUpperBound,
};

enum class CudaSparseLmFallbackReason {
  None,
  CudaUnavailable,
  CudaToolkitUnsupported,
  CudssUnavailable,
  PlanIncompatible,
  DirectJacobianUnsupported,
};

struct CudaSparseLmStageTimings {
  double plan = 0.0;
  double hostZero = 0.0;
  double factorLinearizationAndPackingWall = 0.0;
  double factorLinearizationCpuSum = 0.0;
  double csrPackingCpuSum = 0.0;
  double upload = 0.0;
  double transposeUpdate = 0.0;
  double normalEquations = 0.0;
  double cudssAnalysis = 0.0;
  double damping = 0.0;
  double cudssFactorAndSolve = 0.0;
  double modelError = 0.0;
  double deltaDownload = 0.0;
  double retract = 0.0;
  double nonlinearTrialError = 0.0;
};

struct CudaSparseLmAttemptRecord {
  size_t acceptedIterationsBeforeAttempt = 0;
  size_t attempt = 0;
  double lambda = 0.0;
  double linearizedChange = 0.0;
  double nonlinearChange = 0.0;
  double modelFidelity = 0.0;
  bool accepted = false;
};

struct CudaSparseLevenbergMarquardtResult {
  CudaSparseLmBackend backend = CudaSparseLmBackend::Cuda;
  CudaSparseLmFallbackReason fallbackReason =
      CudaSparseLmFallbackReason::None;
  DirectJacobianStatus fallbackStatus;
  std::string fallbackDetail;
  CudaSparseLmTerminationReason termination =
      CudaSparseLmTerminationReason::None;
  size_t outerLinearizations = 0;
  size_t iterations = 0;
  size_t lambdaAttempts = 0;
  size_t acceptedSteps = 0;
  size_t cudssAnalyses = 0;
  double initialError = 0.0;
  double finalError = 0.0;
  double finalLambda = 0.0;
  CudaSparseLmStageTimings timings;
  std::vector<CudaSparseLmAttemptRecord> attemptTrace;
};
```

The optimizer is standalone rather than derived from `NonlinearOptimizer`, because the base `iterate()` contract requires returning a `GaussianFactorGraph`:

```cpp
class GTSAM_EXPORT CudaSparseLevenbergMarquardtOptimizer {
 public:
  CudaSparseLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const CudaSparseLevenbergMarquardtParams& params = {});

  const Values& optimize();
  const Values& values() const;
  double error() const;
  const CudaSparseLevenbergMarquardtParams& params() const;
  const CudaSparseLevenbergMarquardtResult& result() const;
};
```

- [ ] **Step 4: Re-run the prewritten tests after defining the API and observe RED**

Run:

```bash
cmake --build build-cuda-cudss-on --target testCudaSparseLevenbergMarquardt -j2
```

Expected: declarations now compile, but linking or execution fails because optimizer construction, lazy preflight, CPU fallback, and LM control flow are not implemented. Do not add new fallback tests after this point in Task 7.

- [ ] **Step 5: Implement lazy setup and centralized fallback from original inputs**

The optimizer owns copies of `graph`, `initialValues`, and `currentValues`. At the start of `optimize()`, evaluate the initial nonlinear error, reject a non-finite value, return immediately with `ErrorThreshold` if it is already at or below `params_.errorTol`, and return with `MaxIterations` when `maxIterations == 0`. Only then check CUDA device availability, `GTSAM_ENABLE_CUDSS`, and `DeviceSparseJacobianNormalEquations::preflightCapability()`. Map an unsupported internal device capability to `CudaToolkitUnsupported` before constructing CUDA state; keep the reuse strategy out of the public optimizer API. Then construct layout, plan, host, context, linearizer, and device state. Do not mutate `currentValues` until a trial step is accepted.

Keep setup objects in `std::unique_ptr` members so plan-construction incompatibility can select CPU fallback without leaving partially constructed CUDA members.

Implement the single path exercised by the Step 1 tests:

```cpp
const Values& runCpuFallback(CudaSparseLmFallbackReason reason,
                             const DirectJacobianStatus& status,
                             const std::string& detail) {
  if (!params_.fallbackOnUnsupported) {
    throw std::runtime_error(detail);
  }
  LevenbergMarquardtParams cpuParams = params_;
  currentValues_ =
      LevenbergMarquardtOptimizer(graph_, initialValues_, cpuParams).optimize();
  currentError_ = graph_.error(currentValues_);
  result_.backend = CudaSparseLmBackend::CpuFallback;
  result_.fallbackReason = reason;
  result_.fallbackStatus = status;
  result_.fallbackDetail = detail;
  result_.finalError = currentError_;
  return currentValues_;
}
```

Use it for unavailable CUDA, internal normal-equation capability false, cuDSS not compiled, symbolic plan incompatibility, and expected direct-Jacobian semantic status. Include factor index and dynamic factor type in semantic `detail`. Never use this path for non-finite data or a CUDA execution failure after device work starts.

- [ ] **Step 6: Port the CPU LM state machine and inner lambda loop**

Use an internal state that mirrors the fields relevant to `internal::LevenbergMarquardtState`:

```cpp
struct CudaLmState {
  double lambda = params_.lambdaInitial;
  double currentFactor = params_.lambdaFactor;
  size_t acceptedIterations = 0;
  size_t totalInnerAttempts = 0;
};
```

For each outer call equivalent to `LevenbergMarquardtOptimizer::iterate()`:

1. Save `previousError`, clear the pinned buffers exactly once, and call
   `linearizer_.linearize(graph_, currentValues_, *layout_, *plan_,
   host_.get(), nullptr, params_.validateStructureEveryIteration)`.
2. Throw immediately for `NonFiniteValues`; select complete-solve CPU fallback for the other expected compatibility failures.
3. Upload numerics, form the undamped system, prepare damping, and call the internally idempotent `analyze()`.
4. Repeatedly solve at `state.lambda`. Increment `totalInnerAttempts` once for every solve, and record the lambda before changing it.
5. If linearized change is nonnegative, retract, evaluate nonlinear error, compute cost change, and compute fidelity only when `linearizedChange > epsilon * oldLinearizedError`, exactly as `tryLambda()` does. Set `stopSearchingLambda` when `abs(costChange) < relativeErrorTol * currentError`.
6. On an accepted step, move the trial values into `currentValues_`, increment `acceptedIterations`, and apply the exact decrease rule. Fixed mode divides lambda by `currentFactor`; adaptive mode multiplies it by `max(1/3, 1 - pow(2*fidelity - 1, 3))` and doubles `currentFactor`; both clamp to `lambdaLowerBound`.
7. On a rejected nonterminal attempt, multiply lambda by `currentFactor`; adaptive mode then doubles `currentFactor`. If the new lambda reaches `lambdaUpperBound`, finish the outer call with `LambdaUpperBound`. If the small-cost condition was set, finish it with `SmallCostChange` without increasing lambda.
8. After the outer call completes, invoke `iterationHook(state.acceptedIterations, previousError, currentError)` whether a step was accepted or the inner search terminated without progress. This matches `NonlinearOptimizer::defaultOptimize()`, whose hook also runs after a terminal no-progress `iterate()`.
9. After an accepted step and hook, call the public `checkConvergence(params_, previousError, currentError)`. Otherwise preserve the more specific inner termination reason. Stop with `MaxIterations` when accepted iterations reach the configured bound.

Increment `result_.outerLinearizations` once before each host clear. Set `result_.iterations` and `acceptedSteps` from `state.acceptedIterations`, `lambdaAttempts` from `state.totalInnerAttempts`, and `finalLambda` from `state.lambda` on every exit. When `collectAttemptTrace` is true, append one `CudaSparseLmAttemptRecord` per solve after its outcome is known. cuDSS execution, factorization-status, and nonzero `CUDSS_DATA_INFO` failures continue to throw with stage context as required by this prototype; unlike CPU LM's `IndeterminantLinearSystemException` handling, they do not merely increase lambda.

Implement against the CPU-side `tryLambda()` reference harness already written in Step 1. Make its deterministic trace and termination assertions pass without changing their expected trajectories; they test control flow rather than merely final objective.

- [ ] **Step 7: Make cuDSS analysis truly one-time**

The optimizer calls `device_->analyze()` every outer iteration for simple control flow, but the device component returns immediately when already analyzed against the same persistent system pointers. Make the Step 1 multi-linearization assertion pass with result/profile analysis count exactly one.

- [ ] **Step 8: Re-run all Task 7 tests**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target testCudaSparseLevenbergMarquardt.run -j2
```

Expected: reconfiguration discovers the new optimizer source and test; every test written in Step 1 passes unchanged for the branches available in this build. CUDA and CPU LM converge to matching `Pose2` solutions for both damping modes, the LM trajectory/termination tests pass, error is finite, and cuDSS analysis occurs once. The no-cuDSS build later executes its already-written `CudssUnavailable` branch.

- [ ] **Step 9: Commit the optimizer**

```bash
git add gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h \
        gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp \
        gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp
git commit -m "feat: add generic CUDA sparse Jacobian LM"
```

---

### Task 8: Complete fallback, custom-factor, robust, and structural tests

**Files:**
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp`

- [ ] **Step 1: Expand the prewritten fallback suite with edge and fatal-input tests**

Task 7 already writes the first `PlanIncompatible`, `DirectJacobianUnsupported`, toolkit-capability, cuDSS-availability, and disabled-fallback tests before implementing those paths. Extend that existing suite before Task 8 production changes with a Hessian-only factor, a factor whose returned row count changes with `Values`, and a key covered only by a zero-row factor. Require the zero-row structural case to report `PlanIncompatible`; require returned-factor cases to retain their factor index/status. For each CPU-solvable graph, assert fallback still restarts CPU LM from the original initial values and records the exact reason:

```cpp
CHECK(optimizer.result().backend == CudaSparseLmBackend::CpuFallback);
CHECK(optimizer.result().fallbackReason ==
      CudaSparseLmFallbackReason::DirectJacobianUnsupported);
CHECK(optimizer.result().fallbackStatus.failure ==
      DirectJacobianFailure::ConstrainedFactor);
EXPECT_LONGS_EQUAL(expectedFactorIndex,
                   optimizer.result().fallbackStatus.factorIndex);
EXPECT(assert_equal(cpuFromOriginalInitial, actual, 1e-9));
```

When `fallbackOnUnsupported == false`, assert each newly added semantic condition throws `std::runtime_error` containing the factor index and dynamic factor type.

Add a separate non-finite factor test that always throws from the optimizer with stage and factor index. Do not run CPU fallback for non-finite residuals/Jacobians because the CPU optimizer would receive the same invalid input and the design requires non-finite systems to be surfaced.

- [ ] **Step 2: Extend the existing centralized path for the new edge statuses**

Keep the `runCpuFallback()` implementation introduced and tested in Task 7. Map the new `UnsupportedGaussianFactor`, `ConstrainedFactor`, and `StructuralMismatch` cases to `DirectJacobianUnsupported` with their original factor index/status; map uncovered and zero-row-only layouts to `PlanIncompatible`. Convert `DirectJacobianFailure::NonFiniteValues` into a stage-specific exception instead. Wrap CUDA API errors after device work begins with stage text and rethrow; do not silently fall back after a CUDA execution failure.

- [ ] **Step 3: Add a non-sendable `CustomFactor` optimizer test**

Use `CustomFactor` with an atomic callback counter. Assert the CUDA backend is used, the callback runs exactly once per nonlinear error or linearization request expected by the LM control flow, and its linearization invocation occurs on the caller thread.

- [ ] **Step 4: Add robust and heterogeneous graph parity tests**

Test a graph containing `Pose2`, `Point2`, unary, binary, and ternary `NoiseModelFactorN` factors with mixed tangent and residual dimensions. Repeat with Huber noise. Compare:

```text
streamed J,b       against GaussianFactorGraph reference every iteration
accepted-step count against CPU LM
final objective    within 1e-8 relative tolerance
final Values       within 1e-6 local-coordinate tolerance
```

- [ ] **Step 5: Verify cuDSS-disabled fallback**

Build and run in the existing CUDA-without-cuDSS configuration:

```bash
cmake -S . -B build-cuda-no-cudss
cmake --build build-cuda-no-cudss --target testCudaSparseLevenbergMarquardt.run -j2
```

Expected: optimizer tests that require CUDA solve are conditionally skipped; the explicit no-cuDSS test passes with reason `CudssUnavailable` and detail `cuDSS support is not compiled`.

- [ ] **Step 6: Run all focused nonlinear CUDA tests**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target \
  testCudaSparseJacobian.run \
  testCudaSparseLevenbergMarquardt.run \
  testCudaDeviceValues.run -j2
```

Expected: all focused tests pass.

- [ ] **Step 7: Commit compatibility coverage**

```bash
git add gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h \
        gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp \
        gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp
git commit -m "test: cover sparse CUDA LM compatibility fallback"
```

---

### Task 9: Add stage profiling and BAL baseline comparisons

**Files:**
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h`
- Modify: `gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp`
- Modify: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h`
- Modify: `gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h`
- Modify: `gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu`
- Modify: `timing/sfm_ba/timeCudaSFMBAL.cpp`

- [ ] **Step 1: Write a failing profile-accounting test**

Run a two-iteration graph with `collectTiming = true` and assert all counts are correct, all measured stages are finite and nonnegative, pattern/setup bytes are separated from per-iteration numeric bytes, and total H2D bytes equal:

```text
outerLinearizations * (sizeof(double) * (J.nnz + J.rows))
```

- [ ] **Step 2: Add synchronized timing only at stage boundaries**

Use `std::chrono::steady_clock` around CPU work. When timing is enabled, give the streaming linearizer one timing slot per factor; record `factor->linearize(values)` and `ScatterOneFactor()` separately in that factor's slot, then reduce the slots after both TBB and serial passes. Report their sums as aggregate worker CPU time, which may exceed wall time under TBB, and separately report wall time around the complete parallel linearize-and-pack call. Do not use contended atomic timing counters in the hot loop. When timing is disabled, avoid per-factor clock reads.

For the diagnostic path, time `graph.linearize(values)` wall time and `packGaussianFactorGraph()` wall time independently. This provides the directly comparable Gaussian-object-construction versus CSR-packing split required by the research question.

For asynchronous CUDA stages, record an event pair or synchronize once after the complete stage. Do not synchronize inside individual copy, SpMV, or SpGEMM helpers except for the explicit Task 5 exceptional-cleanup rule. Account separately for the mandatory cuDSS 0.8.0 factorization-info boundary and the final combined scalar/delta D2H boundary.

Split device timing into:

```text
one-time pattern upload
one-time transpose/H structure setup
per-iteration numeric H2D
J^T numeric update
J^T J
J^T b
first cuDSS analysis
per-attempt damping
per-attempt factorization, DATA_INFO boundary, and solve
per-attempt model error
per-attempt delta/scalar D2H
```

The CPU section of the same report must include:

```text
host zero wall time
streaming factor-linearization-plus-pack wall time
sum of per-factor linearize CPU time
sum of per-factor CSR scatter CPU time
diagnostic GaussianFactorGraph linearize wall time
diagnostic GaussianFactorGraph-to-CSR pack wall time
```

- [ ] **Step 3: Add benchmark command-line modes**

Extend `timeCudaSFMBAL` usage and parsing with:

```text
--cuda-sparse-lm
--cuda-sparse-pack-only
--gaussian-graph-pack-diagnostic
```

`--cuda-sparse-pack-only` repeatedly runs `StreamingSparseJacobianLinearizer` into a reused plan/host buffer. `--gaussian-graph-pack-diagnostic` times `graph.linearize(values)` followed by `packGaussianFactorGraph()` into the same plan/host layout. `--cuda-sparse-lm` runs the public optimizer and prints its complete stage profile.

- [ ] **Step 4: Print comparable baseline rows**

For the same BAL input and LM parameters, print:

```text
ordinary GTSAM LM total
existing CUDA SFM specialized LM total
generic sparse-Jacobian CUDA LM total
streaming temporary-factor pack only
GaussianFactorGraph construction plus identical CSR pack
```

Print factor count, residual rows, scalar columns, `J.nnz`, `H.nnz`, iteration count, accepted steps, lambda attempts, H2D/D2H bytes, and whether TBB is compiled.

- [ ] **Step 5: Build and run a smoke benchmark**

Run:

```bash
cmake --build build-cuda-cudss-on --target timeCudaSFMBAL -j2
./build-cuda-cudss-on/timing/sfm_ba/timeCudaSFMBAL \
  --cuda-sparse-pack-only --linearization-repeats 2 \
  examples/Data/dubrovnik-16-22106-pre.txt
./build-cuda-cudss-on/timing/sfm_ba/timeCudaSFMBAL \
  --cuda-sparse-lm examples/Data/dubrovnik-16-22106-pre.txt
```

Expected: both commands finish successfully, all stage times are finite, CSR dimensions are identical between streaming and diagnostic paths, and the generic optimizer reaches the same final objective tolerance as CPU LM.

- [ ] **Step 6: Commit profiling and benchmark support**

```bash
git add gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h \
        gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp \
        gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h \
        gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp \
        gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h \
        gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu \
        timing/sfm_ba/timeCudaSFMBAL.cpp
git commit -m "bench: profile generic sparse Jacobian CUDA LM"
```

---

### Task 10: Final verification and research handoff

**Files:**
- Modify: `docs/superpowers/specs/2026-07-10-direct-sparse-jacobian-gpu-prototype-design.md`
- Create: `docs/superpowers/results/2026-07-14-direct-sparse-jacobian-gpu-prototype.md`

- [ ] **Step 1: Run formatting and inspect the diff**

Run the repository's configured C++ formatter over only the newly created/modified C++ and CUDA files, then inspect:

```bash
clang-format -i \
  gtsam/base/cuda/CudaPinnedHostArray.h \
  gtsam/nonlinear/cuda/SparseJacobianPlan.h \
  gtsam/nonlinear/cuda/SparseJacobianPlan.cpp \
  gtsam/nonlinear/cuda/HostSparseJacobian.h \
  gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.h \
  gtsam/nonlinear/cuda/StreamingSparseJacobianLinearizer.cpp \
  gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h \
  gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.cu \
  gtsam/nonlinear/cuda/CudssLinearSolver.h \
  gtsam/nonlinear/cuda/CudssLinearSolver.cpp \
  gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h \
  gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.cpp \
  gtsam/nonlinear/tests/testCudaSparseJacobian.cpp \
  gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt.cpp \
  gtsam/nonlinear/tests/testCudaDeviceValues.cpp \
  timing/sfm_ba/timeCudaSFMBAL.cpp
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; `.superpowers/` remains untracked and untouched; only files named in this plan are modified.

- [ ] **Step 2: Run focused tests in the cuDSS build**

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on --target \
  testCudaSparseJacobian.run \
  testCudaSparseLevenbergMarquardt.run \
  testCudaDeviceValues.run \
  testCudaSfm.run -j2
```

Expected: all targets pass.

- [ ] **Step 3: Run the no-cuDSS and CPU regression builds**

```bash
cmake -S . -B build-cuda-no-cudss
cmake --build build-cuda-no-cudss --target \
  testCudaSparseLevenbergMarquardt.run testCudaDeviceValues.run -j2
cmake -S . -B build-ci-cpu-sanity
cmake --build build-ci-cpu-sanity --target testNonlinearFactorGraph.run -j2
```

Expected: no-cuDSS fallback tests pass; the non-CUDA GTSAM build and existing nonlinear graph tests remain unaffected.

- [ ] **Step 4: Run compute-sanitizer on the focused GPU test**

```bash
compute-sanitizer --tool memcheck \
  ./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaSparseJacobian
```

Expected: exit status zero with no invalid access, race-independent uninitialized read, or leaked CUDA allocation attributable to the new code.

- [ ] **Step 5: Run the final BAL benchmark matrix**

Run all five baselines on `dubrovnik-16-22106-pre.txt` and `dubrovnik-135-90642-pre.txt` after one untimed warm-up. Use at least ten measured repetitions for pack-only modes and report optimizer runs individually rather than hiding iteration-count variation in an average.

- [ ] **Step 6: Write the result report and update the design boundary**

The result document must include:

```text
hardware/software versions
graph and matrix dimensions
correctness tolerances
stage timing table
transfer byte table
streaming versus GaussianFactorGraph-pack comparison
generic versus specialized SFM comparison
dominant remaining bottleneck
recommendation for or against direct NoiseModelFactor emission
```

Update the design spec's follow-up section with measured evidence only; keep direct writers, compact batch support, upper-only H generation, custom GPU factors, persistent device `Values`, and iSAM2 out of the implemented milestone.

- [ ] **Step 7: Commit verification artifacts**

```bash
git add docs/superpowers/specs/2026-07-10-direct-sparse-jacobian-gpu-prototype-design.md \
        docs/superpowers/results/2026-07-14-direct-sparse-jacobian-gpu-prototype.md
git commit -m "docs: report sparse Jacobian GPU prototype results"
```
