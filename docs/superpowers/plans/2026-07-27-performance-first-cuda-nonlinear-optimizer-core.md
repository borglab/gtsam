# Performance-First CUDA Nonlinear Optimizer Core Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create the independent CUDA nonlinear optimizer core, strict value/factor preflight, immutable value plan, double-buffered device value runtime, and fixed-size Rn/Pose2/Pose3 packing and retraction needed by the later factor and solver subprojects.

**Architecture:** Add a new registry/compiler/data path under `gtsam/nonlinear/cuda` without routing through `CudaSparseLevenbergMarquardtOptimizer`, `SparseJacobianPlan`, or the SFM optimizer. Host-side type erasure is allowed during graph compilation and one-launch-per-value-group dispatch, while device storage and kernels remain statically typed. The public LM class is introduced as a strict, non-solving skeleton that supports empty graphs and rejects every non-null factor until the factor ABI is added in the next subproject.

**Tech Stack:** C++17, CUDA C++, GTSAM `Values`/`Value`/`NonlinearFactorGraph`, `CudaDeviceArray`, `CudaContext`, CppUnitLite, CMake, CUDA events, compute-sanitizer.

---

## Scope and boundaries

This plan implements subproject 1 of the approved umbrella design. It does
not add factor error kernels, Jacobian kernels, noise backends, PCG/cuDSS
integration, LM lambda attempts, or the GN entry point. Those require separate
plans after this core is reviewed.

This plan deliberately does not modify or reuse the existing SFM-oriented
`DeviceValues` container. Its type-block-plus-delta ownership and single-state
layout do not provide the new optimizer's immutable group plan, global tangent
routing, or constant-time whole-state swap.

The new value-backend ABI in this subproject covers:

- runtime shape inspection;
- allocation;
- host pack and unpack;
- device retraction; and
- one launch per compatible value group.

Device `localCoordinates` is outside this first slice because it is not used
by LM/GN value updates. Lie logarithms needed by factor residuals belong to the
factor-math subproject. The ABI is not declared stable for external generator
use until the generator/plugin plan.

## File structure

Create these focused files:

- `gtsam/nonlinear/cuda/CudaValueBackend.h`
  Host-side type-erased value backend and group-storage contracts.
- `gtsam/nonlinear/cuda/CudaNonlinearRegistry.h`
- `gtsam/nonlinear/cuda/CudaNonlinearRegistry.cpp`
  Concrete host-type lookup, stable backend IDs, duplicate validation, and
  explicit built-in registration.
- `gtsam/nonlinear/cuda/CudaGraphPlan.h`
- `gtsam/nonlinear/cuda/CudaGraphPlan.cpp`
- `gtsam/nonlinear/cuda/CudaGraphCompiler.h`
- `gtsam/nonlinear/cuda/CudaGraphCompiler.cpp`
  Immutable value slots/groups, tangent routing, structural fingerprint, and
  accumulated preflight diagnostics.
- `gtsam/nonlinear/cuda/CudaGraphData.h`
- `gtsam/nonlinear/cuda/CudaGraphData.cpp`
  Current/trial value states, shared group routing, global delta allocation,
  pack/unpack coordination, retraction dispatch, and pointer-swap acceptance.
- `gtsam/nonlinear/cuda/CudaBuiltinValueTypes.h`
  Trivially-copyable device representations for Pose2 and Pose3.
- `gtsam/nonlinear/cuda/DeviceLieGroup.h`
  Device-safe SO(3)/SE(3) exponential and composition primitives extracted
  from the working SFM path.
- `gtsam/nonlinear/cuda/CudaBuiltinValueBackends.h`
- `gtsam/nonlinear/cuda/CudaBuiltinValueBackends.cu`
  Fixed-size `double`, `Point2`, `Point3`, `Pose2`, and `Pose3` backends,
  pack/unpack adapters, and retraction kernels.
- `gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.h`
- `gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.cpp`
  Ownership skeleton that compiles and instantiates the new plan/data path.
- `gtsam/nonlinear/cuda/CudaLevenbergMarquardt.h`
- `gtsam/nonlinear/cuda/CudaLevenbergMarquardt.cpp`
  Public strict LM entry point for empty-graph round-trip validation.
- `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`
  Registry, diagnostics, plan, data, packing, retraction, and public API tests.
- `timing/cuda_nonlinear/CMakeLists.txt`
- `timing/cuda_nonlinear/timeCudaNonlinearValues.cpp`
  Pack/retract/swap/unpack benchmark and self-test.
- `docs/superpowers/results/2026-07-27-cuda-nonlinear-optimizer-core.md`
  Verification commands, correctness results, and measured baseline.

Modify:

- `gtsam/nonlinear/tests/CMakeLists.txt`
  Exclude the CUDA-only core test in non-CUDA builds.
- `gtsam/nonlinear/cuda/DeviceGeometryKernels.h`
  Consume shared Lie primitives without changing projection behavior.
- `timing/CMakeLists.txt`
  Add the CUDA nonlinear timing subdirectory.

## Invariants used by every task

1. Tangent columns follow ascending GTSAM key order, matching `Values::dims()`.
2. A value group is identified by `(backendId, runtimeShapeId)`.
3. Group order is lexicographically sorted by that pair; slots within a group
   remain in ascending key order.
4. Stable backend and shape IDs, not `type_info::hash_code()`, enter the
   structural fingerprint.
5. The compiler collects every unsupported value and factor before throwing.
6. Null factor slots are accepted and preserve their original index for future
   factor planning.
7. Production retraction performs no host-device transfer.
8. `acceptTrial()` swaps two owning state handles and performs no CUDA call.
9. CPU/GPU comparisons use the GTSAM build's active manifold chart flags.
10. Existing sparse and SFM CUDA tests remain green after shared primitive
    extraction.

### Task 1: Add the value backend contract and registry

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaValueBackend.h`
- Create: `gtsam/nonlinear/cuda/CudaNonlinearRegistry.h`
- Create: `gtsam/nonlinear/cuda/CudaNonlinearRegistry.cpp`
- Create: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`
- Modify: `gtsam/nonlinear/tests/CMakeLists.txt`

- [ ] **Step 1: Add the CUDA-only test exclusion**

Append this block inside the existing `if(NOT GTSAM_ENABLE_CUDA)` section:

```cmake
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaNonlinearOptimizerCore.cpp")
  list(APPEND EXCLUDE_TESTS "testCudaNonlinearOptimizerCore.cpp")
endif()
```

- [ ] **Step 2: Write failing registry tests**

Start `testCudaNonlinearOptimizerCore.cpp` with a no-op backend used only for
host registry tests:

```cpp
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/cuda/CudaNonlinearRegistry.h>

#include <memory>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {

class EmptyStorage final : public CudaValueGroupStorage {};
class EmptyHostBuffer final : public CudaValueGroupHostBuffer {};

class FakePoint2Backend final : public CudaValueBackend {
 public:
  CudaBackendId backendId() const override { return 0xFACE0001ULL; }
  const char* backendName() const override { return "FakePoint2"; }

  CudaRuntimeShape inspect(const Value& value) const override {
    (void)value.cast<Point2>();
    return {0x20001ULL, 2, 2, "Point2"};
  }

  std::unique_ptr<CudaValueGroupStorage> allocate(
      size_t count) const override {
    (void)count;
    return std::make_unique<EmptyStorage>();
  }

  void pack(const std::vector<const Value*>& values,
            CudaValueGroupStorage* destination,
            cudaStream_t stream) const override {
    (void)values;
    (void)destination;
    (void)stream;
  }

  std::unique_ptr<CudaValueGroupHostBuffer> beginDownload(
      const CudaValueGroupStorage& source,
      cudaStream_t stream) const override {
    (void)source;
    (void)stream;
    return std::make_unique<EmptyHostBuffer>();
  }

  void insertDownloaded(
      const CudaValueGroupHostBuffer& host,
      const std::vector<Key>& keys,
      Values* destination) const override {
    (void)host;
    (void)keys;
    (void)destination;
  }

  void retract(const CudaValueGroupStorage& current,
               const int* deviceTangentOffsets, size_t count,
               const double* deviceDelta, CudaValueGroupStorage* trial,
               cudaStream_t stream) const override {
    (void)current;
    (void)deviceTangentOffsets;
    (void)count;
    (void)deviceDelta;
    (void)trial;
    (void)stream;
  }
};

}  // namespace

TEST(CudaNonlinearRegistry, ResolvesConcreteStoredType) {
  CudaNonlinearRegistry registry;
  auto backend = std::make_shared<FakePoint2Backend>();
  registry.registerValue<Point2>(backend);

  const GenericValue<Point2> value(Point2(1.0, 2.0));
  const auto found = registry.findValue(value);
  CHECK(found != nullptr);
  EXPECT_LONGS_EQUAL(backend->backendId(), found->backendId());
}

TEST(CudaNonlinearRegistry, RejectsDuplicateHostTypeAndBackendId) {
  CudaNonlinearRegistry registry;
  auto first = std::make_shared<FakePoint2Backend>();
  registry.registerValue<Point2>(first);

  CHECK_EXCEPTION(registry.registerValue<Point2>(
                      std::make_shared<FakePoint2Backend>()),
                  std::invalid_argument);
  CHECK_EXCEPTION(registry.registerValue<Point3>(
                      std::make_shared<FakePoint2Backend>()),
                  std::invalid_argument);
}

TEST(CudaNonlinearRegistry, MissingTypeReturnsNull) {
  CudaNonlinearRegistry registry;
  const GenericValue<Point2> value(Point2(1.0, 2.0));
  CHECK(registry.findValue(value) == nullptr);
}
```

- [ ] **Step 3: Reconfigure and run the test to verify it fails**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because
`gtsam/nonlinear/cuda/CudaNonlinearRegistry.h` does not exist.

- [ ] **Step 4: Implement the value backend contract**

Create `CudaValueBackend.h`:

```cpp
#pragma once

#include <gtsam/base/Value.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

using CudaBackendId = uint64_t;
using CudaRuntimeShapeId = uint64_t;

struct CudaRuntimeShape {
  CudaRuntimeShapeId id = 0;
  size_t storageScalars = 0;
  int tangentDimension = 0;
  std::string description;
};

class CudaValueGroupStorage {
 public:
  virtual ~CudaValueGroupStorage() = default;
};

class CudaValueGroupHostBuffer {
 public:
  virtual ~CudaValueGroupHostBuffer() = default;
};

class CudaValueBackend {
 public:
  virtual ~CudaValueBackend() = default;

  virtual CudaBackendId backendId() const = 0;
  virtual const char* backendName() const = 0;
  virtual CudaRuntimeShape inspect(const Value& value) const = 0;

  virtual std::unique_ptr<CudaValueGroupStorage> allocate(
      size_t count) const = 0;

  virtual void pack(const std::vector<const Value*>& values,
                    CudaValueGroupStorage* destination,
                    cudaStream_t stream) const = 0;

  virtual std::unique_ptr<CudaValueGroupHostBuffer> beginDownload(
      const CudaValueGroupStorage& source,
      cudaStream_t stream) const = 0;

  virtual void insertDownloaded(
      const CudaValueGroupHostBuffer& host,
      const std::vector<Key>& keys,
      Values* destination) const = 0;

  virtual void retract(const CudaValueGroupStorage& current,
                       const int* deviceTangentOffsets, size_t count,
                       const double* deviceDelta,
                       CudaValueGroupStorage* trial,
                       cudaStream_t stream) const = 0;
};

}  // namespace gtsam::cuda
```

The virtual dispatch occurs once per value group on the host. No device kernel
uses this virtual interface.

- [ ] **Step 5: Implement concrete host-type lookup**

Create `CudaNonlinearRegistry.h`:

```cpp
#pragma once

#include <gtsam/base/GenericValue.h>
#include <gtsam/nonlinear/cuda/CudaValueBackend.h>

#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>

namespace gtsam::cuda {

class CudaNonlinearRegistry {
 public:
  template <class T>
  void registerValue(std::shared_ptr<const CudaValueBackend> backend) {
    registerValueImpl(std::type_index(typeid(GenericValue<T>)),
                      std::move(backend));
  }

  std::shared_ptr<const CudaValueBackend> findValue(
      const Value& value) const;

 private:
  void registerValueImpl(
      std::type_index hostType,
      std::shared_ptr<const CudaValueBackend> backend);

  mutable std::mutex mutex_;
  std::unordered_map<std::type_index,
                     std::shared_ptr<const CudaValueBackend>>
      valuesByHostType_;
  std::unordered_map<CudaBackendId, std::type_index> hostTypeByBackendId_;
};

}  // namespace gtsam::cuda
```

Create `CudaNonlinearRegistry.cpp` with these exact checks:

```cpp
#include <gtsam/nonlinear/cuda/CudaNonlinearRegistry.h>

#include <stdexcept>

namespace gtsam::cuda {

void CudaNonlinearRegistry::registerValueImpl(
    std::type_index hostType,
    std::shared_ptr<const CudaValueBackend> backend) {
  if (!backend) {
    throw std::invalid_argument(
        "CudaNonlinearRegistry rejects a null value backend");
  }
  if (backend->backendId() == 0) {
    throw std::invalid_argument(
        "CudaNonlinearRegistry rejects backend id zero");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (valuesByHostType_.count(hostType) != 0) {
    throw std::invalid_argument(
        "CudaNonlinearRegistry duplicate value host type");
  }
  if (hostTypeByBackendId_.count(backend->backendId()) != 0) {
    throw std::invalid_argument(
        "CudaNonlinearRegistry duplicate value backend id");
  }
  valuesByHostType_.emplace(hostType, backend);
  hostTypeByBackendId_.emplace(backend->backendId(), hostType);
}

std::shared_ptr<const CudaValueBackend>
CudaNonlinearRegistry::findValue(const Value& value) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found =
      valuesByHostType_.find(std::type_index(typeid(value)));
  return found == valuesByHostType_.end() ? nullptr : found->second;
}

}  // namespace gtsam::cuda
```

- [ ] **Step 6: Build and run the registry tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: all three registry tests pass.

- [ ] **Step 7: Commit the registry**

```bash
git add \
  gtsam/nonlinear/cuda/CudaValueBackend.h \
  gtsam/nonlinear/cuda/CudaNonlinearRegistry.h \
  gtsam/nonlinear/cuda/CudaNonlinearRegistry.cpp \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp \
  gtsam/nonlinear/tests/CMakeLists.txt
git commit -m "feat: add CUDA nonlinear value registry"
```

### Task 2: Compile immutable value slots, groups, and strict diagnostics

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaGraphPlan.h`
- Create: `gtsam/nonlinear/cuda/CudaGraphPlan.cpp`
- Create: `gtsam/nonlinear/cuda/CudaGraphCompiler.h`
- Create: `gtsam/nonlinear/cuda/CudaGraphCompiler.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`

- [ ] **Step 1: Write failing plan and diagnostic tests**

Add a `ShapeOnlyBackend<T>` test backend that returns a supplied stable
backend/shape pair:

```cpp
template <class T>
class ShapeOnlyBackend final : public CudaValueBackend {
 public:
  ShapeOnlyBackend(CudaBackendId backendId,
                   CudaRuntimeShape shape)
      : backendId_(backendId), shape_(std::move(shape)) {}

  CudaBackendId backendId() const override { return backendId_; }
  const char* backendName() const override {
    return shape_.description.c_str();
  }

  CudaRuntimeShape inspect(const Value& value) const override {
    (void)value.cast<T>();
    return shape_;
  }

  std::unique_ptr<CudaValueGroupStorage> allocate(
      size_t count) const override {
    (void)count;
    return std::make_unique<EmptyStorage>();
  }

  void pack(const std::vector<const Value*>& values,
            CudaValueGroupStorage* destination,
            cudaStream_t stream) const override {
    (void)values;
    (void)destination;
    (void)stream;
  }

  std::unique_ptr<CudaValueGroupHostBuffer> beginDownload(
      const CudaValueGroupStorage& source,
      cudaStream_t stream) const override {
    (void)source;
    (void)stream;
    return std::make_unique<EmptyHostBuffer>();
  }

  void insertDownloaded(
      const CudaValueGroupHostBuffer& host,
      const std::vector<Key>& keys,
      Values* destination) const override {
    (void)host;
    (void)keys;
    (void)destination;
  }

  void retract(const CudaValueGroupStorage& current,
               const int* deviceTangentOffsets, size_t count,
               const double* deviceDelta,
               CudaValueGroupStorage* trial,
               cudaStream_t stream) const override {
    (void)current;
    (void)deviceTangentOffsets;
    (void)count;
    (void)deviceDelta;
    (void)trial;
    (void)stream;
  }

 private:
  CudaBackendId backendId_;
  CudaRuntimeShape shape_;
};
```

Then add:

```cpp
TEST(CudaGraphCompiler, GroupsValuesAndAssignsKeyOrderedTangents) {
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(
      std::make_shared<ShapeOnlyBackend<Point2>>(
          0x1001ULL, CudaRuntimeShape{0x2001ULL, 2, 2, "Point2"}));
  registry.registerValue<Point3>(
      std::make_shared<ShapeOnlyBackend<Point3>>(
          0x1002ULL, CudaRuntimeShape{0x2002ULL, 3, 3, "Point3"}));

  Values values;
  values.insert(Symbol('x', 2), Point2(2.0, 3.0));
  values.insert(Symbol('x', 0), Point3(0.0, 1.0, 2.0));
  values.insert(Symbol('x', 1), Point2(1.0, 2.0));

  const NonlinearFactorGraph graph;
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile(graph, values);

  EXPECT_LONGS_EQUAL(3, plan.valueSlots().size());
  EXPECT_LONGS_EQUAL(2, plan.valueGroups().size());
  EXPECT_LONGS_EQUAL(7, plan.totalTangentDimension());

  const auto& x0 = plan.valueSlot(Symbol('x', 0));
  const auto& x1 = plan.valueSlot(Symbol('x', 1));
  const auto& x2 = plan.valueSlot(Symbol('x', 2));
  EXPECT_LONGS_EQUAL(0, x0.tangentOffset);
  EXPECT_LONGS_EQUAL(3, x1.tangentOffset);
  EXPECT_LONGS_EQUAL(5, x2.tangentOffset);
  EXPECT_LONGS_EQUAL(x1.groupIndex, x2.groupIndex);
  EXPECT_LONGS_EQUAL(0, x1.indexInGroup);
  EXPECT_LONGS_EQUAL(1, x2.indexInGroup);
}

TEST(CudaGraphCompiler, StructuralIdentityIsStableAndShapeSensitive) {
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(
      std::make_shared<ShapeOnlyBackend<Point2>>(
          0x1001ULL, CudaRuntimeShape{0x2001ULL, 2, 2, "Point2"}));

  Values a;
  a.insert(Symbol('x', 0), Point2(1.0, 2.0));
  Values b;
  b.insert(Symbol('x', 0), Point2(9.0, -4.0));
  Values c;
  c.insert(Symbol('x', 1), Point2(1.0, 2.0));

  const NonlinearFactorGraph empty;
  const auto planA = CudaGraphCompiler(registry).compile(empty, a);
  const auto planB = CudaGraphCompiler(registry).compile(empty, b);
  const auto planC = CudaGraphCompiler(registry).compile(empty, c);
  NonlinearFactorGraph oneNull;
  oneNull.push_back(nullptr);
  const auto planWithNull =
      CudaGraphCompiler(registry).compile(oneNull, a);

  CHECK(planA.structuralFingerprint() ==
        planB.structuralFingerprint());
  CHECK(planA.structuralFingerprint() !=
        planC.structuralFingerprint());
  CHECK(planA.structuralFingerprint() !=
        planWithNull.structuralFingerprint());
  EXPECT_LONGS_EQUAL(1, planWithNull.factorSlotCount());
}

TEST(CudaGraphCompiler, ReportsEveryUnsupportedEntryBeforeThrowing) {
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(
      std::make_shared<ShapeOnlyBackend<Point2>>(
          0x1001ULL, CudaRuntimeShape{0x2001ULL, 2, 2, "Point2"}));

  Values values;
  values.insert(Symbol('x', 0), Point2(1.0, 2.0));
  values.insert(Symbol('l', 0), Point3(1.0, 2.0, 3.0));

  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Point2>>(
      Symbol('x', 0), Point2(0.0, 0.0),
      noiseModel::Unit::Create(2));
  graph.push_back(nullptr);
  graph.emplace_shared<PriorFactor<Point3>>(
      Symbol('l', 0), Point3(0.0, 0.0, 0.0),
      noiseModel::Unit::Create(3));

  try {
    (void)CudaGraphCompiler(registry).compile(graph, values);
    FAIL("compile should reject unsupported graph content");
  } catch (const CudaGraphPreflightError& error) {
    EXPECT_LONGS_EQUAL(3, error.issues().size());
    CHECK(error.issues()[0].category ==
          CudaPreflightCategory::UnsupportedValue);
    CHECK(error.issues()[1].category ==
          CudaPreflightCategory::UnsupportedFactor);
    CHECK(error.issues()[2].category ==
          CudaPreflightCategory::UnsupportedFactor);
    EXPECT_LONGS_EQUAL(0, error.issues()[1].factorIndex);
    EXPECT_LONGS_EQUAL(2, error.issues()[2].factorIndex);
    CHECK(std::string(error.what()).find("3 unsupported") !=
          std::string::npos);
  }
}

TEST(CudaGraphCompiler, RejectsInvalidRuntimeShape) {
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(
      std::make_shared<ShapeOnlyBackend<Point2>>(
          0x1001ULL, CudaRuntimeShape{0, 2, 2, "invalid"}));
  Values values;
  values.insert(Symbol('x', 0), Point2(1.0, 2.0));

  try {
    (void)CudaGraphCompiler(registry).compile({}, values);
    FAIL("compile should reject shape id zero");
  } catch (const CudaGraphPreflightError& error) {
    EXPECT_LONGS_EQUAL(1, error.issues().size());
    CHECK(error.issues()[0].category ==
          CudaPreflightCategory::InvalidValueShape);
    EXPECT_LONGS_EQUAL(Symbol('x', 0), error.issues()[0].key);
  }
}
```

Include `PriorFactor`, `Symbol`, and `NoiseModel` headers required by these
tests.

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because `CudaGraphPlan.h` and
`CudaGraphCompiler.h` do not exist.

- [ ] **Step 3: Define the immutable plan and diagnostic types**

Create `CudaGraphPlan.h`:

```cpp
#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/cuda/CudaValueBackend.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gtsam::cuda {

struct CudaValueSlotPlan {
  Key key = 0;
  CudaBackendId backendId = 0;
  CudaRuntimeShapeId shapeId = 0;
  size_t groupIndex = 0;
  size_t indexInGroup = 0;
  int tangentOffset = 0;
  int tangentDimension = 0;
};

struct CudaValueGroupPlan {
  std::shared_ptr<const CudaValueBackend> backend;
  CudaRuntimeShape shape;
  std::vector<size_t> slotIndices;
  std::vector<Key> keys;
  std::vector<int> tangentOffsets;
};

class CudaGraphPlan {
 public:
  CudaGraphPlan(std::vector<CudaValueSlotPlan> slots,
                std::vector<CudaValueGroupPlan> groups,
                int totalTangentDimension,
                size_t factorSlotCount,
                uint64_t structuralFingerprint);

  const std::vector<CudaValueSlotPlan>& valueSlots() const;
  const std::vector<CudaValueGroupPlan>& valueGroups() const;
  const CudaValueSlotPlan& valueSlot(Key key) const;
  int totalTangentDimension() const;
  size_t factorSlotCount() const;
  uint64_t structuralFingerprint() const;

 private:
  std::vector<CudaValueSlotPlan> valueSlots_;
  std::vector<CudaValueGroupPlan> valueGroups_;
  FastMap<Key, size_t> slotByKey_;
  int totalTangentDimension_ = 0;
  size_t factorSlotCount_ = 0;
  uint64_t structuralFingerprint_ = 0;
};

enum class CudaPreflightCategory {
  UnsupportedValue,
  InvalidValueShape,
  UnsupportedFactor,
};

struct CudaPreflightIssue {
  CudaPreflightCategory category;
  Key key = 0;
  size_t factorIndex = static_cast<size_t>(-1);
  std::string hostType;
  std::vector<Key> factorKeys;
  std::string detail;
};

class CudaGraphPreflightError : public std::runtime_error {
 public:
  explicit CudaGraphPreflightError(
      std::vector<CudaPreflightIssue> issues);

  const std::vector<CudaPreflightIssue>& issues() const;

 private:
  std::vector<CudaPreflightIssue> issues_;
};

}  // namespace gtsam::cuda
```

Implement the constructors, key lookup, accessors, duplicate-key checks, and
preflight summary string in `CudaGraphPlan.cpp`. Keep compilation and
fingerprinting in `CudaGraphCompiler.cpp`.

- [ ] **Step 4: Implement deterministic compilation**

Create `CudaGraphCompiler.h`:

```cpp
#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/CudaGraphPlan.h>
#include <gtsam/nonlinear/cuda/CudaNonlinearRegistry.h>

namespace gtsam::cuda {

class CudaGraphCompiler {
 public:
  explicit CudaGraphCompiler(const CudaNonlinearRegistry& registry)
      : registry_(registry) {}

  CudaGraphPlan compile(const NonlinearFactorGraph& graph,
                        const Values& values) const;

 private:
  const CudaNonlinearRegistry& registry_;
};

}  // namespace gtsam::cuda
```

Implement `compile()` with this exact sequence:

```cpp
std::vector<CudaPreflightIssue> issues;
std::vector<PendingSlot> pending;

for (const auto& [key, value] : values) {
  auto backend = registry_.findValue(value);
  if (!backend) {
    issues.push_back({CudaPreflightCategory::UnsupportedValue,
                      key,
                      static_cast<size_t>(-1),
                      demangle(typeid(value).name()),
                      {},
                      "no CUDA value backend is registered"});
    continue;
  }

  try {
    const CudaRuntimeShape shape = backend->inspect(value);
    if (shape.id == 0 || shape.tangentDimension <= 0 ||
        shape.storageScalars == 0) {
      throw std::invalid_argument("backend returned an invalid shape");
    }
    pending.push_back({key, &value, std::move(backend), shape});
  } catch (const std::exception& exception) {
    issues.push_back({CudaPreflightCategory::InvalidValueShape,
                      key,
                      static_cast<size_t>(-1),
                      demangle(typeid(value).name()),
                      {},
                      exception.what()});
  }
}

for (size_t factorIndex = 0; factorIndex < graph.size(); ++factorIndex) {
  const auto& factor = graph[factorIndex];
  if (!factor) continue;
  issues.push_back({CudaPreflightCategory::UnsupportedFactor,
                    0,
                    factorIndex,
                    demangle(typeid(*factor).name()),
                    factor->keys(),
                    "no CUDA factor backend is registered"});
}

if (!issues.empty()) {
  throw CudaGraphPreflightError(std::move(issues));
}
```

After validation:

1. Keep `pending` in ascending key order from `Values`.
2. Assign tangent offsets with checked `int` addition.
3. Build a sorted `std::map<std::pair<CudaBackendId,
   CudaRuntimeShapeId>, size_t>` to obtain deterministic group order.
4. Fill group keys, slot indices, and tangent offsets in key order.
5. Set each slot's `groupIndex` and `indexInGroup`.
6. Compute 64-bit FNV-1a over the factor-slot count and null markers, then
   each value key, backend ID, shape ID, tangent dimension, group index, and
   index within group.

Use explicit integer-byte appends rather than hashing object padding:

```cpp
class Fnv1a64 {
 public:
  void append(uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
      hash_ ^= static_cast<uint8_t>(value >> (8 * byte));
      hash_ *= 1099511628211ULL;
    }
  }
  uint64_t value() const { return hash_; }

 private:
  uint64_t hash_ = 14695981039346656037ULL;
};
```

Build `CudaGraphPreflightError::what()` as:

```text
CUDA graph preflight failed: N unsupported or invalid entries
```

Keep full typed detail in `issues()`, not only in the summary string.

- [ ] **Step 5: Build and run all core tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: registry, grouping, tangent-offset, fingerprint, and accumulated
diagnostic tests pass.

- [ ] **Step 6: Commit the graph compiler**

```bash
git add \
  gtsam/nonlinear/cuda/CudaGraphPlan.h \
  gtsam/nonlinear/cuda/CudaGraphPlan.cpp \
  gtsam/nonlinear/cuda/CudaGraphCompiler.h \
  gtsam/nonlinear/cuda/CudaGraphCompiler.cpp \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp
git commit -m "feat: compile CUDA nonlinear value plans"
```

### Task 3: Add double-buffered graph data and group-level retraction dispatch

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaGraphData.h`
- Create: `gtsam/nonlinear/cuda/CudaGraphData.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`

- [ ] **Step 1: Add a recording backend and failing ownership tests**

Add a `RecordingStorage` containing an integer token and a
`RecordingBackend` that records allocation, pack, retract, download, and
host-insertion calls:

```cpp
class RecordingStorage final : public CudaValueGroupStorage {};
class RecordingHostBuffer final : public CudaValueGroupHostBuffer {};

template <class T, int Dimension, CudaBackendId Id>
class RecordingBackend final : public CudaValueBackend {
 public:
  CudaBackendId backendId() const override { return Id; }
  const char* backendName() const override { return "Recording"; }

  CudaRuntimeShape inspect(const Value& value) const override {
    (void)value.cast<T>();
    return {Id + 1, static_cast<size_t>(Dimension), Dimension,
            "Recording"};
  }

  std::unique_ptr<CudaValueGroupStorage> allocate(
      size_t count) const override {
    (void)count;
    ++allocationCount_;
    return std::make_unique<RecordingStorage>();
  }

  void pack(const std::vector<const Value*>& values,
            CudaValueGroupStorage* destination,
            cudaStream_t stream) const override {
    (void)values;
    (void)destination;
    (void)stream;
    ++packCount_;
  }

  std::unique_ptr<CudaValueGroupHostBuffer> beginDownload(
      const CudaValueGroupStorage& source,
      cudaStream_t stream) const override {
    (void)source;
    (void)stream;
    ++downloadCount_;
    return std::make_unique<RecordingHostBuffer>();
  }

  void insertDownloaded(
      const CudaValueGroupHostBuffer& host,
      const std::vector<Key>& keys,
      Values* destination) const override {
    (void)host;
    (void)keys;
    (void)destination;
    ++insertionCount_;
  }

  void retract(const CudaValueGroupStorage& current,
               const int* deviceTangentOffsets, size_t count,
               const double* deviceDelta,
               CudaValueGroupStorage* trial,
               cudaStream_t stream) const override {
    (void)current;
    (void)deviceTangentOffsets;
    (void)count;
    (void)deviceDelta;
    (void)trial;
    (void)stream;
    ++retractCount_;
  }

  int allocationCount() const { return allocationCount_; }
  int packCount() const { return packCount_; }
  int retractCount() const { return retractCount_; }

 private:
  mutable int allocationCount_ = 0;
  mutable int packCount_ = 0;
  mutable int retractCount_ = 0;
  mutable int downloadCount_ = 0;
  mutable int insertionCount_ = 0;
};

using RecordingPoint2Backend =
    RecordingBackend<Point2, 2, 0x3101ULL>;
using RecordingPoint3Backend =
    RecordingBackend<Point3, 3, 0x3102ULL>;
```

Use it in these tests:

```cpp
TEST(CudaGraphData, AllocatesTwoStatesAndOneGlobalDelta) {
  CudaContext context;
  auto backend = std::make_shared<RecordingPoint2Backend>();
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(backend);

  Values values;
  values.insert(Symbol('x', 0), Point2(1.0, 2.0));
  values.insert(Symbol('x', 1), Point2(3.0, 4.0));
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, values);

  CudaGraphData data(plan, values, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, backend->allocationCount());
  EXPECT_LONGS_EQUAL(1, backend->packCount());
  EXPECT_LONGS_EQUAL(4, data.deltaSize());
  EXPECT_LONGS_EQUAL(1, data.groupCount());
}

TEST(CudaGraphData, AcceptSwapsWholeStateHandlesWithoutCudaWork) {
  CudaContext context;
  auto backend = std::make_shared<RecordingPoint2Backend>();
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(backend);
  Values values;
  values.insert(Symbol('x', 0), Point2(1.0, 2.0));
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, values);
  CudaGraphData data(plan, values, context.stream());

  const void* currentBefore = data.currentStateIdentity();
  const void* trialBefore = data.trialStateIdentity();
  data.acceptTrial();

  CHECK(data.currentStateIdentity() == trialBefore);
  CHECK(data.trialStateIdentity() == currentBefore);
  EXPECT_LONGS_EQUAL(0, backend->retractCount());
}

TEST(CudaGraphData, RetractDispatchesOncePerGroup) {
  CudaContext context;
  auto point2 = std::make_shared<RecordingPoint2Backend>();
  auto point3 = std::make_shared<RecordingPoint3Backend>();
  CudaNonlinearRegistry registry;
  registry.registerValue<Point2>(point2);
  registry.registerValue<Point3>(point3);

  Values values;
  values.insert(Symbol('x', 0), Point2(1.0, 2.0));
  values.insert(Symbol('l', 0), Point3(3.0, 4.0, 5.0));
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, values);
  CudaGraphData data(plan, values, context.stream());

  data.retract(context.stream());
  EXPECT_LONGS_EQUAL(1, point2->retractCount());
  EXPECT_LONGS_EQUAL(1, point3->retractCount());
}
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because `CudaGraphData` is undefined.

- [ ] **Step 3: Define graph-data ownership**

Create `CudaGraphData.h`:

```cpp
#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/CudaGraphPlan.h>

#include <memory>
#include <vector>

namespace gtsam::cuda {

struct CudaValueGroupRuntime {
  CudaDeviceArray<int> tangentOffsets;
};

class CudaValueState {
 public:
  size_t groupCount() const { return groups_.size(); }

 private:
  friend class CudaGraphData;
  std::vector<std::unique_ptr<CudaValueGroupStorage>> groups_;
};

class CudaGraphData {
 public:
  CudaGraphData(const CudaGraphPlan& plan, const Values& initialValues,
                cudaStream_t stream);

  size_t groupCount() const;
  size_t deltaSize() const;
  double* deltaData();
  const double* deltaData() const;

  void uploadDelta(const Vector& delta, cudaStream_t stream);
  void retract(cudaStream_t stream);
  void acceptTrial() noexcept;
  Values downloadCurrent(cudaStream_t stream);

  const void* currentStateIdentity() const { return current_.get(); }
  const void* trialStateIdentity() const { return trial_.get(); }

 private:
  const CudaGraphPlan* plan_;
  std::vector<CudaValueGroupRuntime> groupRuntime_;
  std::unique_ptr<CudaValueState> current_;
  std::unique_ptr<CudaValueState> trial_;
  CudaDeviceArray<double> delta_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Implement allocation, routing, and pointer swap**

In `CudaGraphData.cpp`, construct both states with:

```cpp
current_ = std::make_unique<CudaValueState>();
trial_ = std::make_unique<CudaValueState>();
current_->groups_.reserve(plan.valueGroups().size());
trial_->groups_.reserve(plan.valueGroups().size());
groupRuntime_.resize(plan.valueGroups().size());

for (size_t groupIndex = 0;
     groupIndex < plan.valueGroups().size(); ++groupIndex) {
  const auto& group = plan.valueGroups()[groupIndex];
  current_->groups_.push_back(
      group.backend->allocate(group.keys.size()));
  trial_->groups_.push_back(
      group.backend->allocate(group.keys.size()));
  groupRuntime_[groupIndex].tangentOffsets.upload(
      group.tangentOffsets, stream);

  std::vector<const Value*> hostValues;
  hostValues.reserve(group.keys.size());
  for (Key key : group.keys) {
    hostValues.push_back(&initialValues.at(key));
  }
  group.backend->pack(hostValues,
                      current_->groups_[groupIndex].get(), stream);
}

delta_.resize(static_cast<size_t>(plan.totalTangentDimension()));
```

Implement retraction:

```cpp
for (size_t groupIndex = 0;
     groupIndex < plan_->valueGroups().size(); ++groupIndex) {
  const auto& group = plan_->valueGroups()[groupIndex];
  group.backend->retract(
      *current_->groups_[groupIndex],
      groupRuntime_[groupIndex].tangentOffsets.data(),
      group.keys.size(), delta_.data(),
      trial_->groups_[groupIndex].get(), stream);
}
```

Implement acceptance exactly as:

```cpp
void CudaGraphData::acceptTrial() noexcept {
  current_.swap(trial_);
}
```

Implement the two-phase download as:

```cpp
Values CudaGraphData::downloadCurrent(cudaStream_t stream) {
  std::vector<std::unique_ptr<CudaValueGroupHostBuffer>> host;
  host.reserve(plan_->valueGroups().size());
  for (size_t groupIndex = 0;
       groupIndex < plan_->valueGroups().size(); ++groupIndex) {
    const auto& group = plan_->valueGroups()[groupIndex];
    host.push_back(group.backend->beginDownload(
        *current_->groups_[groupIndex], stream));
  }
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));

  Values values;
  for (size_t groupIndex = 0;
       groupIndex < plan_->valueGroups().size(); ++groupIndex) {
    const auto& group = plan_->valueGroups()[groupIndex];
    group.backend->insertDownloaded(
        *host[groupIndex], group.keys, &values);
  }
  return values;
}
```

`uploadDelta()` must reject a dimension mismatch before uploading.
`downloadCurrent()` first calls each backend's `beginDownload()`, retaining
all returned host buffers. It then synchronizes the supplied stream once and
calls `insertDownloaded()` for every group to construct one `Values`.
Built-in backends added in Tasks 4 and 5 must honor this two-phase contract.

- [ ] **Step 5: Run the focused tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: allocation count, delta size, group dispatch count, and state-handle
swap tests pass.

- [ ] **Step 6: Commit graph data**

```bash
git add \
  gtsam/nonlinear/cuda/CudaGraphData.h \
  gtsam/nonlinear/cuda/CudaGraphData.cpp \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp
git commit -m "feat: add double-buffered CUDA graph data"
```

### Task 4: Add fixed-size Euclidean value backends

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaBuiltinValueBackends.h`
- Create: `gtsam/nonlinear/cuda/CudaBuiltinValueBackends.cu`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`

- [ ] **Step 1: Write failing Euclidean pack/retract tests**

Add:

```cpp
TEST(CudaBuiltinValues, EuclideanPackRetractAndUnpackMatchCpu) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);

  Values initial;
  initial.insert(Symbol('d', 0), 2.0);
  initial.insert(Symbol('p', 0), Point2(1.0, -2.0));
  initial.insert(Symbol('l', 0), Point3(3.0, 4.0, 5.0));
  initial.insert(Symbol('p', 1), Point2(-6.0, 7.0));

  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, initial);
  CudaGraphData data(plan, initial, context.stream());

  Vector delta = Vector::Zero(plan.totalTangentDimension());
  delta.segment<1>(
      plan.valueSlot(Symbol('d', 0)).tangentOffset) << 0.25;
  delta.segment<3>(
      plan.valueSlot(Symbol('l', 0)).tangentOffset) <<
      1.0, 1.5, 2.0;
  delta.segment<2>(
      plan.valueSlot(Symbol('p', 0)).tangentOffset) <<
      0.5, -0.75;
  delta.segment<2>(
      plan.valueSlot(Symbol('p', 1)).tangentOffset) <<
      -0.5, 0.125;
  data.uploadDelta(delta, context.stream());
  data.retract(context.stream());
  data.acceptTrial();
  const Values actual = data.downloadCurrent(context.stream());

  const VectorValues cpuDelta =
      FlatDeltaToVectorValues(plan, delta);
  const Values expected = initial.retract(cpuDelta);
  CHECK(assert_equal(expected, actual, 1e-12));
}

TEST(CudaBuiltinValues, EuclideanRejectsWrongStoredTypeAndDeltaSize) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);
  Values initial;
  initial.insert(Symbol('p', 0), Point2(1.0, 2.0));
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, initial);
  CudaGraphData data(plan, initial, context.stream());

  CHECK_EXCEPTION(data.uploadDelta(Vector::Zero(3), context.stream()),
                  std::invalid_argument);
}
```

Implement `FlatDeltaToVectorValues` in the test by iterating
`plan.valueSlots()` and inserting each flat segment under the slot key. Keep
all delta construction keyed through
`plan.valueSlot(key).tangentOffset` as shown above.

```cpp
VectorValues FlatDeltaToVectorValues(
    const CudaGraphPlan& plan, const Vector& delta) {
  VectorValues result;
  for (const auto& slot : plan.valueSlots()) {
    result.insert(
        slot.key,
        Vector(delta.segment(slot.tangentOffset,
                             slot.tangentDimension)));
  }
  return result;
}
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because
`RegisterBuiltinCudaValueBackends` is undefined.

- [ ] **Step 3: Declare stable built-in IDs and registration**

Create `CudaBuiltinValueBackends.h`:

```cpp
#pragma once

#include <gtsam/nonlinear/cuda/CudaNonlinearRegistry.h>

namespace gtsam::cuda {

inline constexpr CudaBackendId kCudaDoubleValueBackendId =
    0x475453414d440001ULL;
inline constexpr CudaBackendId kCudaPoint2ValueBackendId =
    0x475453414d500002ULL;
inline constexpr CudaBackendId kCudaPoint3ValueBackendId =
    0x475453414d500003ULL;
inline constexpr CudaBackendId kCudaPose2ValueBackendId =
    0x475453414d530002ULL;
inline constexpr CudaBackendId kCudaPose3ValueBackendId =
    0x475453414d530003ULL;

void RegisterBuiltinCudaValueBackends(
    CudaNonlinearRegistry& registry);

}  // namespace gtsam::cuda
```

Use distinct stable shape IDs for scalar, Point2, Point3, Pose2, and Pose3.

- [ ] **Step 4: Implement coordinate-major Euclidean storage**

In `CudaBuiltinValueBackends.cu`, define:

```cpp
class CudaRnStorage final : public CudaValueGroupStorage {
 public:
  CudaDeviceArray<double> coordinates;
  size_t count = 0;
  int dimension = 0;
};

class CudaRnHostBuffer final : public CudaValueGroupHostBuffer {
 public:
  std::vector<double> coordinates;
  size_t count = 0;
  int dimension = 0;
};

__global__ void RetractRnKernel(
    const double* current, const int* tangentOffsets,
    size_t count, int dimension, const double* delta,
    double* trial) {
  const size_t index =
      static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) return;
  for (int coordinate = 0; coordinate < dimension; ++coordinate) {
    const size_t location =
        static_cast<size_t>(coordinate) * count + index;
    trial[location] =
        current[location] +
        delta[tangentOffsets[index] + coordinate];
  }
}
```

The templated host adapter for `double`, `Point2`, and `Point3` must:

1. Validate every `Value` with `value.cast<T>()`.
2. Pack coordinate-major host vectors.
3. Enqueue one H2D copy for the group.
4. Allocate trial storage without initializing it.
5. Launch `RetractRnKernel` with 256 threads and
   `(count + 255) / 256` blocks.
6. Allocate a `CudaRnHostBuffer` and enqueue its D2H copy in
   `beginDownload()`.
7. Reconstruct and insert the downloaded `T` objects in
   `insertDownloaded()` after `CudaGraphData` performs the single stream
   synchronization.

Use explicit conversion helpers:

```cpp
template <class T>
double Coordinate(const T& value, int coordinate);

template <>
double Coordinate<double>(const double& value, int coordinate) {
  if (coordinate != 0) {
    throw std::out_of_range("double coordinate");
  }
  return value;
}

template <class T>
T MakeValue(const double* coordinates);
```

Provide concrete specializations for `double`, `Point2`, and `Point3`.
Do not reinterpret Eigen object memory.

In this task, `RegisterBuiltinCudaValueBackends` registers the three Euclidean
backends. Task 5 extends the same function with Pose2 and Pose3 after their
implementations exist.

- [ ] **Step 5: Run Euclidean correctness tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: Euclidean mixed-group GPU retraction equals
`Values::retract()` within `1e-12`.

- [ ] **Step 6: Run compute-sanitizer on the focused test**

Run:

```bash
compute-sanitizer --tool memcheck --error-exitcode=1 \
  ./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: exit 0 and zero memory errors.

- [ ] **Step 7: Commit Euclidean backends**

```bash
git add \
  gtsam/nonlinear/cuda/CudaBuiltinValueBackends.h \
  gtsam/nonlinear/cuda/CudaBuiltinValueBackends.cu \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp
git commit -m "feat: add CUDA Euclidean value backends"
```

### Task 5: Extract Lie primitives and add Pose2/Pose3 backends

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaBuiltinValueTypes.h`
- Create: `gtsam/nonlinear/cuda/DeviceLieGroup.h`
- Modify: `gtsam/nonlinear/cuda/DeviceGeometryKernels.h`
- Modify: `gtsam/nonlinear/cuda/CudaBuiltinValueBackends.cu`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`
- Test: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Write failing Pose2/Pose3 round-trip and retraction tests**

Add deterministic test cases:

```cpp
TEST(CudaBuiltinValues, PosePackRoundTripMatchesCpu) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);

  Values initial;
  initial.insert(Symbol('a', 0),
                 Pose2(1.25, -3.5, 2.8));
  initial.insert(
      Symbol('b', 0),
      Pose3(Rot3::RzRyRx(0.6, -0.4, 2.7),
            Point3(4.0, -5.0, 6.0)));

  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, initial);
  CudaGraphData data(plan, initial, context.stream());
  const Values actual = data.downloadCurrent(context.stream());
  CHECK(assert_equal(initial, actual, 1e-12));
}

TEST(CudaBuiltinValues, PoseRetractMatchesActiveGtsamCharts) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);

  Values initial;
  initial.insert(Symbol('a', 0),
                 Pose2(1.0, 2.0, 2.9));
  initial.insert(
      Symbol('b', 0),
      Pose3(Rot3::RzRyRx(0.4, -0.3, 2.8),
            Point3(3.0, -2.0, 5.0)));

  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, initial);
  Vector delta = Vector::Zero(plan.totalTangentDimension());
  delta.segment<3>(
      plan.valueSlot(Symbol('a', 0)).tangentOffset) <<
      0.3, -0.2, 0.15;
  delta.segment<6>(
      plan.valueSlot(Symbol('b', 0)).tangentOffset) <<
      0.12, -0.08, 0.17, 0.4, -0.3, 0.2;

  CudaGraphData data(plan, initial, context.stream());
  data.uploadDelta(delta, context.stream());
  data.retract(context.stream());
  data.acceptTrial();
  const Values actual = data.downloadCurrent(context.stream());
  const Values expected =
      initial.retract(FlatDeltaToVectorValues(plan, delta));
  CHECK(assert_equal(expected, actual, 2e-11));
}
```

Add this deterministic tail and magnitude test:

```cpp
TEST(CudaBuiltinValues, PoseRetractCoversTailAndMagnitudes) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);
  const std::array<double, 5> magnitudes{
      0.0, 1e-12, 1e-7, 0.1, 2.8};

  Values initial;
  for (size_t i = 0; i < 257; ++i) {
    const double value = static_cast<double>(i);
    initial.insert(
        Symbol('a', i),
        Pose2(0.01 * value, -0.02 * value,
              0.003 * value));
  }
  for (size_t i = 0; i < 259; ++i) {
    const double value = static_cast<double>(i);
    initial.insert(
        Symbol('b', i),
        Pose3(Rot3::RzRyRx(0.001 * value,
                           -0.002 * value,
                           0.003 * value),
              Point3(0.01 * value, -0.02 * value,
                     0.03 * value)));
  }

  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, initial);
  Vector delta = Vector::Zero(plan.totalTangentDimension());
  for (size_t i = 0; i < 257; ++i) {
    const double magnitude = magnitudes[i % magnitudes.size()];
    delta.segment<3>(
        plan.valueSlot(Symbol('a', i)).tangentOffset) <<
        0.25 * magnitude, -0.5 * magnitude, magnitude;
  }
  for (size_t i = 0; i < 259; ++i) {
    const double magnitude = magnitudes[i % magnitudes.size()];
    delta.segment<6>(
        plan.valueSlot(Symbol('b', i)).tangentOffset) <<
        magnitude, -0.5 * magnitude, 0.25 * magnitude,
        0.2 * magnitude, -0.3 * magnitude, 0.4 * magnitude;
  }

  CudaGraphData data(plan, initial, context.stream());
  data.uploadDelta(delta, context.stream());
  data.retract(context.stream());
  data.acceptTrial();
  const Values actual = data.downloadCurrent(context.stream());
  const Values expected =
      initial.retract(FlatDeltaToVectorValues(plan, delta));
  CHECK(assert_equal(expected, actual, 1e-10));
}
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: preflight rejects Pose2/Pose3 because their backends are not
registered.

- [ ] **Step 3: Add trivially-copyable device pose types**

Create `CudaBuiltinValueTypes.h`:

```cpp
#pragma once

namespace gtsam::cuda {

struct CudaPose2Value {
  double c;
  double s;
  double x;
  double y;
};

struct CudaPose3Value {
  double R[9];
  double t[3];
};

}  // namespace gtsam::cuda
```

Add `static_assert(std::is_trivially_copyable_v<...>)` in the backend
translation unit.

- [ ] **Step 4: Extract shared device Lie primitives without semantic changes**

Create `DeviceLieGroup.h` by moving these existing functions from
`DeviceGeometryKernels.h` into `gtsam::cuda::internal`:

```cpp
__host__ __device__ inline void setIdentity3(double*);
__host__ __device__ inline void cayleyRetractRotation(
    const double*, double*);
__host__ __device__ inline void multiply3x3(
    const double*, const double*, double*);
__host__ __device__ inline void hat3(const double*, double*);
__host__ __device__ inline void axpy3x3(
    double, const double*, double*);
__host__ __device__ inline void so3Expmap(
    const double*, double*);
__host__ __device__ inline void se3Expmap(
    const double*, double*, double*);
```

Copy their current bodies exactly before adding new behavior. Include
`DeviceLieGroup.h` from `DeviceGeometryKernels.h`, remove the old duplicate
bodies, and leave `cameraR`, `RetractCamera`, and projection code unchanged.

- [ ] **Step 5: Add device Pose2 and Pose3 retraction**

Add to `DeviceLieGroup.h`:

```cpp
__host__ __device__ inline CudaPose2Value retractPose2(
    const CudaPose2Value& pose, const double* delta) {
  double dx = delta[0];
  double dy = delta[1];
  const double angle = delta[2];

#ifdef GTSAM_SLOW_BUT_CORRECT_EXPMAP
  if (fabs(angle) >= 1e-10) {
    const double c = cos(angle);
    const double s = sin(angle);
    const double inv = 1.0 / angle;
    const double expX = (s * dx - (1.0 - c) * dy) * inv;
    const double expY = ((1.0 - c) * dx + s * dy) * inv;
    dx = expX;
    dy = expY;
  }
#endif

  const double dc = cos(angle);
  const double ds = sin(angle);
  return {
      pose.c * dc - pose.s * ds,
      pose.s * dc + pose.c * ds,
      pose.x + pose.c * dx - pose.s * dy,
      pose.y + pose.s * dx + pose.c * dy,
  };
}

__host__ __device__ inline CudaPose3Value retractPose3(
    const CudaPose3Value& pose, const double* delta) {
  double deltaR[9];
  double deltaT[3];
#ifdef GTSAM_POSE3_EXPMAP
  se3Expmap(delta, deltaR, deltaT);
#else
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  so3Expmap(delta, deltaR);
#else
  cayleyRetractRotation(delta, deltaR);
#endif
  deltaT[0] = delta[3];
  deltaT[1] = delta[4];
  deltaT[2] = delta[5];
#endif

  CudaPose3Value result{};
  multiply3x3(pose.R, deltaR, result.R);
  for (int row = 0; row < 3; ++row) {
    result.t[row] =
        pose.t[row] +
        pose.R[3 * row] * deltaT[0] +
        pose.R[3 * row + 1] * deltaT[1] +
        pose.R[3 * row + 2] * deltaT[2];
  }
  return result;
}
```

Include `CudaBuiltinValueTypes.h`, `gtsam/config.h`, CUDA runtime annotations,
and `<math.h>`.

- [ ] **Step 6: Implement pose storage, adapters, and kernels**

In `CudaBuiltinValueBackends.cu`, add typed storage:

```cpp
template <class DeviceT>
class CudaTypedValueStorage final : public CudaValueGroupStorage {
 public:
  CudaDeviceArray<DeviceT> values;
};

template <class DeviceT, int TangentDim,
          DeviceT (*Retract)(const DeviceT&, const double*)>
__global__ void RetractTypedKernel(
    const DeviceT* current, const int* tangentOffsets,
    size_t count, const double* delta, DeviceT* trial) {
  const size_t index =
      static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index >= count) return;
  trial[index] =
      Retract(current[index], delta + tangentOffsets[index]);
}
```

The Pose2 adapter packs `(cos(theta), sin(theta), x, y)`. The Pose3 adapter
packs the row-major rotation matrix and GTSAM translation. Each adapter uses a
typed `CudaValueGroupHostBuffer` whose vector remains alive from
`beginDownload()` through `insertDownloaded()`. Reconstruct values with:

```cpp
Pose2(Rot2(device.c, device.s), Point2(device.x, device.y))

Matrix3 rotation;
for (int row = 0; row < 3; ++row) {
  for (int column = 0; column < 3; ++column) {
    rotation(row, column) = device.R[3 * row + column];
  }
}
Pose3(Rot3(rotation),
      Point3(device.t[0], device.t[1], device.t[2]))
```

Do not reinterpret Eigen object memory.

Register Pose2 and Pose3 with tangent dimensions 3 and 6. Launch one typed
kernel per pose group with 256 threads and a bounds check.

- [ ] **Step 7: Run core and SFM regression tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore testCudaSfm -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
```

Expected: new pose tests pass; existing SFM tests report no failures.

- [ ] **Step 8: Run sanitizer tail coverage**

Run:

```bash
compute-sanitizer --tool memcheck --error-exitcode=1 \
  ./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: exit 0 and zero out-of-bounds or invalid-memory reports for 257/259
element groups.

- [ ] **Step 9: Commit pose backends and shared primitives**

```bash
git add \
  gtsam/nonlinear/cuda/CudaBuiltinValueTypes.h \
  gtsam/nonlinear/cuda/DeviceLieGroup.h \
  gtsam/nonlinear/cuda/DeviceGeometryKernels.h \
  gtsam/nonlinear/cuda/CudaBuiltinValueBackends.cu \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp
git commit -m "feat: add CUDA Pose2 and Pose3 value backends"
```

### Task 6: Add the independent LM entry-point skeleton

**Files:**

- Create: `gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.h`
- Create: `gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.cpp`
- Create: `gtsam/nonlinear/cuda/CudaLevenbergMarquardt.h`
- Create: `gtsam/nonlinear/cuda/CudaLevenbergMarquardt.cpp`
- Modify: `gtsam/nonlinear/cuda/CudaNonlinearRegistry.h`
- Modify: `gtsam/nonlinear/cuda/CudaNonlinearRegistry.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`

- [ ] **Step 1: Write failing public API tests**

Add:

```cpp
TEST(CudaLevenbergMarquardt, EmptyGraphRoundTripsWithoutCpuFallback) {
  Values initial;
  initial.insert(Symbol('x', 0), Pose2(1.0, -2.0, 0.3));
  initial.insert(Symbol('l', 0), Point3(3.0, 4.0, 5.0));
  NonlinearFactorGraph graph;

  CudaLevenbergMarquardtOptimizer optimizer(graph, initial);
  const Values& result = optimizer.optimize();

  CHECK(assert_equal(initial, result, 1e-12));
  CHECK(optimizer.result().termination ==
        CudaLevenbergMarquardtTermination::EmptyGraph);
  EXPECT_LONGS_EQUAL(0, optimizer.result().iterations);
  CHECK(!optimizer.result().usedSparseCudaOptimizer);
  CHECK(!optimizer.result().usedCpuFallback);
}

TEST(CudaLevenbergMarquardt, NonemptyGraphFailsStrictPreflight) {
  const Key key = Symbol('x', 0);
  Values initial;
  initial.insert(key, Pose2());
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose2>>(
      key, Pose2(), noiseModel::Unit::Create(3));

  try {
    CudaLevenbergMarquardtOptimizer optimizer(graph, initial);
    FAIL("constructor should fail strict preflight");
  } catch (const CudaGraphPreflightError& error) {
    EXPECT_LONGS_EQUAL(1, error.issues().size());
    CHECK(error.issues()[0].category ==
          CudaPreflightCategory::UnsupportedFactor);
  }
}
```

The two result booleans are temporary audit fields for this first slice. They
make accidental delegation to either existing optimizer observable. Keep them
until the first real end-to-end factor solve replaces this skeleton.

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because `CudaLevenbergMarquardt.h` is absent.

- [ ] **Step 3: Add deterministic built-in registry initialization**

Expose:

```cpp
CudaNonlinearRegistry& DefaultCudaNonlinearRegistry();
void InitializeCudaNonlinearBackends();
```

Implement with function-local static storage and `std::once_flag`:

```cpp
CudaNonlinearRegistry& DefaultCudaNonlinearRegistry() {
  static CudaNonlinearRegistry registry;
  return registry;
}

void InitializeCudaNonlinearBackends() {
  static std::once_flag once;
  std::call_once(once, [] {
    RegisterBuiltinCudaValueBackends(
        DefaultCudaNonlinearRegistry());
  });
}
```

There are no static registration constructors. A future plugin may register
with the process registry after built-ins initialize and before compiling its
graph. Existing plans retain shared ownership of the backend objects they
resolved.

- [ ] **Step 4: Implement the internal ownership skeleton**

Create `CudaNonlinearOptimizerEngine.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/CudaGraphCompiler.h>
#include <gtsam/nonlinear/cuda/CudaGraphData.h>

#include <memory>

namespace gtsam::cuda {

class CudaNonlinearOptimizerEngine {
 public:
  CudaNonlinearOptimizerEngine(
      const NonlinearFactorGraph& graph, const Values& initial,
      const CudaNonlinearRegistry& registry);

  Values downloadCurrent();
  const CudaGraphPlan& plan() const { return plan_; }

 private:
  CudaContext context_;
  CudaGraphPlan plan_;
  CudaGraphData data_;
};

}  // namespace gtsam::cuda
```

Initialize members in this order:

```cpp
CudaNonlinearOptimizerEngine::CudaNonlinearOptimizerEngine(
    const NonlinearFactorGraph& graph, const Values& initial,
    const CudaNonlinearRegistry& registry)
    : context_(),
      plan_(CudaGraphCompiler(registry).compile(graph, initial)),
      data_(plan_, initial, context_.stream()) {
  context_.synchronize();
}
```

This path does not include or construct either existing optimizer.

- [ ] **Step 5: Implement the public LM skeleton**

Create `CudaLevenbergMarquardt.h`:

```cpp
#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <cstddef>
#include <memory>

namespace gtsam::cuda {

class CudaNonlinearOptimizerEngine;

class GTSAM_EXPORT CudaLevenbergMarquardtParams
    : public LevenbergMarquardtParams {};

enum class CudaLevenbergMarquardtTermination {
  None,
  EmptyGraph,
};

struct CudaLevenbergMarquardtResult {
  CudaLevenbergMarquardtTermination termination =
      CudaLevenbergMarquardtTermination::None;
  size_t iterations = 0;
  bool usedSparseCudaOptimizer = false;
  bool usedCpuFallback = false;
};

class GTSAM_EXPORT CudaLevenbergMarquardtOptimizer {
 public:
  CudaLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph,
      const Values& initialValues,
      const CudaLevenbergMarquardtParams& params = {});
  ~CudaLevenbergMarquardtOptimizer();

  const Values& optimize();
  const Values& values() const;
  const CudaLevenbergMarquardtResult& result() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
```

In `CudaLevenbergMarquardt.cpp`, initialize built-ins, construct the independent
engine, and implement `optimize()`:

```cpp
const Values& CudaLevenbergMarquardtOptimizer::optimize() {
  if (impl_->optimized) return impl_->values;
  impl_->values = impl_->engine->downloadCurrent();
  impl_->result.termination =
      CudaLevenbergMarquardtTermination::EmptyGraph;
  impl_->optimized = true;
  return impl_->values;
}
```

The constructor must reject a non-empty graph through compiler preflight
before `optimize()` can run. It must not catch `CudaGraphPreflightError` and
must not instantiate a fallback optimizer.

- [ ] **Step 6: Build and run public API tests**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: empty graph round-trips; non-empty graph reports exactly one strict
unsupported-factor issue.

- [ ] **Step 7: Commit the new public skeleton**

```bash
git add \
  gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.h \
  gtsam/nonlinear/cuda/CudaNonlinearOptimizerEngine.cpp \
  gtsam/nonlinear/cuda/CudaLevenbergMarquardt.h \
  gtsam/nonlinear/cuda/CudaLevenbergMarquardt.cpp \
  gtsam/nonlinear/cuda/CudaNonlinearRegistry.h \
  gtsam/nonlinear/cuda/CudaNonlinearRegistry.cpp \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp
git commit -m "feat: add independent CUDA LM optimizer skeleton"
```

### Task 7: Add value-runtime performance and transfer baselines

**Files:**

- Create: `timing/cuda_nonlinear/CMakeLists.txt`
- Create: `timing/cuda_nonlinear/timeCudaNonlinearValues.cpp`
- Modify: `timing/CMakeLists.txt`
- Modify: `gtsam/nonlinear/cuda/CudaGraphData.h`
- Modify: `gtsam/nonlinear/cuda/CudaGraphData.cpp`
- Modify: `gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp`

- [ ] **Step 1: Add exact transfer counters and a failing no-transfer test**

Add:

```cpp
struct CudaGraphDataTransfers {
  size_t initialValuesH2dBytes = 0;
  size_t routingH2dBytes = 0;
  size_t deltaH2dBytes = 0;
  size_t finalValuesD2hBytes = 0;

  size_t totalH2dBytes() const {
    return initialValuesH2dBytes + routingH2dBytes +
           deltaH2dBytes;
  }
};
```

Expose `const CudaGraphDataTransfers& transfers() const`.
Update counters only at the existing upload/download call sites; do not add
synchronization.

Add:

```cpp
TEST(CudaGraphData, RetractAndAcceptPerformNoTransfers) {
  CudaContext context;
  CudaNonlinearRegistry registry;
  RegisterBuiltinCudaValueBackends(registry);
  Values values;
  for (size_t i = 0; i < 513; ++i) {
    const double value = static_cast<double>(i);
    values.insert(Symbol('x', i),
                  Pose3(Rot3(),
                        Point3(value, -value, 0.5 * value)));
  }
  const CudaGraphPlan plan =
      CudaGraphCompiler(registry).compile({}, values);
  CudaGraphData data(plan, values, context.stream());
  data.uploadDelta(
      Vector::Constant(plan.totalTangentDimension(), 1e-4),
      context.stream());
  context.synchronize();
  const auto before = data.transfers();

  for (int repeat = 0; repeat < 100; ++repeat) {
    data.retract(context.stream());
    data.acceptTrial();
  }
  context.synchronize();
  const auto after = data.transfers();

  EXPECT_LONGS_EQUAL(before.totalH2dBytes(),
                     after.totalH2dBytes());
  EXPECT_LONGS_EQUAL(before.finalValuesD2hBytes,
                     after.finalValuesD2hBytes);
}
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore -j2
```

Expected: compilation fails because transfer counters are absent.

- [ ] **Step 3: Implement counters without hot-path instrumentation**

Count bytes at:

- group routing upload: `sizeof(int) * group.tangentOffsets.size()`;
- initial pack: backend-reported `shape.storageScalars * count *
  sizeof(double)`;
- explicit `uploadDelta()`: `delta.size() * sizeof(double)`;
- final unpack: the same backend-reported storage bytes.

`retract()` and `acceptTrial()` do not touch the counters. Do not add timing
branches or atomics to kernels.

- [ ] **Step 4: Add the timing target**

Append to `timing/CMakeLists.txt`:

```cmake
add_subdirectory(cuda_nonlinear)
```

Create `timing/cuda_nonlinear/CMakeLists.txt`:

```cmake
if(GTSAM_ENABLE_CUDA)
  gtsamAddTimingGlob("*.cpp" "" "gtsam" ${GTSAM_BUILD_TIMING_ALWAYS})
  target_compile_definitions(
    timeCudaNonlinearValues
    PRIVATE GTSAM_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
endif()
```

- [ ] **Step 5: Implement the benchmark and self-test**

`timeCudaNonlinearValues.cpp` must support:

```text
--values N
--repeats N
--self-test
--json PATH
```

Build three scenarios with deterministic values and deltas:

```cpp
enum class Scenario { Point3, Pose2, Pose3 };
```

For each scenario:

1. Create `N` values.
2. Compile the plan once.
3. Time allocation/pack once with `std::chrono`.
4. Upload one delta once.
5. Record CUDA events around `repeats` calls to `retract()` and
   `acceptTrial()`.
6. Time the same number of repeated CPU `Values::retract()` operations with
   `std::chrono`.
7. Download once.
8. Compare the final GPU values with the repeated CPU result.
9. Report CPU and GPU nanoseconds per value-update and exact transfer counts.

Compute `max_local_error` by calling
`cpuResult.localCoordinates(gpuResult)` and taking the largest absolute
coefficient across all returned `VectorValues` entries. This uses the CPU
manifold only as an offline benchmark oracle.

The JSON object must contain:

```json
{
  "build_type": "Release",
  "values": 100000,
  "repeats": 100,
  "scenarios": [
    {
      "name": "pose3",
      "compile_seconds": 0.0,
      "pack_seconds": 0.0,
      "retract_accept_seconds": 0.0,
      "gpu_nanoseconds_per_value_update": 0.0,
      "cpu_retract_seconds": 0.0,
      "cpu_nanoseconds_per_value_update": 0.0,
      "unpack_seconds": 0.0,
      "h2d_bytes": 0,
      "d2h_bytes": 0,
      "max_local_error": 0.0
    }
  ]
}
```

The self-test uses 17 values and 3 repeats, checks CPU/GPU equality, verifies
that transfer counts do not change inside the repeated device loop, and exits
nonzero on failure. It does not assert a hardware-dependent speed threshold.

- [ ] **Step 6: Build and run benchmark self-test**

Run:

```bash
cmake -S . -B build-cuda-cudss-on
cmake --build build-cuda-cudss-on \
  --target testCudaNonlinearOptimizerCore timeCudaNonlinearValues -j2
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
./build-cuda-cudss-on/timing/cuda_nonlinear/timeCudaNonlinearValues \
  --self-test
```

Expected: core tests and benchmark self-test pass.

- [ ] **Step 7: Record a Release baseline**

Run:

```bash
./build-cuda-cudss-on/timing/cuda_nonlinear/timeCudaNonlinearValues \
  --values 100000 --repeats 100 \
  --json /tmp/cuda-nonlinear-values-baseline.json
```

Expected: JSON contains all three scenarios, finite nonnegative timings, exact
transfer counts, and CPU/GPU equality within the test tolerances.

- [ ] **Step 8: Commit performance instrumentation**

```bash
git add \
  gtsam/nonlinear/cuda/CudaGraphData.h \
  gtsam/nonlinear/cuda/CudaGraphData.cpp \
  gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore.cpp \
  timing/CMakeLists.txt \
  timing/cuda_nonlinear/CMakeLists.txt \
  timing/cuda_nonlinear/timeCudaNonlinearValues.cpp
git commit -m "bench: baseline CUDA nonlinear value runtime"
```

### Task 8: Verify build modes, regressions, sanitizer results, and document the slice

**Files:**

- Create: `docs/superpowers/results/2026-07-27-cuda-nonlinear-optimizer-core.md`

- [ ] **Step 1: Run the focused CUDA test suite**

Run:

```bash
cmake --build build-cuda-cudss-on \
  --target \
    testCudaNonlinearOptimizerCore \
    testCudaDeviceValues \
    testCudaSparseJacobian \
    testCudaSparseLevenbergMarquardt \
    testCudaSfm \
    timeCudaNonlinearValues \
  -j2

./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaDeviceValues
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaSparseJacobian
./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaSparseLevenbergMarquardt
./build-cuda-cudss-on/gtsam/slam/tests/testCudaSfm
./build-cuda-cudss-on/timing/cuda_nonlinear/timeCudaNonlinearValues \
  --self-test
```

Expected: every executable exits 0 with no test failures.

- [ ] **Step 2: Run CUDA memory and race checking**

Run:

```bash
compute-sanitizer --tool memcheck --error-exitcode=1 \
  ./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore

compute-sanitizer --tool racecheck --error-exitcode=1 \
  ./build-cuda-cudss-on/gtsam/nonlinear/tests/testCudaNonlinearOptimizerCore
```

Expected: both commands exit 0 with zero reported errors.

- [ ] **Step 3: Verify the CUDA-disabled build excludes the new API cleanly**

Run:

```bash
cmake -S . -B build-ci-cpu-sanity \
  -DGTSAM_ENABLE_CUDA=OFF \
  -DGTSAM_BUILD_TESTS=ON
cmake --build build-ci-cpu-sanity --target gtsam -j2
if ctest --test-dir build-ci-cpu-sanity -N |
    rg -q 'testCudaNonlinearOptimizerCore'; then
  echo "CUDA-only test leaked into the CPU build"
  exit 1
fi
```

Expected: `gtsam` builds successfully and the CUDA core test is not listed.

- [ ] **Step 4: Run the Release baseline twice and check repeatability**

Run:

```bash
./build-cuda-cudss-on/timing/cuda_nonlinear/timeCudaNonlinearValues \
  --values 100000 --repeats 100 \
  --json /tmp/cuda-nonlinear-values-a.json

./build-cuda-cudss-on/timing/cuda_nonlinear/timeCudaNonlinearValues \
  --values 100000 --repeats 100 \
  --json /tmp/cuda-nonlinear-values-b.json
```

Expected:

- all correctness errors are within declared tolerances;
- no repeated retract/accept transfer bytes are reported;
- both files contain the same scenario and value counts; and
- timing differences are reported without applying a hardware-dependent
  binary threshold.

- [ ] **Step 5: Write the result document**

Create
`docs/superpowers/results/2026-07-27-cuda-nonlinear-optimizer-core.md` with:

```markdown
# CUDA Nonlinear Optimizer Core Result

## Scope delivered

- independent registry/compiler/plan/data path
- fixed Rn, Pose2, and Pose3 device value groups
- double-buffered current/trial values
- device-only retract and constant-time accept
- strict non-null-factor rejection
- independent empty-graph LM entry point

## Correctness verification

| Command | Result |
| --- | --- |
| testCudaNonlinearOptimizerCore | PASS (exit 0, zero failures) |
| testCudaDeviceValues | PASS (exit 0, zero failures) |
| testCudaSparseJacobian | PASS (exit 0, zero failures) |
| testCudaSparseLevenbergMarquardt | PASS (exit 0, zero failures) |
| testCudaSfm | PASS (exit 0, zero failures) |
| timeCudaNonlinearValues --self-test | PASS (exit 0) |
| compute-sanitizer memcheck | PASS (zero errors) |
| compute-sanitizer racecheck | PASS (zero errors) |
| CUDA-disabled gtsam build | PASS (exit 0) |
| CUDA-only test exclusion | PASS (test absent from ctest -N) |

## Performance baseline

State the GPU model, Release build type, exact commands, and both JSON output
paths. Embed both complete JSON objects in fenced `json` blocks so the
Point3/Pose2/Pose3 compile, pack, retract/accept, unpack, transfer, and
correctness values are preserved without transcription.

## Deferred by design

- factor and noise backends
- nonlinear error evaluation
- Jacobian/Hessian representations
- PCG and cuDSS integration
- LM lambda attempts and GN entry point
- dynamic-sized values
```

Create this document only after every PASS statement is supported by the
corresponding command in Steps 1-4. If any command fails, stop at that failure
instead of recording the table. Do not claim a speedup because this slice has
no complete optimizer loop.

- [ ] **Step 6: Inspect the final diff**

Run:

```bash
git diff --check
git status --short
git diff --stat
```

Expected: no whitespace errors; only files named by this plan are modified or
untracked.

- [ ] **Step 7: Commit the verified result**

```bash
git add \
  docs/superpowers/results/2026-07-27-cuda-nonlinear-optimizer-core.md
git commit -m "docs: record CUDA nonlinear optimizer core results"
```

## Spec traceability

This plan covers the approved subproject-1 requirements as follows:

| Approved requirement | Plan tasks |
| --- | --- |
| Independent new optimizer path | Tasks 2, 3, and 6 |
| Registry and explicit built-in initialization | Tasks 1, 4, 5, and 6 |
| Strict accumulated preflight | Tasks 2 and 6 |
| Immutable plan and structural identity | Task 2 |
| Fixed-shape value grouping and tangent routing | Tasks 2 and 3 |
| Double-buffered device values | Task 3 |
| Fixed Rn/Pose pack, unpack, and retract | Tasks 4 and 5 |
| Constant-time accept and no loop transfers | Tasks 3 and 7 |
| Existing CUDA paths remain independent | Tasks 5, 6, and 8 |
| Correctness, sanitizer, CPU-build, and performance evidence | Tasks 7 and 8 |

The following umbrella-spec requirements are intentionally assigned to later
subprojects and are not partially approximated here:

| Deferred requirement | Owning subproject |
| --- | --- |
| Factor registry, error batches, activation, and noise semantics | Error batches and nonlinear loop |
| GN entry point and device nonlinear error | Error batches and nonlinear loop |
| Block Jacobian, direct Hessian, and implicit operators | Linearization and solver integration |
| PCG and cuDSS | Linearization and solver integration |
| Deterministic generator and user plugin SDK | Generator and plugin SDK |
| Runtime-sized values and runtime-arity factors | Ordinary built-in coverage |
| Smart-factor triangulation and local Schur elimination | Specialized representations |

## Completion gate

Do not begin the factor/error subproject until all of these are true:

- the new public class does not include, instantiate, or call either existing
  CUDA optimizer;
- non-null factors fail one accumulated strict-preflight pass;
- every planned fixed-size value type round-trips and retracts against CPU
  GTSAM;
- retraction handles non-multiple launch tails under compute-sanitizer;
- accepted/trial state identity swaps without CUDA work;
- repeated retraction/acceptance changes no transfer counter;
- existing sparse and SFM CUDA regression tests pass;
- the CUDA-disabled library build passes;
- the Release value-runtime baseline is recorded; and
- the result document contains actual evidence rather than projected results.
