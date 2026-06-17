# CUDA BAL v0 Skeleton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the first CUDA-facing skeleton for accelerating `timing/timeSFMBAL.cpp`: CUDA memory wrappers, packed device values, SFM observation packing, and CSR sparsity construction for cuDSS-ready normal equations.

**Architecture:** Keep normal GTSAM APIs as the user-facing layer, then build a CUDA execution layer that maps `Key -> typed slot -> contiguous device array`. Generic CUDA infrastructure lives in `gtsam/base/cuda` and `gtsam/nonlinear/cuda`; SFM/BAL-specific packing lives in `gtsam/slam/cuda`. This plan stops before projection/Jacobian kernels and cuDSS numeric solves.

**Tech Stack:** C++17, CMake, CUDA Runtime, optional cuDSS discovery, CppUnitLite, existing GTSAM `SfmData`, `Values`, and `Key` types.

---

## File Structure

- Modify `CMakeLists.txt`: include CUDA option handling.
- Create `cmake/HandleCuda.cmake`: defines `GTSAM_ENABLE_CUDA`, enables CUDA, finds CUDA Runtime, optionally finds cuDSS.
- Modify `gtsam/CMakeLists.txt`: include `.cu` sources only when CUDA is enabled and link CUDA/cuDSS libraries.
- Modify `gtsam/base/CMakeLists.txt`, `gtsam/nonlinear/CMakeLists.txt`, `gtsam/slam/CMakeLists.txt`: install `cuda/*.h` headers.
- Modify `gtsam/base/tests/CMakeLists.txt`, `gtsam/nonlinear/tests/CMakeLists.txt`, `gtsam/slam/tests/CMakeLists.txt`: exclude CUDA-only tests from normal globs when CUDA is disabled.
- Create `gtsam/base/cuda/CudaErrors.h`: CUDA error checking helper.
- Create `gtsam/base/cuda/CudaContext.h`: stream-owning context.
- Create `gtsam/base/cuda/CudaDeviceArray.h`: RAII GPU array wrapper with upload/download.
- Create `gtsam/base/tests/testCudaBase.cpp`: grouped CUDA base tests for context, errors, and device arrays.
- Create `gtsam/nonlinear/cuda/DeviceVariableIndex.h`: maps GTSAM keys to typed device slots.
- Create `gtsam/nonlinear/cuda/DeviceValues.h`: type-erased container of typed device value blocks.
- Create `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`: CSR structure and RHS container for device normal equations.
- Create `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`: grouped CUDA nonlinear tests for variable indexing, typed value blocks, and sparse normal equations.
- Create `gtsam/slam/cuda/CudaSfmTypes.h`: `CudaCamera9`, `CudaPoint3`, and `CudaSfmObservation`.
- Create `gtsam/slam/cuda/CudaSfmValues.h`: host-side SFM camera/point packing into `DeviceValues`.
- Create `gtsam/slam/cuda/CudaSfmProjectionBatch.h`: host-side packer for BAL observations.
- Create `gtsam/slam/cuda/CudaBalCsrStructure.h`: host-side CSR pattern builder for BAL normal equations.
- Create `gtsam/slam/tests/testCudaSfm.cpp`: grouped CUDA SFM tests for value packing, observation packing, and CSR structure.

---

### Task 1: Add CUDA Build Option

**Files:**
- Create: `cmake/HandleCuda.cmake`
- Modify: `CMakeLists.txt`
- Modify: `gtsam/CMakeLists.txt`
- Modify: `gtsam/base/CMakeLists.txt`
- Modify: `gtsam/nonlinear/CMakeLists.txt`
- Modify: `gtsam/slam/CMakeLists.txt`
- Modify: `gtsam/base/tests/CMakeLists.txt`
- Modify: `gtsam/nonlinear/tests/CMakeLists.txt`
- Modify: `gtsam/slam/tests/CMakeLists.txt`

- [ ] **Step 1: Write the build option module**

Create `cmake/HandleCuda.cmake`:

```cmake
option(GTSAM_ENABLE_CUDA "Enable experimental CUDA acceleration support" OFF)
option(GTSAM_ENABLE_CUDSS "Enable experimental cuDSS linear solver support" OFF)

if(GTSAM_ENABLE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_EXTENSIONS OFF)

  add_compile_definitions(GTSAM_ENABLE_CUDA=1)

  if(GTSAM_ENABLE_CUDSS)
    find_library(CUDSS_LIBRARY cudss
      HINTS
        ${CUDAToolkit_LIBRARY_DIR}
        ${CUDAToolkit_LIBRARY_ROOT}
        $ENV{CUDA_HOME}/lib64
        $ENV{CONDA_PREFIX}/lib
    )

    if(NOT CUDSS_LIBRARY)
      message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON but libcudss was not found")
    endif()

    add_library(cudss::cudss UNKNOWN IMPORTED)
    set_target_properties(cudss::cudss PROPERTIES
      IMPORTED_LOCATION "${CUDSS_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${CUDAToolkit_INCLUDE_DIRS}")
    add_compile_definitions(GTSAM_ENABLE_CUDSS=1)
  endif()
endif()
```

- [ ] **Step 2: Include the module from the top-level CMake file**

In `CMakeLists.txt`, add this after `include(cmake/HandleTBB.cmake)`:

```cmake
include(cmake/HandleCuda.cmake)             # CUDA acceleration options
```

- [ ] **Step 3: Include CUDA source files and link CUDA libraries**

In `gtsam/CMakeLists.txt`, update the source glob inside the `foreach(subdir ${gtsam_subdirs})` loop:

```cmake
file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")
if(GTSAM_ENABLE_CUDA)
  file(GLOB_RECURSE subdir_cuda_srcs "${subdir}/*.cu")
  list(APPEND subdir_srcs ${subdir_cuda_srcs})
endif()
```

Then after:

```cmake
target_link_libraries(gtsam PUBLIC ${GTSAM_ADDITIONAL_LIBRARIES})
```

add:

```cmake
if(GTSAM_ENABLE_CUDA)
  target_link_libraries(gtsam PUBLIC CUDA::cudart)
  if(GTSAM_ENABLE_CUDSS)
    target_link_libraries(gtsam PUBLIC cudss::cudss)
  endif()
endif()
```

- [ ] **Step 4: Install CUDA headers in submodule CMake files**

In `gtsam/base/CMakeLists.txt`, add:

```cmake
file(GLOB base_cuda_headers "cuda/*.h")
install(FILES ${base_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/cuda)
```

In `gtsam/nonlinear/CMakeLists.txt`, add:

```cmake
file(GLOB nonlinear_cuda_headers "cuda/*.h")
install(FILES ${nonlinear_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear/cuda)
```

In `gtsam/slam/CMakeLists.txt`, add:

```cmake
file(GLOB slam_cuda_headers "cuda/*.h")
install(FILES ${slam_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/slam/cuda)
```

- [ ] **Step 5: Exclude CUDA-only tests when CUDA is disabled**

In `gtsam/base/tests/CMakeLists.txt`, before `gtsamAddTestsGlob(...)`, add:

```cmake
if(NOT GTSAM_ENABLE_CUDA)
  list(APPEND EXCLUDE_TESTS
    "testCudaBase.cpp")
endif()
```

In `gtsam/nonlinear/tests/CMakeLists.txt`, before `gtsamAddTestsGlob(...)`, add:

```cmake
if(NOT GTSAM_ENABLE_CUDA)
  list(APPEND EXCLUDE_TESTS
    "testCudaDeviceValues.cpp")
endif()
```

In `gtsam/slam/tests/CMakeLists.txt`, before `gtsamAddTestsGlob(...)`, add:

```cmake
if(NOT GTSAM_ENABLE_CUDA)
  list(APPEND EXCLUDE_TESTS
    "testCudaSfm.cpp")
endif()
```

Even the host-only CUDA scaffolding tests stay behind `GTSAM_ENABLE_CUDA`; this keeps CPU-only GTSAM builds from surfacing experimental CUDA test targets.

- [ ] **Step 6: Configure with CUDA disabled**

Run:

```bash
cmake -S . -B build-cuda-off -DGTSAM_ENABLE_CUDA=OFF -DGTSAM_BUILD_TESTS=ON
```

Expected: configure succeeds and does not require CUDA.

- [ ] **Step 7: Configure with CUDA enabled**

Run:

```bash
cmake -S . -B build-cuda-on -DGTSAM_ENABLE_CUDA=ON -DGTSAM_ENABLE_CUDSS=OFF -DGTSAM_BUILD_TESTS=ON
```

Expected: configure succeeds on a CUDA machine and finds `CUDAToolkit`.

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt cmake/HandleCuda.cmake gtsam/CMakeLists.txt gtsam/base/CMakeLists.txt gtsam/nonlinear/CMakeLists.txt gtsam/slam/CMakeLists.txt gtsam/base/tests/CMakeLists.txt gtsam/nonlinear/tests/CMakeLists.txt gtsam/slam/tests/CMakeLists.txt
git commit -m "build: add experimental CUDA option"
```

---

### Task 2: Add Base CUDA Memory Wrappers

**Files:**
- Create: `gtsam/base/cuda/CudaErrors.h`
- Create: `gtsam/base/cuda/CudaContext.h`
- Create: `gtsam/base/cuda/CudaDeviceArray.h`
- Create: `gtsam/base/tests/testCudaBase.cpp`

- [ ] **Step 1: Add the failing test**

Create `gtsam/base/tests/testCudaBase.cpp`:

```cpp
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>

#include <CppUnitLite/TestHarness.h>

#include <vector>

using namespace gtsam::cuda;

TEST(CudaDeviceArray, UploadDownloadRoundTrip) {
  CudaContext context;
  std::vector<double> host = {1.0, 2.0, 3.5, -4.0};

  CudaDeviceArray<double> device;
  device.upload(host, context.stream());

  std::vector<double> actual;
  device.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(host.size(), actual.size());
  for (size_t i = 0; i < host.size(); ++i) {
    DOUBLES_EQUAL(host[i], actual[i], 1e-12);
  }
}

TEST(CudaDeviceArray, MoveTransfersOwnership) {
  CudaContext context;
  CudaDeviceArray<int> original(3);
  std::vector<int> host = {4, 5, 6};
  original.upload(host, context.stream());

  CudaDeviceArray<int> moved(std::move(original));
  std::vector<int> actual;
  moved.download(&actual, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(0, original.size());
  EXPECT_LONGS_EQUAL(3, moved.size());
  EXPECT_LONGS_EQUAL(4, actual[0]);
  EXPECT_LONGS_EQUAL(5, actual[1]);
  EXPECT_LONGS_EQUAL(6, actual[2]);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaBase
```

Expected: build fails because `CudaContext` and `CudaDeviceArray` do not exist.

- [ ] **Step 3: Add CUDA error checking**

Create `gtsam/base/cuda/CudaErrors.h`:

```cpp
#pragma once

#include <cuda_runtime_api.h>

#include <sstream>
#include <stdexcept>
#include <string>

namespace gtsam::cuda {

class CudaError : public std::runtime_error {
 public:
  explicit CudaError(const std::string& message) : std::runtime_error(message) {}
};

inline void checkCuda(cudaError_t status, const char* expression,
                      const char* file, int line) {
  if (status == cudaSuccess) return;
  std::ostringstream os;
  os << "CUDA call failed at " << file << ":" << line << ": " << expression
     << " returned " << cudaGetErrorString(status);
  throw CudaError(os.str());
}

}  // namespace gtsam::cuda

#define GTSAM_CUDA_CHECK(expr) \
  ::gtsam::cuda::checkCuda((expr), #expr, __FILE__, __LINE__)
```

- [ ] **Step 4: Add CUDA context**

Create `gtsam/base/cuda/CudaContext.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaErrors.h>

#include <cuda_runtime_api.h>

namespace gtsam::cuda {

class CudaContext {
 public:
  CudaContext() { GTSAM_CUDA_CHECK(cudaStreamCreate(&stream_)); }

  explicit CudaContext(cudaStream_t externalStream)
      : stream_(externalStream), ownsStream_(false) {}

  CudaContext(const CudaContext&) = delete;
  CudaContext& operator=(const CudaContext&) = delete;

  CudaContext(CudaContext&& other) noexcept
      : stream_(other.stream_), ownsStream_(other.ownsStream_) {
    other.stream_ = nullptr;
    other.ownsStream_ = false;
  }

  CudaContext& operator=(CudaContext&& other) noexcept {
    if (this == &other) return *this;
    reset();
    stream_ = other.stream_;
    ownsStream_ = other.ownsStream_;
    other.stream_ = nullptr;
    other.ownsStream_ = false;
    return *this;
  }

  ~CudaContext() { reset(); }

  cudaStream_t stream() const { return stream_; }

  void synchronize() const { GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream_)); }

 private:
  void reset() {
    if (ownsStream_ && stream_) {
      cudaStreamDestroy(stream_);
    }
    stream_ = nullptr;
    ownsStream_ = false;
  }

  cudaStream_t stream_ = nullptr;
  bool ownsStream_ = true;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 5: Add device array**

Create `gtsam/base/cuda/CudaDeviceArray.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaErrors.h>

#include <cuda_runtime_api.h>

#include <utility>
#include <vector>

namespace gtsam::cuda {

template <typename T>
class CudaDeviceArray {
 public:
  CudaDeviceArray() = default;

  explicit CudaDeviceArray(size_t size) { resize(size); }

  CudaDeviceArray(const CudaDeviceArray&) = delete;
  CudaDeviceArray& operator=(const CudaDeviceArray&) = delete;

  CudaDeviceArray(CudaDeviceArray&& other) noexcept
      : data_(other.data_), size_(other.size_) {
    other.data_ = nullptr;
    other.size_ = 0;
  }

  CudaDeviceArray& operator=(CudaDeviceArray&& other) noexcept {
    if (this == &other) return *this;
    reset();
    data_ = other.data_;
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  ~CudaDeviceArray() { reset(); }

  void resize(size_t size) {
    if (size == size_) return;
    reset();
    size_ = size;
    if (size_ > 0) {
      GTSAM_CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&data_),
                                  sizeof(T) * size_));
    }
  }

  void upload(const std::vector<T>& host, cudaStream_t stream = nullptr) {
    resize(host.size());
    if (host.empty()) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, host.data(), sizeof(T) * size_,
                                     cudaMemcpyHostToDevice, stream));
  }

  void download(std::vector<T>* host, cudaStream_t stream = nullptr) const {
    host->resize(size_);
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(host->data(), data_, sizeof(T) * size_,
                                     cudaMemcpyDeviceToHost, stream));
  }

  void reset() {
    if (data_) {
      cudaFree(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }

  T* data() { return data_; }
  const T* data() const { return data_; }
  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 6: Run the tests**

Run:

```bash
cmake --build build-cuda-on --target testCudaBase
./build-cuda-on/gtsam/base/tests/testCudaBase
```

Expected: both tests pass.

- [ ] **Step 7: Commit**

```bash
git add gtsam/base/cuda gtsam/base/tests/CMakeLists.txt gtsam/base/tests/testCudaBase.cpp
git commit -m "feat: add CUDA device array wrapper"
```

---

### Task 3: Add Device Variable Index

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceVariableIndex.h`
- Create: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Add the failing test**

Create `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`:

```cpp
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/DeviceVariableIndex.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace gtsam::cuda;

namespace {
constexpr uint32_t kCameraType = 1;
constexpr uint32_t kPointType = 2;
}

TEST(DeviceVariableIndex, AddsAndFindsSlots) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 3);
  const Key pointKey = Symbol('p', 7);

  index.add(cameraKey, kCameraType, 3, 9);
  index.add(pointKey, kPointType, 7, 3);

  const DeviceVariableSlot camera = index.at(cameraKey);
  EXPECT_LONGS_EQUAL(kCameraType, camera.typeId);
  EXPECT_LONGS_EQUAL(3, camera.slot);
  EXPECT_LONGS_EQUAL(9, camera.tangentDim);
  EXPECT_LONGS_EQUAL(3, index.slot(cameraKey, kCameraType));

  const DeviceVariableSlot point = index.at(pointKey);
  EXPECT_LONGS_EQUAL(kPointType, point.typeId);
  EXPECT_LONGS_EQUAL(7, point.slot);
  EXPECT_LONGS_EQUAL(3, point.tangentDim);
}

TEST(DeviceVariableIndex, RejectsWrongType) {
  DeviceVariableIndex index;
  const Key cameraKey = Symbol('c', 1);
  index.add(cameraKey, kCameraType, 0, 9);
  CHECK_EXCEPTION(index.slot(cameraKey, kPointType), std::invalid_argument);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
```

Expected: build fails because `DeviceVariableIndex.h` does not exist.

- [ ] **Step 3: Add the implementation**

Create `gtsam/nonlinear/cuda/DeviceVariableIndex.h`:

```cpp
#pragma once

#include <gtsam/base/types.h>

#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace gtsam::cuda {

struct DeviceVariableSlot {
  Key key = 0;
  uint32_t typeId = 0;
  int slot = -1;
  int tangentDim = 0;
};

class DeviceVariableIndex {
 public:
  void add(Key key, uint32_t typeId, int slot, int tangentDim) {
    if (entries_.count(key) != 0) {
      throw std::invalid_argument("DeviceVariableIndex duplicate key");
    }
    entries_.emplace(key, DeviceVariableSlot{key, typeId, slot, tangentDim});
  }

  const DeviceVariableSlot& at(Key key) const {
    const auto it = entries_.find(key);
    if (it == entries_.end()) {
      throw std::out_of_range("DeviceVariableIndex missing key");
    }
    return it->second;
  }

  int slot(Key key, uint32_t expectedTypeId) const {
    const DeviceVariableSlot& entry = at(key);
    if (entry.typeId != expectedTypeId) {
      throw std::invalid_argument("DeviceVariableIndex type mismatch");
    }
    return entry.slot;
  }

  size_t size() const { return entries_.size(); }
  bool empty() const { return entries_.empty(); }

 private:
  std::unordered_map<Key, DeviceVariableSlot> entries_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Run the tests**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
./build-cuda-on/gtsam/nonlinear/tests/testCudaDeviceValues
```

Expected: tests pass.

- [ ] **Step 5: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceVariableIndex.h gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: add device variable index"
```

---

### Task 4: Add Device Values Typed Blocks

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceValues.h`
- Modify: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Add the failing CUDA-enabled test to the nonlinear CUDA suite**

Add these includes near the top of `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`:

```cpp
#include <gtsam/nonlinear/cuda/DeviceValues.h>

#include <vector>
```

Append this test block:

```cpp
namespace {
constexpr uint32_t kTinyType = 77;

struct TinyValue {
  double x;
  double y;
};
}

TEST(DeviceValues, AddsAndDownloadsTypedBlock) {
  CudaContext context;
  DeviceValues values;

  std::vector<Key> keys = {Symbol('t', 0), Symbol('t', 1)};
  std::vector<TinyValue> host = {{1.0, 2.0}, {3.0, 4.0}};

  DeviceValueBlock<TinyValue>& block =
      values.addBlock<TinyValue>(kTinyType, 2, keys, host, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, values.index().size());
  EXPECT_LONGS_EQUAL(2, block.values.size());
  EXPECT_LONGS_EQUAL(2, block.delta.size());
  EXPECT_LONGS_EQUAL(2, block.tangentDim);
  EXPECT_LONGS_EQUAL(1, values.index().slot(Symbol('t', 1), kTinyType));

  std::vector<TinyValue> actual;
  values.block<TinyValue>(kTinyType).values.download(&actual, context.stream());
  context.synchronize();

  DOUBLES_EQUAL(1.0, actual[0].x, 1e-12);
  DOUBLES_EQUAL(4.0, actual[1].y, 1e-12);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
```

Expected: build fails because `DeviceValues.h` does not exist.

- [ ] **Step 3: Add the implementation**

Create `gtsam/nonlinear/cuda/DeviceValues.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/nonlinear/cuda/DeviceVariableIndex.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace gtsam::cuda {

template <typename T>
struct DeviceValueBlock {
  CudaDeviceArray<T> values;
  CudaDeviceArray<double> delta;
  int tangentDim = 0;
};

class DeviceValues {
 public:
  const DeviceVariableIndex& index() const { return index_; }
  DeviceVariableIndex& index() { return index_; }

  template <typename T>
  DeviceValueBlock<T>& addBlock(uint32_t typeId, int tangentDim,
                                const std::vector<Key>& keys,
                                const std::vector<T>& hostValues,
                                cudaStream_t stream = nullptr) {
    if (keys.size() != hostValues.size()) {
      throw std::invalid_argument("DeviceValues keys and values size mismatch");
    }
    if (blocks_.count(typeId) != 0) {
      throw std::invalid_argument("DeviceValues duplicate type block");
    }

    auto storage = std::make_unique<TypedBlock<T>>();
    storage->block.tangentDim = tangentDim;
    storage->block.values.upload(hostValues, stream);
    storage->block.delta.resize(hostValues.size() * tangentDim);

    for (size_t i = 0; i < keys.size(); ++i) {
      index_.add(keys[i], typeId, static_cast<int>(i), tangentDim);
    }

    DeviceValueBlock<T>* result = &storage->block;
    blocks_.emplace(typeId, std::move(storage));
    return *result;
  }

  template <typename T>
  DeviceValueBlock<T>& block(uint32_t typeId) {
    auto* typed = dynamic_cast<TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

  template <typename T>
  const DeviceValueBlock<T>& block(uint32_t typeId) const {
    const auto* typed = dynamic_cast<const TypedBlock<T>*>(blockStorage(typeId));
    if (!typed) {
      throw std::invalid_argument("DeviceValues type mismatch");
    }
    return typed->block;
  }

 private:
  struct BlockStorage {
    virtual ~BlockStorage() = default;
  };

  template <typename T>
  struct TypedBlock final : BlockStorage {
    DeviceValueBlock<T> block;
  };

  BlockStorage* blockStorage(uint32_t typeId) {
    const auto it = blocks_.find(typeId);
    if (it == blocks_.end()) {
      throw std::out_of_range("DeviceValues missing type block");
    }
    return it->second.get();
  }

  const BlockStorage* blockStorage(uint32_t typeId) const {
    const auto it = blocks_.find(typeId);
    if (it == blocks_.end()) {
      throw std::out_of_range("DeviceValues missing type block");
    }
    return it->second.get();
  }

  DeviceVariableIndex index_;
  std::unordered_map<uint32_t, std::unique_ptr<BlockStorage>> blocks_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Run the CUDA-enabled tests**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
./build-cuda-on/gtsam/nonlinear/tests/testCudaDeviceValues
```

Expected: test passes.

- [ ] **Step 5: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceValues.h gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: add device values typed blocks"
```

---

### Task 5: Add SFM CUDA Types, Value Packing, and Projection Batch Packer

**Files:**
- Create: `gtsam/slam/cuda/CudaSfmTypes.h`
- Create: `gtsam/slam/cuda/CudaSfmValues.h`
- Create: `gtsam/slam/cuda/CudaSfmProjectionBatch.h`
- Create: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add the failing test**

Create `gtsam/slam/tests/testCudaSfm.cpp`:

```cpp
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace gtsam::cuda;
using gtsam::symbol_shorthand::P;

namespace {
SfmData makeTinySfmData() {
  SfmData data;
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.01, 0.001));
  data.cameras.emplace_back(Pose3(Rot3::RzRyRx(0.01, 0.02, 0.03),
                                  Point3(1.0, 0.0, 0.0)),
                            Cal3Bundler(120.0, 0.02, 0.002));

  SfmTrack track0(Point3(0.0, 0.0, 5.0));
  track0.measurements.emplace_back(0, Point2(10.0, 20.0));
  track0.measurements.emplace_back(1, Point2(11.0, 21.0));
  data.tracks.push_back(track0);

  SfmTrack track1(Point3(1.0, 0.0, 6.0));
  track1.measurements.emplace_back(0, Point2(30.0, 40.0));
  data.tracks.push_back(track1);

  return data;
}
}

TEST(CudaSfmProjectionBatch, PacksOnlyTracksWithAtLeastTwoMeasurements) {
  const SfmData data = makeTinySfmData();
  CudaContext context;

  CudaSfmProjectionBatch batch =
      CudaSfmProjectionBatch::FromSfmData(data, context.stream());
  DeviceValues values = PackSfmValues(data, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, batch.numCameras());
  EXPECT_LONGS_EQUAL(2, batch.numPoints());
  EXPECT_LONGS_EQUAL(2, batch.numObservations());
  EXPECT_LONGS_EQUAL(4, values.index().size());
  EXPECT_LONGS_EQUAL(1, values.index().slot(P(1), kCudaSfmPoint3Type));

  std::vector<CudaSfmObservation> observations;
  batch.observations().download(&observations, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(0, observations[0].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[0].pointSlot);
  DOUBLES_EQUAL(10.0, observations[0].measuredU, 1e-12);

  EXPECT_LONGS_EQUAL(1, observations[1].cameraSlot);
  EXPECT_LONGS_EQUAL(0, observations[1].pointSlot);
  DOUBLES_EQUAL(21.0, observations[1].measuredV, 1e-12);

  std::vector<CudaPoint3> points;
  values.block<CudaPoint3>(kCudaSfmPoint3Type)
      .values.download(&points, context.stream());
  context.synchronize();
  DOUBLES_EQUAL(1.0, points[1].x, 1e-12);
  DOUBLES_EQUAL(6.0, points[1].z, 1e-12);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaSfm
```

Expected: build fails because `CudaSfmProjectionBatch.h` does not exist.

- [ ] **Step 3: Add SFM CUDA POD types**

Create `gtsam/slam/cuda/CudaSfmTypes.h`:

```cpp
#pragma once

namespace gtsam::cuda {

inline constexpr unsigned int kCudaSfmCamera9Type = 0x53464d43u;
inline constexpr unsigned int kCudaSfmPoint3Type = 0x53464d50u;

struct CudaCamera9 {
  double r[3];
  double t[3];
  double f;
  double k1;
  double k2;
};

struct CudaPoint3 {
  double x;
  double y;
  double z;
};

struct CudaSfmObservation {
  int cameraSlot;
  int pointSlot;
  double measuredU;
  double measuredV;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Add the SFM value packer**

Create `gtsam/slam/cuda/CudaSfmValues.h`:

```cpp
#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/cuda/DeviceValues.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>
#include <gtsam/sfm/SfmData.h>

#include <vector>

namespace gtsam::cuda {

inline CudaCamera9 PackCamera9(const SfmCamera& camera) {
  const Pose3 openGlPose = gtsam2openGL(camera.pose());
  const Vector3 r = Rot3::Logmap(openGlPose.rotation());
  const Point3 t = openGlPose.translation();
  const Cal3Bundler& calibration = camera.calibration();

  return CudaCamera9{{r.x(), r.y(), r.z()},
                     {t.x(), t.y(), t.z()},
                     calibration.f(),
                     calibration.k1(),
                     calibration.k2()};
}

inline CudaPoint3 PackPoint3(const Point3& point) {
  return CudaPoint3{point.x(), point.y(), point.z()};
}

inline DeviceValues PackSfmValues(const SfmData& data,
                                  cudaStream_t stream = nullptr) {
  DeviceValues values;

  std::vector<Key> cameraKeys;
  std::vector<CudaCamera9> cameras;
  cameraKeys.reserve(data.numberCameras());
  cameras.reserve(data.numberCameras());
  for (size_t i = 0; i < data.numberCameras(); ++i) {
    cameraKeys.push_back(symbol_shorthand::C(i));
    cameras.push_back(PackCamera9(data.camera(i)));
  }

  std::vector<Key> pointKeys;
  std::vector<CudaPoint3> points;
  pointKeys.reserve(data.numberTracks());
  points.reserve(data.numberTracks());
  for (size_t j = 0; j < data.numberTracks(); ++j) {
    pointKeys.push_back(symbol_shorthand::P(j));
    points.push_back(PackPoint3(data.track(j).p));
  }

  values.addBlock<CudaCamera9>(kCudaSfmCamera9Type, 9, cameraKeys, cameras,
                               stream);
  values.addBlock<CudaPoint3>(kCudaSfmPoint3Type, 3, pointKeys, points, stream);
  return values;
}

}  // namespace gtsam::cuda
```

- [ ] **Step 5: Add the projection batch packer**

Create `gtsam/slam/cuda/CudaSfmProjectionBatch.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/cuda/CudaSfmTypes.h>
#include <gtsam/sfm/SfmData.h>

#include <vector>

namespace gtsam::cuda {

class CudaSfmProjectionBatch {
 public:
  static CudaSfmProjectionBatch FromSfmData(const SfmData& data,
                                            cudaStream_t stream = nullptr) {
    CudaSfmProjectionBatch batch;
    batch.numCameras_ = data.numberCameras();
    batch.numPoints_ = data.numberTracks();

    std::vector<CudaSfmObservation> hostObservations;
    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      const SfmTrack& track = data.track(pointSlot);
      if (track.measurements.size() < 2) continue;
      for (const SfmMeasurement& measurement : track.measurements) {
        hostObservations.push_back(CudaSfmObservation{
            static_cast<int>(measurement.first),
            static_cast<int>(pointSlot),
            measurement.second.x(),
            measurement.second.y()});
      }
    }

    batch.observations_.upload(hostObservations, stream);
    return batch;
  }

  const CudaDeviceArray<CudaSfmObservation>& observations() const {
    return observations_;
  }

  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  size_t numObservations() const { return observations_.size(); }

 private:
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  CudaDeviceArray<CudaSfmObservation> observations_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 6: Run the test**

Run:

```bash
cmake --build build-cuda-on --target testCudaSfm
./build-cuda-on/gtsam/slam/tests/testCudaSfm
```

Expected: test passes.

- [ ] **Step 7: Commit**

```bash
git add gtsam/slam/cuda/CudaSfmTypes.h gtsam/slam/cuda/CudaSfmValues.h gtsam/slam/cuda/CudaSfmProjectionBatch.h gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: pack SFM values and projection batches for CUDA"
```

---

### Task 6: Add Device Sparse Normal Equations Container

**Files:**
- Create: `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`
- Modify: `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`

- [ ] **Step 1: Add the failing test to the nonlinear CUDA suite**

Add this include near the top of `gtsam/nonlinear/tests/testCudaDeviceValues.cpp`:

```cpp
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
```

Append this test block:

```cpp
TEST(DeviceSparseNormalEquations, UploadsCsrPatternAndRhs) {
  CudaContext context;
  DeviceSparseNormalEquations system;

  std::vector<int> rowPointers = {0, 2, 3};
  std::vector<int> colIndices = {0, 1, 1};
  system.uploadPattern(2, rowPointers, colIndices, context.stream());
  context.synchronize();

  EXPECT_LONGS_EQUAL(2, system.rows());
  EXPECT_LONGS_EQUAL(3, system.nonzeros());
  EXPECT_LONGS_EQUAL(3, system.values().size());
  EXPECT_LONGS_EQUAL(2, system.rhs().size());
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
```

Expected: build fails because `DeviceSparseNormalEquations.h` does not exist.

- [ ] **Step 3: Add the implementation**

Create `gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h`:

```cpp
#pragma once

#include <gtsam/base/cuda/CudaDeviceArray.h>

#include <stdexcept>
#include <vector>

namespace gtsam::cuda {

class DeviceSparseNormalEquations {
 public:
  void uploadPattern(int rows, const std::vector<int>& rowPointers,
                     const std::vector<int>& colIndices,
                     cudaStream_t stream = nullptr) {
    if (rows < 0) {
      throw std::invalid_argument("DeviceSparseNormalEquations rows < 0");
    }
    if (rowPointers.size() != static_cast<size_t>(rows + 1)) {
      throw std::invalid_argument("DeviceSparseNormalEquations bad rowPointers");
    }
    if (!rowPointers.empty() &&
        rowPointers.back() != static_cast<int>(colIndices.size())) {
      throw std::invalid_argument("DeviceSparseNormalEquations bad nnz");
    }

    rows_ = rows;
    rowPointers_.upload(rowPointers, stream);
    colIndices_.upload(colIndices, stream);
    values_.resize(colIndices.size());
    rhs_.resize(rows);
  }

  int rows() const { return rows_; }
  int nonzeros() const { return static_cast<int>(values_.size()); }

  const CudaDeviceArray<int>& rowPointers() const { return rowPointers_; }
  const CudaDeviceArray<int>& colIndices() const { return colIndices_; }
  CudaDeviceArray<double>& values() { return values_; }
  const CudaDeviceArray<double>& values() const { return values_; }
  CudaDeviceArray<double>& rhs() { return rhs_; }
  const CudaDeviceArray<double>& rhs() const { return rhs_; }

 private:
  int rows_ = 0;
  CudaDeviceArray<int> rowPointers_;
  CudaDeviceArray<int> colIndices_;
  CudaDeviceArray<double> values_;
  CudaDeviceArray<double> rhs_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Run the test**

Run:

```bash
cmake --build build-cuda-on --target testCudaDeviceValues
./build-cuda-on/gtsam/nonlinear/tests/testCudaDeviceValues
```

Expected: test passes.

- [ ] **Step 5: Commit**

```bash
git add gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h gtsam/nonlinear/tests/testCudaDeviceValues.cpp
git commit -m "feat: add device sparse normal equations container"
```

---

### Task 7: Add BAL CSR Pattern Builder

**Files:**
- Create: `gtsam/slam/cuda/CudaBalCsrStructure.h`
- Modify: `gtsam/slam/tests/testCudaSfm.cpp`

- [ ] **Step 1: Add the failing test to the SFM CUDA suite**

Add this include near the top of `gtsam/slam/tests/testCudaSfm.cpp`:

```cpp
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
```

Append this test block:

```cpp
namespace {
SfmData makeTinyBalData() {
  SfmData data;
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.0, 0.0));
  data.cameras.emplace_back(Pose3(), Cal3Bundler(100.0, 0.0, 0.0));

  SfmTrack track0(Point3(0.0, 0.0, 5.0));
  track0.measurements.emplace_back(0, Point2(1.0, 2.0));
  track0.measurements.emplace_back(1, Point2(3.0, 4.0));
  data.tracks.push_back(track0);

  SfmTrack track1(Point3(1.0, 0.0, 5.0));
  track1.measurements.emplace_back(1, Point2(5.0, 6.0));
  data.tracks.push_back(track1);

  return data;
}
}

TEST(CudaBalCsrStructure, BuildsUpperTrianglePatternForMeasuredTrack) {
  const SfmData data = makeTinyBalData();
  const CudaBalCsrStructure structure = CudaBalCsrStructure::FromSfmData(data);

  EXPECT_LONGS_EQUAL(24, structure.dimension());
  EXPECT_LONGS_EQUAL(2, structure.numCameras());
  EXPECT_LONGS_EQUAL(2, structure.numPoints());

  CHECK(structure.hasEntry(0, 0));
  CHECK(structure.hasEntry(8, 8));
  CHECK(structure.hasEntry(9, 9));
  CHECK(structure.hasEntry(17, 17));
  CHECK(structure.hasEntry(18, 18));
  CHECK(structure.hasEntry(20, 20));

  CHECK(structure.hasEntry(0, 18));
  CHECK(structure.hasEntry(9, 18));
  CHECK(!structure.hasEntry(0, 21));
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:

```bash
cmake --build build-cuda-on --target testCudaSfm
```

Expected: build fails because `CudaBalCsrStructure.h` does not exist.

- [ ] **Step 3: Add the implementation**

Create `gtsam/slam/cuda/CudaBalCsrStructure.h`:

```cpp
#pragma once

#include <gtsam/sfm/SfmData.h>

#include <set>
#include <utility>
#include <vector>

namespace gtsam::cuda {

class CudaBalCsrStructure {
 public:
  static CudaBalCsrStructure FromSfmData(const SfmData& data) {
    CudaBalCsrStructure structure;
    structure.numCameras_ = data.numberCameras();
    structure.numPoints_ = data.numberTracks();
    structure.dimension_ =
        static_cast<int>(9 * structure.numCameras_ + 3 * structure.numPoints_);

    std::vector<std::set<int>> rows(structure.dimension_);
    auto addBlock = [&](int rowStart, int rowDim, int colStart, int colDim) {
      for (int r = 0; r < rowDim; ++r) {
        for (int c = 0; c < colDim; ++c) {
          const int row = rowStart + r;
          const int col = colStart + c;
          if (row <= col) {
            rows[row].insert(col);
          } else {
            rows[col].insert(row);
          }
        }
      }
    };

    for (size_t cameraSlot = 0; cameraSlot < data.numberCameras(); ++cameraSlot) {
      addBlock(static_cast<int>(9 * cameraSlot), 9,
               static_cast<int>(9 * cameraSlot), 9);
    }

    for (size_t pointSlot = 0; pointSlot < data.numberTracks(); ++pointSlot) {
      const int pointOffset =
          static_cast<int>(9 * data.numberCameras() + 3 * pointSlot);
      addBlock(pointOffset, 3, pointOffset, 3);

      const SfmTrack& track = data.track(pointSlot);
      if (track.measurements.size() < 2) continue;
      for (const SfmMeasurement& measurement : track.measurements) {
        const int cameraOffset = static_cast<int>(9 * measurement.first);
        addBlock(cameraOffset, 9, pointOffset, 3);
      }
    }

    structure.rowPointers_.push_back(0);
    for (const std::set<int>& row : rows) {
      for (int col : row) {
        structure.colIndices_.push_back(col);
      }
      structure.rowPointers_.push_back(
          static_cast<int>(structure.colIndices_.size()));
    }
    return structure;
  }

  int dimension() const { return dimension_; }
  size_t numCameras() const { return numCameras_; }
  size_t numPoints() const { return numPoints_; }
  const std::vector<int>& rowPointers() const { return rowPointers_; }
  const std::vector<int>& colIndices() const { return colIndices_; }

  bool hasEntry(int row, int col) const {
    if (row > col) std::swap(row, col);
    if (row < 0 || row >= dimension_) return false;
    for (int k = rowPointers_[row]; k < rowPointers_[row + 1]; ++k) {
      if (colIndices_[k] == col) return true;
    }
    return false;
  }

 private:
  size_t numCameras_ = 0;
  size_t numPoints_ = 0;
  int dimension_ = 0;
  std::vector<int> rowPointers_;
  std::vector<int> colIndices_;
};

}  // namespace gtsam::cuda
```

- [ ] **Step 4: Run the test**

Run:

```bash
cmake --build build-cuda-on --target testCudaSfm
./build-cuda-on/gtsam/slam/tests/testCudaSfm
```

Expected: test passes.

- [ ] **Step 5: Commit**

```bash
git add gtsam/slam/cuda/CudaBalCsrStructure.h gtsam/slam/tests/testCudaSfm.cpp
git commit -m "feat: add BAL CSR sparsity builder"
```

---

### Task 8: Wire Pack/CSR Verification Into `timeSFMBAL`

**Files:**
- Modify: `timing/timeSFMBAL.cpp`

- [ ] **Step 1: Add a pack-only CLI path**

In `timing/timeSFMBAL.cpp`, update `usage()`:

```cpp
return "Usage: timeSFMBAL [--colamd] [--profile] [--cuda-structure-only] "
       "[--benchmark-action-json FILE] [BALfile]";
```

Add a field to `RunOptions`:

```cpp
bool cudaStructureOnly = false;
```

In `parseBalFiles`, add a local variable:

```cpp
bool cudaStructureOnly = false;
```

Then add this case inside the argument loop:

```cpp
if (strcmp(argv[i], "--cuda-structure-only") == 0) {
  cudaStructureOnly = true;
  continue;
}
```

Make each `RunOptions` return include the new field:

```cpp
return {profile, benchmarkActionJson, benchmarkActionJsonPath,
        cudaStructureOnly, {filename}};
```

Add this after loading `SfmData db` and before building the CPU graph:

```cpp
#if GTSAM_ENABLE_CUDA
    if (options.cudaStructureOnly) {
      gtsam::cuda::CudaContext context;
      const auto values = gtsam::cuda::PackSfmValues(db, context.stream());
      const auto batch =
          gtsam::cuda::CudaSfmProjectionBatch::FromSfmData(db, context.stream());
      const auto csr = gtsam::cuda::CudaBalCsrStructure::FromSfmData(db);
      context.synchronize();

      std::cout << "CUDA BAL structure: cameras=" << batch.numCameras()
                << " points=" << batch.numPoints()
                << " observations=" << batch.numObservations()
                << " packed_values=" << values.index().size()
                << " dimension=" << csr.dimension()
                << " csr_nnz=" << csr.colIndices().size() << std::endl;
      continue;
    }
#else
    if (options.cudaStructureOnly) {
      throw std::runtime_error(
          "--cuda-structure-only requires configuring with GTSAM_ENABLE_CUDA=ON");
    }
#endif
```

Also add includes guarded by `GTSAM_ENABLE_CUDA`:

```cpp
#if GTSAM_ENABLE_CUDA
#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/slam/cuda/CudaBalCsrStructure.h>
#include <gtsam/slam/cuda/CudaSfmProjectionBatch.h>
#include <gtsam/slam/cuda/CudaSfmValues.h>
#endif
```

- [ ] **Step 2: Build the timing target**

Run:

```bash
cmake --build build-cuda-on --target timeSFMBAL
```

Expected: target builds.

- [ ] **Step 3: Run structure-only verification on the tiny BAL example**

Run:

```bash
./build-cuda-on/timing/timeSFMBAL --cuda-structure-only "$(pwd)/examples/Data/dubrovnik-3-7-pre.txt"
```

Expected output contains:

```text
CUDA BAL structure: cameras=3 points=7
```

- [ ] **Step 4: Commit**

```bash
git add timing/timeSFMBAL.cpp
git commit -m "feat: add CUDA BAL structure timing path"
```

---

### Task 9: Self-Review and Baseline Verification

**Files:**
- Review: all files touched above.

- [ ] **Step 1: Verify CPU-only build still works**

Run:

```bash
cmake --build build-cuda-off --target gtsam
```

Expected: `gtsam` builds without CUDA headers, CUDA toolkit, or CUDA test targets.

- [ ] **Step 2: Verify CUDA build works**

Run:

```bash
cmake --build build-cuda-on --target testCudaBase testCudaDeviceValues testCudaSfm timeSFMBAL
```

Expected: all targets build.

- [ ] **Step 3: Run new tests**

Run:

```bash
./build-cuda-on/gtsam/base/tests/testCudaBase
./build-cuda-on/gtsam/nonlinear/tests/testCudaDeviceValues
./build-cuda-on/gtsam/slam/tests/testCudaSfm
```

Expected: all tests pass.

- [ ] **Step 4: Run pack-only timing path**

Run:

```bash
./build-cuda-on/timing/timeSFMBAL --cuda-structure-only "$(pwd)/examples/Data/dubrovnik-3-7-pre.txt"
```

Expected: command prints camera, point, observation, dimension, and CSR nonzero counts without entering CPU LM optimization.

- [ ] **Step 5: Inspect public boundaries**

Confirm:

```bash
rg -n "CudaCamera9|CudaSfmObservation" gtsam/nonlinear gtsam/base
```

Expected: no matches outside generic includes or comments. SFM-specific types should stay under `gtsam/slam/cuda`.

- [ ] **Step 6: Commit verification notes if any docs changed**

If verification notes are added to the plan or a follow-up doc:

```bash
git add docs/superpowers/plans/2026-06-16-cuda-bal-v0-skeleton.md
git commit -m "docs: record CUDA BAL skeleton plan"
```

---

## Follow-Up Plan After This Milestone

After this skeleton lands, the next plan should cover:

1. GPU residual-only kernel for `CudaSfmProjectionBatch`.
2. Numeric Jacobian comparison against `GeneralSFMFactor<PinholeCamera<Cal3Bundler>, Point3>` on tiny data.
3. GPU accumulation into `DeviceSparseNormalEquations`.
4. `CudssLinearSolver` wrapper with Cholesky first, then LDLT/LDU fallback.
5. One accepted LM step from `timeSFMBAL`.
