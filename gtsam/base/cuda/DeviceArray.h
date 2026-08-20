/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    DeviceArray.h
 * @brief   Owning typed device allocation with transfer profiling
 * @author  Ruogu Li
 * @date    Jun 16, 2026
 */

#pragma once

#include <gtsam/base/cuda/Errors.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam::cuda {

/// Timing and byte count for one profiled host/device transfer.
struct DeviceTransferTiming {
  size_t bytes = 0;
  double resizeElapsed = 0.0;
  double copyElapsed = 0.0;
};

/// Accumulator for multiple profiled host/device transfers.
struct DeviceTransferSummary {
  size_t bytes = 0;
  double resizeElapsed = 0.0;
  double copyElapsed = 0.0;

  void add(const DeviceTransferTiming& timing) {
    bytes += timing.bytes;
    resizeElapsed += timing.resizeElapsed;
    copyElapsed += timing.copyElapsed;
  }
};

namespace internal {

inline double transferElapsedSince(
    std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
      .count();
}

}  // namespace internal

/**
 * Move-only RAII owner for a contiguous CUDA device allocation.
 *
 * Transfers are enqueued on the caller-provided stream. The ordinary upload
 * and download methods do not synchronize; the caller must keep host storage
 * alive until the stream reaches the copy. Profiled transfers synchronize so
 * they can return a complete wall-clock duration.
 */
template <typename T>
class DeviceArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "DeviceArray requires trivially copyable values");

 public:
  /// Construct an empty allocation.
  DeviceArray() = default;

  /// Allocate storage for `size` elements.
  explicit DeviceArray(size_t size) { resize(size); }

  DeviceArray(const DeviceArray&) = delete;
  DeviceArray& operator=(const DeviceArray&) = delete;

  DeviceArray(DeviceArray&& other) noexcept
      : data_(other.data_), size_(other.size_) {
    other.data_ = nullptr;
    other.size_ = 0;
  }

  DeviceArray& operator=(DeviceArray&& other) noexcept {
    if (this == &other) return *this;
    resetUnchecked();
    data_ = other.data_;
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  ~DeviceArray() { resetUnchecked(); }

  /// Replace the allocation with storage for `size` elements.
  void resize(size_t size) {
    if (size == size_) return;
    reset();
    if (size > 0) {
      T* newData = nullptr;
      GTSAM_CUDA_CHECK(
          cudaMalloc(reinterpret_cast<void**>(&newData), sizeof(T) * size));
      data_ = newData;
    }
    size_ = size;
  }

  /// Resize and asynchronously copy a host vector to device storage.
  void upload(const std::vector<T>& host, cudaStream_t stream = nullptr) {
    resize(host.size());
    if (host.empty()) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, host.data(), sizeof(T) * size_,
                                     cudaMemcpyHostToDevice, stream));
  }

  /// Upload and synchronize, returning allocation and copy timings.
  DeviceTransferTiming uploadProfiled(
      const std::vector<T>& host, cudaStream_t stream = nullptr) {
    DeviceTransferTiming timing;
    timing.bytes = sizeof(T) * host.size();

    const auto resizeStart = std::chrono::steady_clock::now();
    resize(host.size());
    timing.resizeElapsed = internal::transferElapsedSince(resizeStart);

    if (host.empty()) return timing;
    const auto copyStart = std::chrono::steady_clock::now();
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, host.data(), sizeof(T) * size_,
                                     cudaMemcpyHostToDevice, stream));
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
    timing.copyElapsed = internal::transferElapsedSince(copyStart);
    return timing;
  }

  /// Asynchronously fill the allocation with bitwise zero.
  void zero(cudaStream_t stream = nullptr) {
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemsetAsync(data_, 0, sizeof(T) * size_, stream));
  }

  /// Resize and asynchronously copy another device allocation.
  void copyFrom(const DeviceArray<T>& other,
                cudaStream_t stream = nullptr) {
    if (this == &other) return;
    resize(other.size());
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, other.data(), sizeof(T) * size_,
                                     cudaMemcpyDeviceToDevice, stream));
  }

  /// Resize and asynchronously copy device storage to a host vector.
  void download(std::vector<T>* host, cudaStream_t stream = nullptr) const {
    host->resize(size_);
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(host->data(), data_, sizeof(T) * size_,
                                     cudaMemcpyDeviceToHost, stream));
  }

  /// Download and synchronize, returning host resize and copy timings.
  DeviceTransferTiming downloadProfiled(
      std::vector<T>* host, cudaStream_t stream = nullptr) const {
    DeviceTransferTiming timing;
    timing.bytes = sizeof(T) * size_;

    const auto resizeStart = std::chrono::steady_clock::now();
    host->resize(size_);
    timing.resizeElapsed = internal::transferElapsedSince(resizeStart);

    if (size_ == 0) return timing;
    const auto copyStart = std::chrono::steady_clock::now();
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(host->data(), data_, sizeof(T) * size_,
                                     cudaMemcpyDeviceToHost, stream));
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
    timing.copyElapsed = internal::transferElapsedSince(copyStart);
    return timing;
  }

  /// Release the device allocation and return to the empty state.
  void reset() {
    if (data_) {
      GTSAM_CUDA_CHECK(cudaFree(data_));
    }
    data_ = nullptr;
    size_ = 0;
  }

  /// Return the mutable device pointer, or nullptr when empty.
  T* data() { return data_; }
  /// Return the device pointer, or nullptr when empty.
  const T* data() const { return data_; }
  /// Return the number of allocated elements.
  size_t size() const { return size_; }
  /// Return whether the allocation contains no elements.
  bool empty() const { return size_ == 0; }

 private:
  T* data_ = nullptr;
  size_t size_ = 0;

  void resetUnchecked() noexcept {
    if (data_) {
      cudaFree(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }
};

}  // namespace gtsam::cuda
