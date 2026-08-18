#pragma once

#include <gtsam/base/cuda/Errors.h>

#include <cuda_runtime_api.h>

#include <chrono>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam::cuda {

struct DeviceTransferTiming {
  size_t bytes = 0;
  double resizeElapsed = 0.0;
  double copyElapsed = 0.0;
};

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

template <typename T>
class DeviceArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "DeviceArray requires trivially copyable values");

 public:
  DeviceArray() = default;

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

  void upload(const std::vector<T>& host, cudaStream_t stream = nullptr) {
    resize(host.size());
    if (host.empty()) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, host.data(), sizeof(T) * size_,
                                     cudaMemcpyHostToDevice, stream));
  }

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

  // Fills the allocation with bitwise zero.
  void zero(cudaStream_t stream = nullptr) {
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemsetAsync(data_, 0, sizeof(T) * size_, stream));
  }

  void copyFrom(const DeviceArray<T>& other,
                cudaStream_t stream = nullptr) {
    if (this == &other) return;
    resize(other.size());
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(data_, other.data(), sizeof(T) * size_,
                                     cudaMemcpyDeviceToDevice, stream));
  }

  void download(std::vector<T>* host, cudaStream_t stream = nullptr) const {
    host->resize(size_);
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(host->data(), data_, sizeof(T) * size_,
                                     cudaMemcpyDeviceToHost, stream));
  }

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

  void reset() {
    if (data_) {
      GTSAM_CUDA_CHECK(cudaFree(data_));
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

  void resetUnchecked() noexcept {
    if (data_) {
      cudaFree(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }
};

}  // namespace gtsam::cuda
