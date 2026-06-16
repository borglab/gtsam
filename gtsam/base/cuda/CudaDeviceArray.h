#pragma once

#include <gtsam/base/cuda/CudaErrors.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam::cuda {

template <typename T>
class CudaDeviceArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "CudaDeviceArray requires trivially copyable values");

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
    resetUnchecked();
    data_ = other.data_;
    size_ = other.size_;
    other.data_ = nullptr;
    other.size_ = 0;
    return *this;
  }

  ~CudaDeviceArray() { resetUnchecked(); }

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

  void download(std::vector<T>* host, cudaStream_t stream = nullptr) const {
    host->resize(size_);
    if (size_ == 0) return;
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(host->data(), data_, sizeof(T) * size_,
                                     cudaMemcpyDeviceToHost, stream));
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
