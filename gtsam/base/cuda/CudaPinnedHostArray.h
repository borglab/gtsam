#pragma once

#include <gtsam/base/cuda/CudaErrors.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace gtsam::cuda {

template <typename T>
class CudaPinnedHostArray {
  static_assert(std::is_trivially_copyable_v<T>,
                "CudaPinnedHostArray requires trivially copyable values");

 public:
  CudaPinnedHostArray() = default;

  explicit CudaPinnedHostArray(size_t size) { resize(size); }

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

  ~CudaPinnedHostArray() { resetUnchecked(); }

  void resize(size_t size) {
    if (size == size_) return;
    if (size > std::numeric_limits<size_t>::max() / sizeof(T)) {
      throw std::overflow_error("CudaPinnedHostArray allocation size overflow");
    }

    CudaPinnedHostArray replacement;
    if (size > 0) {
      GTSAM_CUDA_CHECK(cudaMallocHost(
          reinterpret_cast<void**>(&replacement.data_), sizeof(T) * size));
      replacement.size_ = size;
    }

    if (data_) {
      GTSAM_CUDA_CHECK(cudaFreeHost(data_));
    }
    data_ = replacement.data_;
    size_ = replacement.size_;
    replacement.data_ = nullptr;
    replacement.size_ = 0;
  }

  // Fills the allocation with bitwise zero.
  void clear() {
    if (size_ == 0) return;
    std::memset(data_, 0, sizeof(T) * size_);
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
      cudaFreeHost(data_);
    }
    data_ = nullptr;
    size_ = 0;
  }
};

}  // namespace gtsam::cuda
