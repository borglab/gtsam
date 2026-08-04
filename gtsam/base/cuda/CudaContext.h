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
