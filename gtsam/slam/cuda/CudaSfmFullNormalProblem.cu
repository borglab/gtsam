#include <gtsam/slam/cuda/CudaSfmFullNormalProblem.h>

#include <gtsam/base/cuda/CudaErrors.h>

#include <cmath>
#include <stdexcept>

namespace gtsam::cuda {
namespace {

constexpr int kBlockSize = 256;

int Grid(size_t count) {
  return static_cast<int>((count + kBlockSize - 1) / kBlockSize);
}

__global__ void AccumulateRhsAndBlocksKernel(
    const CudaSfmObservation* observations, size_t observationCount,
    const double* residuals, const double* cameraJacobians,
    const double* pointJacobians, int numCameras, double* rhs,
    double* cameraBlocks, double* pointBlocks) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= observationCount) return;

  const CudaSfmObservation observation = observations[i];
  const int cameraBase = 9 * observation.cameraSlot;
  const int pointBase = 9 * numCameras + 3 * observation.pointSlot;
  const double residual0 = residuals[2 * i];
  const double residual1 = residuals[2 * i + 1];
  const double* camera = cameraJacobians + 18 * i;
  const double* point = pointJacobians + 6 * i;

  for (int column = 0; column < 9; ++column) {
    const double j0 = camera[column];
    const double j1 = camera[9 + column];
    atomicAdd(&rhs[cameraBase + column],
              -j0 * residual0 - j1 * residual1);
  }
  for (int column = 0; column < 3; ++column) {
    const double j0 = point[column];
    const double j1 = point[3 + column];
    atomicAdd(&rhs[pointBase + column],
              -j0 * residual0 - j1 * residual1);
  }
  double* cameraBlock = cameraBlocks + 81 * observation.cameraSlot;
  for (int row = 0; row < 9; ++row) {
    for (int column = 0; column < 9; ++column) {
      atomicAdd(&cameraBlock[9 * row + column],
                camera[row] * camera[column] +
                    camera[9 + row] * camera[9 + column]);
    }
  }
  double* pointBlock = pointBlocks + 9 * observation.pointSlot;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      atomicAdd(&pointBlock[3 * row + column],
                point[row] * point[column] +
                    point[3 + row] * point[3 + column]);
    }
  }
}

__global__ void ApplyNormalKernel(
    const CudaSfmObservation* observations, size_t observationCount,
    const double* cameraJacobians, const double* pointJacobians,
    int numCameras, const double* input, double* output) {
  const size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i >= observationCount) return;

  const CudaSfmObservation observation = observations[i];
  const int cameraBase = 9 * observation.cameraSlot;
  const int pointBase = 9 * numCameras + 3 * observation.pointSlot;
  const double* camera = cameraJacobians + 18 * i;
  const double* point = pointJacobians + 6 * i;
  double projected0 = 0.0;
  double projected1 = 0.0;
  for (int column = 0; column < 9; ++column) {
    projected0 += camera[column] * input[cameraBase + column];
    projected1 += camera[9 + column] * input[cameraBase + column];
  }
  for (int column = 0; column < 3; ++column) {
    projected0 += point[column] * input[pointBase + column];
    projected1 += point[3 + column] * input[pointBase + column];
  }
  for (int column = 0; column < 9; ++column) {
    atomicAdd(&output[cameraBase + column],
              camera[column] * projected0 + camera[9 + column] * projected1);
  }
  for (int column = 0; column < 3; ++column) {
    atomicAdd(&output[pointBase + column],
              point[column] * projected0 + point[3 + column] * projected1);
  }
}

template <int N>
__device__ void FactorBlock(const double* source, const double* damping,
                            double lambda, double* factor) {
  for (int row = 0; row < N; ++row) {
    for (int column = 0; column < N; ++column) {
      factor[N * row + column] = source[N * row + column];
    }
    factor[N * row + row] += lambda * (damping ? damping[row] : 1.0);
  }
  for (int row = 0; row < N; ++row) {
    for (int column = 0; column <= row; ++column) {
      double value = factor[N * row + column];
      for (int inner = 0; inner < column; ++inner) {
        value -= factor[N * row + inner] * factor[N * column + inner];
      }
      if (row == column) {
        factor[N * row + column] =
            value > 1e-30 && isfinite(value) ? sqrt(value) : 1e-15;
      } else {
        factor[N * row + column] = value / factor[N * column + column];
      }
    }
    for (int column = row + 1; column < N; ++column) {
      factor[N * row + column] = 0.0;
    }
  }
}

__global__ void FactorCameraBlocksKernel(
    int numCameras, double lambda, const double* dampingDiagonal,
    const double* cameraBlocks, double* cameraFactors) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  FactorBlock<9>(cameraBlocks + 81 * camera,
                 dampingDiagonal ? dampingDiagonal + 9 * camera : nullptr,
                 lambda, cameraFactors + 81 * camera);
}

__global__ void FactorPointBlocksKernel(
    int numPoints, int pointOffset, double lambda,
    const double* dampingDiagonal, const double* pointBlocks,
    double* pointFactors) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  FactorBlock<3>(pointBlocks + 9 * point,
                 dampingDiagonal ? dampingDiagonal + pointOffset + 3 * point
                                 : nullptr,
                 lambda, pointFactors + 9 * point);
}

__global__ void AddDampingKernel(int dimension, double lambda,
                                 const double* dampingDiagonal,
                                 const double* input, double* output) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < dimension) {
    output[i] += lambda * (dampingDiagonal ? dampingDiagonal[i] : 1.0) *
                 input[i];
  }
}

template <int N>
__device__ void ApplyBlockFactor(const double* factor, const double* input,
                                 double* output) {
  double value[N];
  for (int row = 0; row < N; ++row) {
    double sum = input[row];
    for (int column = 0; column < row; ++column) {
      sum -= factor[N * row + column] * value[column];
    }
    value[row] = sum / factor[N * row + row];
  }
  for (int row = N - 1; row >= 0; --row) {
    double sum = value[row];
    for (int column = row + 1; column < N; ++column) {
      sum -= factor[N * column + row] * output[column];
    }
    output[row] = sum / factor[N * row + row];
  }
}

__global__ void ApplyCameraBlockFactorsKernel(
    int numCameras, const double* cameraFactors, const double* input,
    double* output) {
  const int camera = blockIdx.x * blockDim.x + threadIdx.x;
  if (camera >= numCameras) return;
  ApplyBlockFactor<9>(cameraFactors + 81 * camera, input + 9 * camera,
                      output + 9 * camera);
}

__global__ void ApplyPointBlockFactorsKernel(
    int numPoints, int pointOffset, const double* pointFactors,
    const double* input, double* output) {
  const int point = blockIdx.x * blockDim.x + threadIdx.x;
  if (point >= numPoints) return;
  ApplyBlockFactor<3>(pointFactors + 9 * point,
                      input + pointOffset + 3 * point,
                      output + pointOffset + 3 * point);
}

}  // namespace

struct CudaSfmFullNormalProblem::Impl {
  class Operator final : public CudaLinearOperator {
   public:
    explicit Operator(const Impl* owner) : owner_(owner) {}
    int dimension() const override { return owner_->dimension; }
    void apply(const double* input, double* output,
               cudaStream_t stream) const override {
      owner_->applyNormal(input, output, stream);
    }

   private:
    const Impl* owner_;
  };

  class Preconditioner final : public CudaPreconditioner {
   public:
    explicit Preconditioner(const Impl* owner) : owner_(owner) {}
    int dimension() const override { return owner_->dimension; }
    void apply(const double* input, double* output,
               cudaStream_t stream) const override {
      if (!owner_->prepared) {
        throw std::logic_error("SFM full-normal preconditioner is not prepared");
      }
      if (owner_->numCameras > 0) {
        ApplyCameraBlockFactorsKernel<<<Grid(owner_->numCameras), kBlockSize,
                                        0, stream>>>(
            owner_->numCameras, owner_->cameraFactors.data(), input, output);
        GTSAM_CUDA_CHECK(cudaGetLastError());
      }
      if (owner_->numPoints > 0) {
        ApplyPointBlockFactorsKernel<<<Grid(owner_->numPoints), kBlockSize, 0,
                                       stream>>>(
            owner_->numPoints, 9 * owner_->numCameras,
            owner_->pointFactors.data(), input, output);
        GTSAM_CUDA_CHECK(cudaGetLastError());
      }
    }

   private:
    const Impl* owner_;
  };

  Impl() : linearOperator(this), preconditioner(this) {}

  const CudaSfmProjectionBatch* batch = nullptr;
  const CudaSfmProjectionLinearization* linearization = nullptr;
  int numCameras = 0;
  int numPoints = 0;
  int dimension = 0;
  double lambda = 0.0;
  const double* dampingDiagonal = nullptr;
  bool linearized = false;
  bool prepared = false;
  bool explicitSystemInitialized = false;
  size_t linearizationCount = 0;
  size_t preparationCount = 0;
  CudaDeviceArray<double> rhs;
  CudaDeviceArray<double> cameraBlocks;
  CudaDeviceArray<double> pointBlocks;
  CudaDeviceArray<double> cameraFactors;
  CudaDeviceArray<double> pointFactors;
  DeviceSparseSpdSystem sparseSystem;
  Operator linearOperator;
  Preconditioner preconditioner;

  void setShape(const CudaSfmProjectionBatch& newBatch, int newNumCameras) {
    if (newNumCameras < 0 ||
        static_cast<size_t>(newNumCameras) < newBatch.numCameras()) {
      throw std::invalid_argument("SFM full-normal problem camera mismatch");
    }
    batch = &newBatch;
    numCameras = newNumCameras;
    numPoints = static_cast<int>(newBatch.numPoints());
    dimension = 9 * numCameras + 3 * numPoints;
    linearization = nullptr;
    linearized = false;
    prepared = false;
  }

  void applyNormal(const double* input, double* output,
                   cudaStream_t stream) const {
    if (!prepared || !input || !output) {
      throw std::logic_error("SFM full-normal operator is not prepared");
    }
    GTSAM_CUDA_CHECK(cudaMemsetAsync(
        output, 0, sizeof(double) * static_cast<size_t>(dimension), stream));
    const size_t observations = batch->numObservations();
    if (observations > 0) {
      ApplyNormalKernel<<<Grid(observations), kBlockSize, 0, stream>>>(
          batch->observations().data(), observations,
          linearization->cameraJacobians.data(),
          linearization->pointJacobians.data(), numCameras, input, output);
      GTSAM_CUDA_CHECK(cudaGetLastError());
    }
    AddDampingKernel<<<Grid(dimension), kBlockSize, 0, stream>>>(
        dimension, lambda, dampingDiagonal, input, output);
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
};

CudaSfmFullNormalProblem::CudaSfmFullNormalProblem()
    : impl_(std::make_unique<Impl>()) {}
CudaSfmFullNormalProblem::~CudaSfmFullNormalProblem() = default;
CudaSfmFullNormalProblem::CudaSfmFullNormalProblem(
    CudaSfmFullNormalProblem&&) noexcept = default;
CudaSfmFullNormalProblem& CudaSfmFullNormalProblem::operator=(
    CudaSfmFullNormalProblem&&) noexcept = default;

void CudaSfmFullNormalProblem::initialize(
    const CudaSfmProjectionBatch& batch, int numCameras, cudaStream_t) {
  impl_->setShape(batch, numCameras);
  impl_->explicitSystemInitialized = false;
  impl_->rhs.resize(static_cast<size_t>(impl_->dimension));
  impl_->cameraBlocks.resize(81 * static_cast<size_t>(numCameras));
  impl_->pointBlocks.resize(9 * static_cast<size_t>(impl_->numPoints));
  impl_->cameraFactors.resize(81 * static_cast<size_t>(numCameras));
  impl_->pointFactors.resize(9 * static_cast<size_t>(impl_->numPoints));
}

void CudaSfmFullNormalProblem::initializeSparse(
    const CudaSfmProjectionBatch& batch, int numCameras,
    const std::vector<int>& rowPointers,
    const std::vector<int>& columnIndices, cudaStream_t stream,
    CudaDeviceTransferSummary* transferProfile) {
  impl_->setShape(batch, numCameras);
  impl_->sparseSystem.uploadPattern(impl_->dimension, rowPointers,
                                    columnIndices, stream, transferProfile);
  impl_->explicitSystemInitialized = true;
}

void CudaSfmFullNormalProblem::linearize(
    const CudaSfmProjectionLinearization& linearization,
    cudaStream_t stream) {
  if (!impl_->batch || impl_->dimension <= 0) {
    throw std::logic_error("SFM full-normal problem is not initialized");
  }
  const size_t observations = impl_->batch->numObservations();
  if (linearization.residuals.size() != 2 * observations ||
      linearization.cameraJacobians.size() != 18 * observations ||
      linearization.pointJacobians.size() != 6 * observations) {
    throw std::invalid_argument("SFM full-normal linearization size mismatch");
  }
  impl_->linearization = &linearization;
  if (impl_->explicitSystemInitialized) {
    AccumulateCudaSfmNormalEquations(linearization, *impl_->batch,
                                     impl_->numCameras,
                                     &impl_->sparseSystem, stream);
    impl_->sparseSystem.captureUndampedDiagonal(stream);
    impl_->linearized = true;
    impl_->prepared = false;
    ++impl_->linearizationCount;
    return;
  }
  GTSAM_CUDA_CHECK(cudaMemsetAsync(
      impl_->rhs.data(), 0,
      sizeof(double) * static_cast<size_t>(impl_->dimension), stream));
  impl_->cameraBlocks.zero(stream);
  impl_->pointBlocks.zero(stream);
  if (observations > 0) {
    AccumulateRhsAndBlocksKernel<<<Grid(observations), kBlockSize, 0, stream>>>(
        impl_->batch->observations().data(), observations,
        linearization.residuals.data(), linearization.cameraJacobians.data(),
        linearization.pointJacobians.data(), impl_->numCameras,
        impl_->rhs.data(), impl_->cameraBlocks.data(), impl_->pointBlocks.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  impl_->linearized = true;
  impl_->prepared = false;
  ++impl_->linearizationCount;
}

namespace {
template <class ImplType>
CudaSfmFullNormalView PrepareFullNormal(
    ImplType* impl, double lambda,
    const CudaDeviceArray<double>* dampingDiagonal, cudaStream_t stream) {
  if (!impl->linearized || !std::isfinite(lambda) || lambda <= 0.0) {
    throw std::invalid_argument("invalid SFM full-normal preparation");
  }
  if (dampingDiagonal &&
      dampingDiagonal->size() != static_cast<size_t>(impl->dimension)) {
    throw std::invalid_argument("SFM full-normal damping size mismatch");
  }
  impl->lambda = lambda;
  impl->dampingDiagonal = dampingDiagonal ? dampingDiagonal->data() : nullptr;
  if (impl->numCameras > 0) {
    FactorCameraBlocksKernel<<<Grid(impl->numCameras), kBlockSize, 0, stream>>>(
        impl->numCameras, lambda, impl->dampingDiagonal,
        impl->cameraBlocks.data(), impl->cameraFactors.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  if (impl->numPoints > 0) {
    FactorPointBlocksKernel<<<Grid(impl->numPoints), kBlockSize, 0, stream>>>(
        impl->numPoints, 9 * impl->numCameras, lambda, impl->dampingDiagonal,
        impl->pointBlocks.data(), impl->pointFactors.data());
    GTSAM_CUDA_CHECK(cudaGetLastError());
  }
  impl->prepared = true;
  ++impl->preparationCount;
  return {&impl->linearOperator, &impl->preconditioner, impl->rhs.data()};
}
}  // namespace

CudaSfmFullNormalView CudaSfmFullNormalProblem::prepare(
    double lambda, cudaStream_t stream) {
  if (impl_->explicitSystemInitialized) {
    throw std::logic_error(
        "explicit SFM full-normal problem requires prepareSparse");
  }
  return PrepareFullNormal(impl_.get(), lambda, nullptr, stream);
}

CudaSfmFullNormalView CudaSfmFullNormalProblem::prepare(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (impl_->explicitSystemInitialized) {
    throw std::logic_error(
        "explicit SFM full-normal problem requires prepareSparse");
  }
  return PrepareFullNormal(impl_.get(), lambda, &dampingDiagonal, stream);
}

namespace {
template <class ImplType>
DeviceSparseSpdSystem& PrepareSparseFullNormal(
    ImplType* impl, double lambda,
    const CudaDeviceArray<double>* dampingDiagonal, cudaStream_t stream) {
  if (!impl->explicitSystemInitialized || !impl->linearized ||
      !std::isfinite(lambda) || lambda <= 0.0) {
    throw std::invalid_argument("invalid explicit SFM full-normal preparation");
  }
  if (dampingDiagonal &&
      dampingDiagonal->size() != static_cast<size_t>(impl->dimension)) {
    throw std::invalid_argument("SFM full-normal damping size mismatch");
  }
  if (dampingDiagonal) {
    impl->sparseSystem.restoreAndAddDiagonal(lambda, *dampingDiagonal, stream);
  } else {
    impl->sparseSystem.restoreAndAddDiagonal(lambda, stream);
  }
  impl->prepared = true;
  ++impl->preparationCount;
  return impl->sparseSystem;
}
}  // namespace

DeviceSparseSpdSystem& CudaSfmFullNormalProblem::prepareSparse(
    double lambda, cudaStream_t stream) {
  return PrepareSparseFullNormal(impl_.get(), lambda, nullptr, stream);
}

DeviceSparseSpdSystem& CudaSfmFullNormalProblem::prepareSparse(
    double lambda, const CudaDeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  return PrepareSparseFullNormal(impl_.get(), lambda, &dampingDiagonal,
                                 stream);
}

int CudaSfmFullNormalProblem::dimension() const { return impl_->dimension; }
size_t CudaSfmFullNormalProblem::linearizationCount() const {
  return impl_->linearizationCount;
}
size_t CudaSfmFullNormalProblem::preparationCount() const {
  return impl_->preparationCount;
}

}  // namespace gtsam::cuda
