/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianNormalOperator.cu
 * @brief   Matrix-free J'J + lambda D operator and its Jacobi preconditioner
 * @author  Ruogu Li
 * @date    Aug 16, 2026
 */

#include <gtsam/nonlinear/cuda/internal/JacobianNormalOperator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace gtsam::cuda {
namespace {

constexpr int kBlockSize = 256;
constexpr int kMaxBlockWidth = 16;
constexpr double kPivotRelativeTolerance = 1e-14;
constexpr double kTinyDiagonal = 1e-300;
constexpr double kAlpha = 1.0;
constexpr double kBeta = 0.0;

int gridSize(int count) {
  return count / kBlockSize + (count % kBlockSize != 0);
}

void checkRuntimeStatus(cudaError_t status, const char* stage) {
  if (status == cudaSuccess) return;
  std::ostringstream message;
  message << "CUDA Jacobian-normal failure during " << stage << ": "
          << cudaGetErrorName(status) << " (" << cudaGetErrorString(status)
          << ")";
  throw std::runtime_error(message.str());
}

void checkCusparse(cusparseStatus_t status, const char* stage) {
  if (status == CUSPARSE_STATUS_SUCCESS) return;
  std::ostringstream message;
  message << "cuSPARSE Jacobian-normal failure during " << stage << ": "
          << cusparseGetErrorName(status) << " ("
          << cusparseGetErrorString(status) << ")";
  throw std::runtime_error(message.str());
}

__device__ int packedIndex(int row, int column) {
  return row * (row + 1) / 2 + column;
}

__global__ void addDampingKernel(double* output, const double* input,
                                 const double* diagonal, int count,
                                 double lambda) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < count) output[index] += lambda * diagonal[index] * input[index];
}

__global__ void columnSquaredNormsKernel(const int* rowPointers,
                                         const double* values, int columns,
                                         double* diagonal) {
  const int column = blockIdx.x * blockDim.x + threadIdx.x;
  if (column >= columns) return;
  double sum = 0.0;
  for (int index = rowPointers[column]; index < rowPointers[column + 1];
       ++index) {
    const double value = values[index];
    sum += value * value;
  }
  diagonal[column] = sum;
}

__global__ void buildGramBlocksKernel(const int* columnBegins, int count,
                                      int width, const int* rowPointers,
                                      const double* values,
                                      double* gramBlocks) {
  const int warpsPerBlock = blockDim.x / 32;
  const int variable = blockIdx.x * warpsPerBlock + threadIdx.x / 32;
  if (variable >= count) return;
  const int lane = threadIdx.x % 32;
  const int packedCount = width * (width + 1) / 2;
  const int firstColumn = columnBegins[variable];
  const int base = rowPointers[firstColumn];
  const int entries = rowPointers[firstColumn + 1] - base;

  double accumulators[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int entry = 0; entry < packedCount; ++entry) accumulators[entry] = 0.0;
  double slice[kMaxBlockWidth];
  for (int j = lane; j < entries; j += 32) {
    for (int k = 0; k < width; ++k) {
      slice[k] = values[base + k * entries + j];
    }
    for (int row = 0; row < width; ++row) {
      for (int column = 0; column <= row; ++column) {
        accumulators[packedIndex(row, column)] +=
            slice[row] * slice[column];
      }
    }
  }
  for (int entry = 0; entry < packedCount; ++entry) {
    double value = accumulators[entry];
    for (int offset = 16; offset > 0; offset /= 2) {
      value += __shfl_down_sync(0xffffffff, value, offset);
    }
    if (lane == 0) gramBlocks[entry * count + variable] = value;
  }
}

__global__ void dampInvertBlocksKernel(const double* gramBlocks,
                                       const int* columnBegins, int count,
                                       int width,
                                       const double* dampingDiagonal,
                                       double lambda,
                                       double* inverseBlocks) {
  const int variable = blockIdx.x * blockDim.x + threadIdx.x;
  if (variable >= count) return;
  const int packedCount = width * (width + 1) / 2;
  const int firstColumn = columnBegins[variable];
  double damped[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int entry = 0; entry < packedCount; ++entry) {
    damped[entry] = gramBlocks[entry * count + variable];
  }
  double maxDiagonal = 0.0;
  for (int i = 0; i < width; ++i) {
    damped[packedIndex(i, i)] +=
        lambda * dampingDiagonal[firstColumn + i];
    maxDiagonal = fmax(maxDiagonal, fabs(damped[packedIndex(i, i)]));
  }

  bool failed = false;
  for (int column = 0; column < width && !failed; ++column) {
    double pivot = damped[packedIndex(column, column)];
    for (int k = 0; k < column; ++k) {
      const double value = damped[packedIndex(column, k)];
      pivot -= value * value;
    }
    if (!isfinite(pivot) || pivot <= kPivotRelativeTolerance * maxDiagonal) {
      failed = true;
      break;
    }
    damped[packedIndex(column, column)] = sqrt(pivot);
    for (int row = column + 1; row < width; ++row) {
      double value = damped[packedIndex(row, column)];
      for (int k = 0; k < column; ++k) {
        value -= damped[packedIndex(row, k)] *
                 damped[packedIndex(column, k)];
      }
      damped[packedIndex(row, column)] =
          value / damped[packedIndex(column, column)];
    }
  }

  if (failed) {
    for (int entry = 0; entry < packedCount; ++entry) {
      inverseBlocks[entry * count + variable] = 0.0;
    }
    for (int i = 0; i < width; ++i) {
      const double diagonal = gramBlocks[packedIndex(i, i) * count + variable] +
                              lambda * dampingDiagonal[firstColumn + i];
      inverseBlocks[packedIndex(i, i) * count + variable] =
          1.0 / fmax(diagonal, kTinyDiagonal);
    }
    return;
  }

  double inverseFactor[kMaxBlockWidth * (kMaxBlockWidth + 1) / 2];
  for (int column = 0; column < width; ++column) {
    inverseFactor[packedIndex(column, column)] =
        1.0 / damped[packedIndex(column, column)];
    for (int row = column + 1; row < width; ++row) {
      double value = 0.0;
      for (int k = column; k < row; ++k) {
        value -= damped[packedIndex(row, k)] *
                 inverseFactor[packedIndex(k, column)];
      }
      inverseFactor[packedIndex(row, column)] =
          value / damped[packedIndex(row, row)];
    }
  }
  for (int row = 0; row < width; ++row) {
    for (int column = 0; column <= row; ++column) {
      double value = 0.0;
      for (int k = row; k < width; ++k) {
        value += inverseFactor[packedIndex(k, row)] *
                 inverseFactor[packedIndex(k, column)];
      }
      inverseBlocks[packedIndex(row, column) * count + variable] = value;
    }
  }
}

__global__ void applyBlockInverseKernel(const double* inverseBlocks,
                                        const int* columnBegins, int count,
                                        int width, const double* input,
                                        double* output) {
  const int variable = blockIdx.x * blockDim.x + threadIdx.x;
  if (variable >= count) return;
  const int firstColumn = columnBegins[variable];
  double local[kMaxBlockWidth];
  for (int i = 0; i < width; ++i) local[i] = input[firstColumn + i];
  for (int row = 0; row < width; ++row) {
    double sum = 0.0;
    for (int column = 0; column < width; ++column) {
      const int entry = row >= column ? packedIndex(row, column)
                                      : packedIndex(column, row);
      sum += inverseBlocks[entry * count + variable] * local[column];
    }
    output[firstColumn + row] = sum;
  }
}

__global__ void applyJacobiKernel(const double* undampedDiagonal,
                                  const double* dampingDiagonal,
                                  double lambda, const double* input,
                                  double* output, int count) {
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= count) return;
  const double denominator =
      fmax(undampedDiagonal[index] + lambda * dampingDiagonal[index],
           kTinyDiagonal);
  output[index] = input[index] / denominator;
}

struct WidthClass {
  int width = 0;
  int count = 0;
  DeviceArray<int> columnBegins;
  DeviceArray<double> gramBlocks;
  DeviceArray<double> inverseBlocks;
};

}  // namespace

struct JacobianNormalOperator::Impl {
  cusparseHandle_t handle = nullptr;
  cusparseSpMatDescr_t jacobian = nullptr;
  cusparseSpMatDescr_t transpose = nullptr;
  cusparseDnVecDescr_t inputDescriptor = nullptr;
  cusparseDnVecDescr_t intermediateDescriptor = nullptr;
  cusparseDnVecDescr_t outputDescriptor = nullptr;
  DeviceArray<double> intermediate;
  DeviceArray<unsigned char> forwardWorkspace;
  DeviceArray<unsigned char> transposeWorkspace;
  int rows = 0;
  int columns = 0;
  double lambda = 0.0;
  const double* dampingDiagonal = nullptr;
  cudaStream_t fixedStream = nullptr;
  cusparseSpMVAlg_t algorithm = CUSPARSE_SPMV_ALG_DEFAULT;

  ~Impl() {
    if (inputDescriptor || intermediateDescriptor || outputDescriptor) {
      cudaStreamSynchronize(fixedStream);
    }
    if (outputDescriptor) cusparseDestroyDnVec(outputDescriptor);
    if (intermediateDescriptor) cusparseDestroyDnVec(intermediateDescriptor);
    if (inputDescriptor) cusparseDestroyDnVec(inputDescriptor);
  }
};

JacobianNormalOperator::JacobianNormalOperator()
    : impl_(std::make_unique<Impl>()) {}
JacobianNormalOperator::~JacobianNormalOperator() = default;
JacobianNormalOperator::JacobianNormalOperator(
    JacobianNormalOperator&&) noexcept = default;
JacobianNormalOperator& JacobianNormalOperator::operator=(
    JacobianNormalOperator&&) noexcept = default;

void JacobianNormalOperator::initialize(
    cusparseHandle_t handle, int rows, int columns,
    cusparseSpMatDescr_t jacobian, cusparseSpMatDescr_t jacobianTranspose,
    cudaStream_t stream) {
  if (!handle || !jacobian || !jacobianTranspose || rows <= 0 || columns <= 0) {
    throw std::invalid_argument(
        "JacobianNormalOperator requires valid descriptors and positive "
        "dimensions");
  }
  auto state = std::make_unique<Impl>();
  state->handle = handle;
  state->jacobian = jacobian;
  state->transpose = jacobianTranspose;
  state->rows = rows;
  state->columns = columns;
  state->fixedStream = stream;
  state->intermediate.resize(static_cast<size_t>(rows));
  state->intermediate.zero(stream);
  if (const char* name = std::getenv("GTSAM_PCG_SPMV_ALG")) {
    if (std::string(name) == "alg1") state->algorithm = CUSPARSE_SPMV_CSR_ALG1;
    if (std::string(name) == "alg2") state->algorithm = CUSPARSE_SPMV_CSR_ALG2;
  }
  checkCusparse(cusparseCreateDnVec(&state->inputDescriptor, columns,
                                    nullptr, CUDA_R_64F),
                "input descriptor creation");
  checkCusparse(cusparseCreateDnVec(&state->intermediateDescriptor, rows,
                                    state->intermediate.data(), CUDA_R_64F),
                "intermediate descriptor creation");
  checkCusparse(cusparseCreateDnVec(&state->outputDescriptor, columns,
                                    nullptr, CUDA_R_64F),
                "output descriptor creation");
  // cuSPARSE rejects null dense-vector values on some toolkit versions.
  // Point the borrowed descriptors at valid storage until the first apply.
  checkCusparse(cusparseDnVecSetValues(state->inputDescriptor,
                                      state->intermediate.data()),
                "initial input binding");
  checkCusparse(cusparseDnVecSetValues(state->outputDescriptor,
                                      state->intermediate.data()),
                "initial output binding");
  size_t bytes = 0;
  checkCusparse(cusparseSpMV_bufferSize(
                    handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    jacobian, state->inputDescriptor, &kBeta,
                    state->intermediateDescriptor, CUDA_R_64F,
                    state->algorithm, &bytes),
                "forward workspace query");
  state->forwardWorkspace.resize(bytes);
  bytes = 0;
  checkCusparse(cusparseSpMV_bufferSize(
                    handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    jacobianTranspose, state->intermediateDescriptor, &kBeta,
                    state->outputDescriptor, CUDA_R_64F, state->algorithm,
                    &bytes),
                "transpose workspace query");
  state->transposeWorkspace.resize(bytes);
  checkCusparse(cusparseSpMV_preprocess(
                    handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    jacobian, state->inputDescriptor, &kBeta,
                    state->intermediateDescriptor, CUDA_R_64F,
                    state->algorithm, state->forwardWorkspace.data()),
                "forward preprocess");
  checkCusparse(cusparseSpMV_preprocess(
                    handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    jacobianTranspose, state->intermediateDescriptor, &kBeta,
                    state->outputDescriptor, CUDA_R_64F, state->algorithm,
                    state->transposeWorkspace.data()),
                "transpose preprocess");
  impl_ = std::move(state);
}

void JacobianNormalOperator::setDamping(
    double lambda, const DeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (impl_->columns <= 0) {
    throw std::logic_error("JacobianNormalOperator is not initialized");
  }
  if (stream != impl_->fixedStream || !std::isfinite(lambda) || lambda < 0.0 ||
      dampingDiagonal.size() != static_cast<size_t>(impl_->columns)) {
    throw std::invalid_argument(
        "JacobianNormalOperator received invalid damping or stream");
  }
  impl_->lambda = lambda;
  impl_->dampingDiagonal = dampingDiagonal.data();
}

int JacobianNormalOperator::dimension() const { return impl_->columns; }

void JacobianNormalOperator::apply(const double* input, double* output,
                                       cudaStream_t stream) const {
  if (!input || !output || !impl_->dampingDiagonal ||
      stream != impl_->fixedStream) {
    throw std::invalid_argument(
        "JacobianNormalOperator apply requires prepared storage and the "
        "fixed stream");
  }
  checkCusparse(cusparseDnVecSetValues(impl_->inputDescriptor,
                                      const_cast<double*>(input)),
                "input binding");
  checkCusparse(cusparseDnVecSetValues(impl_->outputDescriptor, output),
                "output binding");
  checkCusparse(cusparseSpMV(
                    impl_->handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    impl_->jacobian, impl_->inputDescriptor, &kBeta,
                    impl_->intermediateDescriptor, CUDA_R_64F, impl_->algorithm,
                    impl_->forwardWorkspace.data()),
                "J*x");
  checkCusparse(cusparseSpMV(
                    impl_->handle, CUSPARSE_OPERATION_NON_TRANSPOSE, &kAlpha,
                    impl_->transpose, impl_->intermediateDescriptor, &kBeta,
                    impl_->outputDescriptor, CUDA_R_64F, impl_->algorithm,
                    impl_->transposeWorkspace.data()),
                "J'*(J*x)");
  addDampingKernel<<<gridSize(impl_->columns), kBlockSize, 0, stream>>>(
      output, input, impl_->dampingDiagonal, impl_->columns, impl_->lambda);
  checkRuntimeStatus(cudaGetLastError(), "damping tail launch");
}

struct JacobianNormalPreconditioner::Impl {
  int columns = 0;
  const int* rowPointers = nullptr;
  DevicePcgPreconditioner type = DevicePcgPreconditioner::BlockJacobi;
  std::vector<WidthClass> widthClasses;
  DeviceArray<double> undampedDiagonal;
  const double* dampingDiagonal = nullptr;
  double lambda = 0.0;
  cudaStream_t fixedStream = nullptr;
  bool collectProfile = false;
  double buildTime = 0.0;
  double prepareTime = 0.0;
  bool built = false;
  bool prepared = false;
};

JacobianNormalPreconditioner::JacobianNormalPreconditioner()
    : impl_(std::make_unique<Impl>()) {}
JacobianNormalPreconditioner::~JacobianNormalPreconditioner() = default;
JacobianNormalPreconditioner::JacobianNormalPreconditioner(
    JacobianNormalPreconditioner&&) noexcept = default;
JacobianNormalPreconditioner&
JacobianNormalPreconditioner::operator=(
    JacobianNormalPreconditioner&&) noexcept = default;

void JacobianNormalPreconditioner::initialize(
    int columns, const DeviceArray<int>& transposeRowPointers,
    const std::vector<int>& blockOffsets, DevicePcgPreconditioner type,
    cudaStream_t stream, bool collectProfile) {
  if (columns <= 0 || transposeRowPointers.size() !=
                          static_cast<size_t>(columns) + 1 ||
      blockOffsets.size() < 2 || blockOffsets.front() != 0 ||
      blockOffsets.back() != columns) {
    throw std::invalid_argument(
        "JacobianNormalPreconditioner has invalid dimensions or blocks");
  }
  for (size_t i = 1; i < blockOffsets.size(); ++i) {
    const int width = blockOffsets[i] - blockOffsets[i - 1];
    if (width <= 0 || width > kMaxBlockWidth) {
      throw std::invalid_argument(
          "JacobianNormalPreconditioner block widths must be in [1,16]");
    }
  }
  auto state = std::make_unique<Impl>();
  state->columns = columns;
  state->rowPointers = transposeRowPointers.data();
  state->type = type;
  state->fixedStream = stream;
  state->collectProfile = collectProfile;

  std::vector<int> hostRows(static_cast<size_t>(columns) + 1);
  checkRuntimeStatus(cudaMemcpyAsync(hostRows.data(), transposeRowPointers.data(),
                            sizeof(int) * hostRows.size(),
                            cudaMemcpyDeviceToHost, stream),
            "row-pointer validation download");
  checkRuntimeStatus(cudaStreamSynchronize(stream), "row-pointer validation sync");
  std::map<int, std::vector<int>> beginsByWidth;
  for (size_t i = 1; i < blockOffsets.size(); ++i) {
    const int begin = blockOffsets[i - 1];
    const int end = blockOffsets[i];
    const int expected = hostRows[begin + 1] - hostRows[begin];
    for (int column = begin + 1; column < end; ++column) {
      if (hostRows[column + 1] - hostRows[column] != expected) {
        throw std::runtime_error(
            "JacobianNormalPreconditioner requires equal transpose "
            "column patterns within each variable block");
      }
    }
    beginsByWidth[end - begin].push_back(begin);
  }
  if (type == DevicePcgPreconditioner::BlockJacobi) {
    for (auto& [width, begins] : beginsByWidth) {
      WidthClass widthClass;
      widthClass.width = width;
      widthClass.count = static_cast<int>(begins.size());
      widthClass.columnBegins.upload(begins, stream);
      const size_t packed = static_cast<size_t>(width) * (width + 1) / 2 *
                            begins.size();
      widthClass.gramBlocks.resize(packed);
      widthClass.inverseBlocks.resize(packed);
      widthClass.gramBlocks.zero(stream);
      widthClass.inverseBlocks.zero(stream);
      state->widthClasses.push_back(std::move(widthClass));
    }
  } else if (type == DevicePcgPreconditioner::Jacobi) {
    state->undampedDiagonal.resize(static_cast<size_t>(columns));
    state->undampedDiagonal.zero(stream);
  }
  impl_ = std::move(state);
}

void JacobianNormalPreconditioner::build(
    const DeviceArray<double>& transposeValues, cudaStream_t stream) {
  if (impl_->columns <= 0 || stream != impl_->fixedStream) {
    throw std::logic_error(
        "JacobianNormalPreconditioner is not initialized on this stream");
  }
  const auto start = std::chrono::steady_clock::now();
  if (impl_->type == DevicePcgPreconditioner::BlockJacobi) {
    for (WidthClass& widthClass : impl_->widthClasses) {
      const int warps = kBlockSize / 32;
      const int grid = widthClass.count / warps +
                       (widthClass.count % warps != 0);
      buildGramBlocksKernel<<<grid, kBlockSize, 0, stream>>>(
          widthClass.columnBegins.data(), widthClass.count, widthClass.width,
          impl_->rowPointers, transposeValues.data(),
          widthClass.gramBlocks.data());
      checkRuntimeStatus(cudaGetLastError(), "Gram block build launch");
    }
  } else if (impl_->type == DevicePcgPreconditioner::Jacobi) {
    columnSquaredNormsKernel<<<gridSize(impl_->columns), kBlockSize, 0,
                               stream>>>(impl_->rowPointers,
                                         transposeValues.data(), impl_->columns,
                                         impl_->undampedDiagonal.data());
    checkRuntimeStatus(cudaGetLastError(), "Jacobi diagonal build launch");
  }
  impl_->built = true;
  impl_->prepared = false;
  if (impl_->collectProfile) {
    checkRuntimeStatus(cudaStreamSynchronize(stream), "profiled build sync");
    impl_->buildTime +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  }
}

void JacobianNormalPreconditioner::prepare(
    double lambda, const DeviceArray<double>& dampingDiagonal,
    cudaStream_t stream) {
  if (!impl_->built || stream != impl_->fixedStream ||
      !std::isfinite(lambda) || lambda < 0.0 ||
      dampingDiagonal.size() != static_cast<size_t>(impl_->columns)) {
    throw std::invalid_argument(
        "JacobianNormalPreconditioner requires built data and valid "
        "damping");
  }
  const auto start = std::chrono::steady_clock::now();
  impl_->lambda = lambda;
  impl_->dampingDiagonal = dampingDiagonal.data();
  if (impl_->type == DevicePcgPreconditioner::BlockJacobi) {
    for (WidthClass& widthClass : impl_->widthClasses) {
      dampInvertBlocksKernel<<<gridSize(widthClass.count), kBlockSize, 0,
                               stream>>>(
          widthClass.gramBlocks.data(), widthClass.columnBegins.data(),
          widthClass.count, widthClass.width, dampingDiagonal.data(), lambda,
          widthClass.inverseBlocks.data());
      checkRuntimeStatus(cudaGetLastError(), "damped block inversion launch");
    }
  }
  impl_->prepared = true;
  if (impl_->collectProfile) {
    checkRuntimeStatus(cudaStreamSynchronize(stream), "profiled prepare sync");
    impl_->prepareTime +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
  }
}

int JacobianNormalPreconditioner::dimension() const {
  return impl_->columns;
}

void JacobianNormalPreconditioner::apply(const double* input,
                                             double* output,
                                             cudaStream_t stream) const {
  if (!impl_->prepared || !input || !output || stream != impl_->fixedStream) {
    throw std::invalid_argument(
        "JacobianNormalPreconditioner apply requires prepared storage "
        "and the fixed stream");
  }
  switch (impl_->type) {
    case DevicePcgPreconditioner::BlockJacobi:
      for (const WidthClass& widthClass : impl_->widthClasses) {
        applyBlockInverseKernel<<<gridSize(widthClass.count), kBlockSize, 0,
                                  stream>>>(
            widthClass.inverseBlocks.data(), widthClass.columnBegins.data(),
            widthClass.count, widthClass.width, input, output);
        checkRuntimeStatus(cudaGetLastError(), "block preconditioner apply launch");
      }
      break;
    case DevicePcgPreconditioner::Jacobi:
      applyJacobiKernel<<<gridSize(impl_->columns), kBlockSize, 0, stream>>>(
          impl_->undampedDiagonal.data(), impl_->dampingDiagonal, impl_->lambda,
          input, output, impl_->columns);
      checkRuntimeStatus(cudaGetLastError(), "Jacobi preconditioner apply launch");
      break;
    case DevicePcgPreconditioner::None:
      checkRuntimeStatus(cudaMemcpyAsync(output, input,
                                sizeof(double) *
                                    static_cast<size_t>(impl_->columns),
                                cudaMemcpyDeviceToDevice, stream),
                "identity preconditioner copy");
      break;
  }
}

double JacobianNormalPreconditioner::buildSeconds() const {
  return impl_->buildTime;
}

double JacobianNormalPreconditioner::prepareSeconds() const {
  return impl_->prepareTime;
}

}  // namespace gtsam::cuda
