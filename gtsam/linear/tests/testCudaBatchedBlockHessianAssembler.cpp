/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedBlockHessianAssembler.cpp
 * @brief   Unit tests for the batched GPU block-Hessian assembler
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/cuda/internal/BatchedBlockHessianAssembler.h>

#include <random>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_block_hessian_fixture {

bool HasCudaDevice() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess) {
    (void)cudaGetLastError();
    return false;
  }
  return count > 0;
}

// One batch item's dense Jacobian (jacobianRows x numBlocks*blockDim) and
// RHS (jacobianRows), the shape assemble() consumes.
struct JacobianItem {
  Matrix A;
  Vector b;
};

JacobianItem MakeRandomJacobian(int rows, int dimension, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  JacobianItem item;
  item.A = Matrix(rows, dimension);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < dimension; ++c) item.A(r, c) = dist(rng);
  item.b = Vector(rows);
  for (int r = 0; r < rows; ++r) item.b(r) = dist(rng);
  return item;
}

// Independent CPU ground truth, computed the same way
// BatchedBlockTridiagonalLevenbergMarquardt.cpp's pre-GPU assembly loop did.
void ExpectedBlocks(const JacobianItem& item, int numBlocks, int blockDim,
                    std::vector<Matrix>* diag, std::vector<Matrix>* offDiag,
                    Vector* rhs) {
  diag->resize(static_cast<size_t>(numBlocks));
  offDiag->resize(static_cast<size_t>(numBlocks - 1));
  *rhs = item.A.transpose() * item.b;
  for (int t = 0; t < numBlocks; ++t) {
    const Matrix At = item.A.block(0, t * blockDim, item.A.rows(), blockDim);
    (*diag)[static_cast<size_t>(t)] = At.transpose() * At;
    if (t + 1 < numBlocks) {
      const Matrix At1 =
          item.A.block(0, (t + 1) * blockDim, item.A.rows(), blockDim);
      (*offDiag)[static_cast<size_t>(t)] = At.transpose() * At1;
    }
  }
}

DeviceArray<double> UploadJacobians(const std::vector<JacobianItem>& batch) {
  const int batchSize = static_cast<int>(batch.size());
  const int rows = static_cast<int>(batch.front().A.rows());
  const int dimension = static_cast<int>(batch.front().A.cols());
  std::vector<double> flat(static_cast<size_t>(batchSize) * rows * dimension);
  for (int i = 0; i < batchSize; ++i) {
    Eigen::Map<Matrix>(flat.data() + static_cast<size_t>(i) * rows * dimension,
                       rows, dimension) = batch[static_cast<size_t>(i)].A;
  }
  DeviceArray<double> device(flat.size());
  device.upload(flat);
  return device;
}

DeviceArray<double> UploadRhsVectors(const std::vector<JacobianItem>& batch) {
  const int batchSize = static_cast<int>(batch.size());
  const int rows = static_cast<int>(batch.front().b.size());
  std::vector<double> flat(static_cast<size_t>(batchSize) * rows);
  for (int i = 0; i < batchSize; ++i) {
    Eigen::Map<Vector>(flat.data() + static_cast<size_t>(i) * rows, rows) =
        batch[static_cast<size_t>(i)].b;
  }
  DeviceArray<double> device(flat.size());
  device.upload(flat);
  return device;
}

std::vector<double> Download(const DeviceArray<double>& device) {
  std::vector<double> flat;
  device.download(&flat);
  cudaStreamSynchronize(nullptr);
  return flat;
}

// A small, fixed (not randomized-seed-only) batch: batchSize=3, numBlocks=4,
// blockDim=3, jacobianRows=17 (deliberately not a multiple of blockDim, to
// catch any accidental assumption that rows is block-structured).
TEST(BatchedBlockHessianAssembler, MatchesCpuReference) {
  if (!HasCudaDevice()) return;
  const int batchSize = 3, numBlocks = 4, blockDim = 3, rows = 17;
  const int dimension = numBlocks * blockDim;
  std::vector<JacobianItem> batch;
  for (int i = 0; i < batchSize; ++i)
    batch.push_back(MakeRandomJacobian(rows, dimension, 100 + i));

  DeviceArray<double> deviceA = UploadJacobians(batch);
  DeviceArray<double> deviceB = UploadRhsVectors(batch);
  DeviceArray<double> diagOut, offDiagOut, rhsOut;

  BatchedBlockHessianAssembler assembler;
  assembler.assemble(batchSize, numBlocks, blockDim, rows, &deviceA, &deviceB,
                     &diagOut, &offDiagOut, &rhsOut);

  const std::vector<double> diagFlat = Download(diagOut);
  const std::vector<double> offDiagFlat = Download(offDiagOut);
  const std::vector<double> rhsFlat = Download(rhsOut);

  for (int i = 0; i < batchSize; ++i) {
    std::vector<Matrix> expectedDiag, expectedOffDiag;
    Vector expectedRhs;
    ExpectedBlocks(batch[static_cast<size_t>(i)], numBlocks, blockDim,
                   &expectedDiag, &expectedOffDiag, &expectedRhs);

    const size_t diagItemOffset =
        static_cast<size_t>(i) * numBlocks * blockDim * blockDim;
    for (int t = 0; t < numBlocks; ++t) {
      const Matrix actual = Eigen::Map<const Matrix>(
          diagFlat.data() + diagItemOffset +
              static_cast<size_t>(t) * blockDim * blockDim,
          blockDim, blockDim);
      EXPECT(assert_equal(expectedDiag[static_cast<size_t>(t)], actual,
                          1e-9));
    }

    const size_t offDiagItemOffset =
        static_cast<size_t>(i) * (numBlocks - 1) * blockDim * blockDim;
    for (int t = 0; t < numBlocks - 1; ++t) {
      const Matrix actual = Eigen::Map<const Matrix>(
          offDiagFlat.data() + offDiagItemOffset +
              static_cast<size_t>(t) * blockDim * blockDim,
          blockDim, blockDim);
      EXPECT(assert_equal(expectedOffDiag[static_cast<size_t>(t)], actual,
                          1e-9));
    }

    const Vector actualRhs = Eigen::Map<const Vector>(
        rhsFlat.data() + static_cast<size_t>(i) * dimension, dimension);
    EXPECT(assert_equal(expectedRhs, actualRhs, 1e-9));
  }
}

// batchSize=1 has no cross-item boundary to discard a raw pair across --
// exercises that edge of the offDiag discard copy on its own.
TEST(BatchedBlockHessianAssembler, SingleItemBatch) {
  if (!HasCudaDevice()) return;
  const int numBlocks = 5, blockDim = 2, rows = 9;
  const std::vector<JacobianItem> batch = {
      MakeRandomJacobian(rows, numBlocks * blockDim, 777)};

  DeviceArray<double> deviceA = UploadJacobians(batch);
  DeviceArray<double> deviceB = UploadRhsVectors(batch);
  DeviceArray<double> diagOut, offDiagOut, rhsOut;
  BatchedBlockHessianAssembler assembler;
  assembler.assemble(1, numBlocks, blockDim, rows, &deviceA, &deviceB,
                     &diagOut, &offDiagOut, &rhsOut);

  std::vector<Matrix> expectedDiag, expectedOffDiag;
  Vector expectedRhs;
  ExpectedBlocks(batch.front(), numBlocks, blockDim, &expectedDiag,
                &expectedOffDiag, &expectedRhs);

  const std::vector<double> offDiagFlat = Download(offDiagOut);
  for (int t = 0; t < numBlocks - 1; ++t) {
    const Matrix actual = Eigen::Map<const Matrix>(
        offDiagFlat.data() + static_cast<size_t>(t) * blockDim * blockDim,
        blockDim, blockDim);
    EXPECT(assert_equal(expectedOffDiag[static_cast<size_t>(t)], actual,
                        1e-9));
  }
}

}  // namespace batched_block_hessian_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
