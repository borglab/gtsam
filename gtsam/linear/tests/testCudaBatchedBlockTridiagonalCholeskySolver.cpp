/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedBlockTridiagonalCholeskySolver.cpp
 * @brief   Unit tests for the batched block-tridiagonal cuSOLVER/cuBLAS
 *          Cholesky backend
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/cuda/internal/BatchedBlockTridiagonalCholeskySolver.h>

#include <Eigen/Cholesky>
#include <random>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_block_tridiagonal_fixture {

bool HasCudaDevice() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess) {
    (void)cudaGetLastError();
    return false;
  }
  return count > 0;
}

// One batch item's block-tridiagonal system: numBlocks diagonal blocks and
// numBlocks-1 upper off-diagonal blocks, every block blockDim x blockDim.
struct BlockItem {
  std::vector<Matrix> diagonal;    // size numBlocks
  std::vector<Matrix> offDiagonal; // size numBlocks - 1, offDiagonal[t] = H_{t,t+1}
};

// Assembles one item's blocks into the equivalent dense SPD matrix, for an
// independent Eigen ground truth.
Matrix AssembleDense(const BlockItem& item) {
  const int numBlocks = static_cast<int>(item.diagonal.size());
  const int blockDim = static_cast<int>(item.diagonal.front().rows());
  Matrix dense = Matrix::Zero(numBlocks * blockDim, numBlocks * blockDim);
  for (int t = 0; t < numBlocks; ++t) {
    dense.block(t * blockDim, t * blockDim, blockDim, blockDim) =
        item.diagonal[static_cast<size_t>(t)];
  }
  for (int t = 0; t < numBlocks - 1; ++t) {
    dense.block(t * blockDim, (t + 1) * blockDim, blockDim, blockDim) =
        item.offDiagonal[static_cast<size_t>(t)];
    dense.block((t + 1) * blockDim, t * blockDim, blockDim, blockDim) =
        item.offDiagonal[static_cast<size_t>(t)].transpose();
  }
  return dense;
}

// A random, diagonally-dominant (hence SPD) block-tridiagonal item: diag
// blocks are M^T M + blockDim*I, off-diag blocks scaled down to keep SPD.
BlockItem MakeRandomBlockItem(int numBlocks, int blockDim, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  BlockItem item;
  for (int t = 0; t < numBlocks; ++t) {
    Matrix M(blockDim, blockDim);
    for (int r = 0; r < blockDim; ++r)
      for (int c = 0; c < blockDim; ++c) M(r, c) = dist(rng);
    item.diagonal.push_back(M.transpose() * M +
                           blockDim * Matrix::Identity(blockDim, blockDim));
  }
  for (int t = 0; t < numBlocks - 1; ++t) {
    Matrix M(blockDim, blockDim);
    for (int r = 0; r < blockDim; ++r)
      for (int c = 0; c < blockDim; ++c) M(r, c) = 0.1 * dist(rng);
    item.offDiagonal.push_back(M);
  }
  return item;
}

// Uploads a batch's diagonal blocks into one contiguous DeviceArray, in the
// batch-item-major layout BatchedBlockTridiagonalCholeskySolver expects.
DeviceArray<double> UploadDiagonal(const std::vector<BlockItem>& batch) {
  const int batchSize = static_cast<int>(batch.size());
  const int numBlocks = static_cast<int>(batch.front().diagonal.size());
  const int blockDim = static_cast<int>(batch.front().diagonal.front().rows());
  std::vector<double> flat(static_cast<size_t>(batchSize) * numBlocks *
                           blockDim * blockDim);
  size_t offset = 0;
  for (const auto& item : batch) {
    for (const auto& block : item.diagonal) {
      Eigen::Map<Matrix>(flat.data() + offset, blockDim, blockDim) = block;
      offset += static_cast<size_t>(blockDim) * blockDim;
    }
  }
  DeviceArray<double> device(flat.size());
  device.upload(flat);
  return device;
}

DeviceArray<double> UploadOffDiagonal(const std::vector<BlockItem>& batch) {
  const int batchSize = static_cast<int>(batch.size());
  const int numBlocks = static_cast<int>(batch.front().diagonal.size());
  const int blockDim = static_cast<int>(batch.front().diagonal.front().rows());
  std::vector<double> flat(static_cast<size_t>(batchSize) * (numBlocks - 1) *
                           blockDim * blockDim);
  size_t offset = 0;
  for (const auto& item : batch) {
    for (const auto& block : item.offDiagonal) {
      Eigen::Map<Matrix>(flat.data() + offset, blockDim, blockDim) = block;
      offset += static_cast<size_t>(blockDim) * blockDim;
    }
  }
  DeviceArray<double> device(flat.size());
  device.upload(flat);
  return device;
}

// Uploads k right-hand-side columns per item (batch-item-major, numBlocks
// contiguous blockDim*k column-major blocks per item).
DeviceArray<double> UploadRhs(const std::vector<std::vector<Matrix>>& rhsPerItem) {
  const int batchSize = static_cast<int>(rhsPerItem.size());
  const int numBlocks = static_cast<int>(rhsPerItem.front().size());
  const int blockDim = static_cast<int>(rhsPerItem.front().front().rows());
  const int k = static_cast<int>(rhsPerItem.front().front().cols());
  std::vector<double> flat(static_cast<size_t>(batchSize) * numBlocks *
                           blockDim * k);
  size_t offset = 0;
  for (const auto& item : rhsPerItem) {
    for (const auto& block : item) {
      Eigen::Map<Matrix>(flat.data() + offset, blockDim, k) = block;
      offset += static_cast<size_t>(blockDim) * k;
    }
  }
  DeviceArray<double> device(flat.size());
  device.upload(flat);
  return device;
}

std::vector<std::vector<Matrix>> DownloadRhs(const DeviceArray<double>& device,
                                            int batchSize, int numBlocks,
                                            int blockDim, int k) {
  std::vector<double> flat;
  device.download(&flat);
  cudaStreamSynchronize(nullptr);
  std::vector<std::vector<Matrix>> result(static_cast<size_t>(batchSize));
  size_t offset = 0;
  for (int b = 0; b < batchSize; ++b) {
    for (int t = 0; t < numBlocks; ++t) {
      result[static_cast<size_t>(b)].push_back(
          Eigen::Map<const Matrix>(flat.data() + offset, blockDim, k));
      offset += static_cast<size_t>(blockDim) * k;
    }
  }
  return result;
}

}  // namespace batched_block_tridiagonal_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace batched_block_tridiagonal_fixture {

// A small, fixed (not randomized) batch, so failures are exactly
// reproducible: batchSize=2, numBlocks=3, blockDim=2.
TEST(BatchedBlockTridiagonalCholeskySolver, FactorizeAndSolveMatchesDenseReference) {
  if (!HasCudaDevice()) return;
  const std::vector<BlockItem> batch = {MakeRandomBlockItem(3, 2, 7),
                                       MakeRandomBlockItem(3, 2, 11)};
  const int batchSize = 2, numBlocks = 3, blockDim = 2;

  std::vector<Vector> b = {Vector::Constant(6, 1.0), Vector::Constant(6, -0.5)};
  std::vector<Vector> expected;
  for (int i = 0; i < batchSize; ++i) {
    const Matrix dense = AssembleDense(batch[static_cast<size_t>(i)]);
    expected.push_back(dense.ldlt().solve(b[static_cast<size_t>(i)]));
  }

  std::vector<std::vector<Matrix>> rhsBlocks(static_cast<size_t>(batchSize));
  for (int i = 0; i < batchSize; ++i)
    for (int t = 0; t < numBlocks; ++t)
      rhsBlocks[static_cast<size_t>(i)].push_back(
          b[static_cast<size_t>(i)].segment(t * blockDim, blockDim));

  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  DeviceArray<double> rhs = UploadRhs(rhsBlocks);

  BatchedBlockTridiagonalCholeskySolver solver;
  solver.factorize(batchSize, numBlocks, blockDim, &diagonal, &offDiagonal);
  solver.solve(1, &rhs);

  const auto result = DownloadRhs(rhs, batchSize, numBlocks, blockDim, 1);
  for (int i = 0; i < batchSize; ++i) {
    Vector actual(numBlocks * blockDim);
    for (int t = 0; t < numBlocks; ++t)
      actual.segment(t * blockDim, blockDim) =
          result[static_cast<size_t>(i)][static_cast<size_t>(t)].col(0);
    EXPECT(assert_equal(expected[static_cast<size_t>(i)], actual, 1e-9));
  }
}

// k=3 right-hand-side columns solved in one call must match three
// independent single-column solves against the same factorization.
TEST(BatchedBlockTridiagonalCholeskySolver, MultipleRhsColumnsMatchIndependentSolves) {
  if (!HasCudaDevice()) return;
  const std::vector<BlockItem> batch = {MakeRandomBlockItem(4, 3, 21)};
  const int batchSize = 1, numBlocks = 4, blockDim = 3, k = 3;
  const Matrix dense = AssembleDense(batch.front());

  std::mt19937 rng(99);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Matrix bAll(numBlocks * blockDim, k);
  for (int r = 0; r < bAll.rows(); ++r)
    for (int c = 0; c < bAll.cols(); ++c) bAll(r, c) = dist(rng);

  Matrix expected(numBlocks * blockDim, k);
  for (int c = 0; c < k; ++c) expected.col(c) = dense.ldlt().solve(bAll.col(c));

  std::vector<std::vector<Matrix>> rhsBlocks(1);
  for (int t = 0; t < numBlocks; ++t)
    rhsBlocks[0].push_back(bAll.block(t * blockDim, 0, blockDim, k));

  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  DeviceArray<double> rhs = UploadRhs(rhsBlocks);

  BatchedBlockTridiagonalCholeskySolver solver;
  solver.factorize(batchSize, numBlocks, blockDim, &diagonal, &offDiagonal);
  solver.solve(k, &rhs);

  const auto result = DownloadRhs(rhs, batchSize, numBlocks, blockDim, k);
  Matrix actual(numBlocks * blockDim, k);
  for (int t = 0; t < numBlocks; ++t)
    actual.block(t * blockDim, 0, blockDim, k) = result[0][static_cast<size_t>(t)];
  EXPECT(assert_equal(expected, actual, 1e-9));
}

// A larger, closer-to-production shape (numBlocks=8, blockDim=6, batchSize=4)
// against Eigen ground truth, exercising the full sequential chain.
TEST(BatchedBlockTridiagonalCholeskySolver, LargerRandomBatchMatchesEigen) {
  if (!HasCudaDevice()) return;
  const int batchSize = 4, numBlocks = 8, blockDim = 6;
  std::vector<BlockItem> batch;
  for (int i = 0; i < batchSize; ++i)
    batch.push_back(MakeRandomBlockItem(numBlocks, blockDim, 1000 + i));

  std::mt19937 rng(555);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  std::vector<Vector> b(static_cast<size_t>(batchSize));
  std::vector<Vector> expected(static_cast<size_t>(batchSize));
  std::vector<std::vector<Matrix>> rhsBlocks(static_cast<size_t>(batchSize));
  for (int i = 0; i < batchSize; ++i) {
    b[static_cast<size_t>(i)] = Vector(numBlocks * blockDim);
    for (int r = 0; r < numBlocks * blockDim; ++r) b[static_cast<size_t>(i)](r) = dist(rng);
    const Matrix dense = AssembleDense(batch[static_cast<size_t>(i)]);
    expected[static_cast<size_t>(i)] = dense.ldlt().solve(b[static_cast<size_t>(i)]);
    for (int t = 0; t < numBlocks; ++t)
      rhsBlocks[static_cast<size_t>(i)].push_back(
          b[static_cast<size_t>(i)].segment(t * blockDim, blockDim));
  }

  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  DeviceArray<double> rhs = UploadRhs(rhsBlocks);

  BatchedBlockTridiagonalCholeskySolver solver;
  solver.factorize(batchSize, numBlocks, blockDim, &diagonal, &offDiagonal);
  solver.solve(1, &rhs);

  const auto result = DownloadRhs(rhs, batchSize, numBlocks, blockDim, 1);
  for (int i = 0; i < batchSize; ++i) {
    Vector actual(numBlocks * blockDim);
    for (int t = 0; t < numBlocks; ++t)
      actual.segment(t * blockDim, blockDim) =
          result[static_cast<size_t>(i)][static_cast<size_t>(t)].col(0);
    EXPECT(assert_equal(expected[static_cast<size_t>(i)], actual, 1e-7));
  }
}

// Cross-checks against an independent Eigen dense solve of the same batch,
// assembled into dense n x n matrices, at a size distinct from the other
// Eigen-reference tests above (batchSize=2, numBlocks=5, blockDim=4).
TEST(BatchedBlockTridiagonalCholeskySolver, MatchesEigenDenseCholeskyOnAssembledSystem) {
  if (!HasCudaDevice()) return;
  const int batchSize = 2, numBlocks = 5, blockDim = 4;
  std::vector<BlockItem> batch;
  for (int i = 0; i < batchSize; ++i)
    batch.push_back(MakeRandomBlockItem(numBlocks, blockDim, 2000 + i));

  std::vector<Matrix> dense;
  for (const auto& item : batch) dense.push_back(AssembleDense(item));

  std::mt19937 rng(777);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  const int n = numBlocks * blockDim;
  std::vector<Vector> b(static_cast<size_t>(batchSize));
  std::vector<Vector> expected(static_cast<size_t>(batchSize));
  for (int i = 0; i < batchSize; ++i) {
    b[static_cast<size_t>(i)] = Vector(n);
    for (int r = 0; r < n; ++r) b[static_cast<size_t>(i)](r) = dist(rng);
    expected[static_cast<size_t>(i)] =
        dense[static_cast<size_t>(i)].ldlt().solve(b[static_cast<size_t>(i)]);
  }

  std::vector<std::vector<Matrix>> rhsBlocks(static_cast<size_t>(batchSize));
  for (int i = 0; i < batchSize; ++i)
    for (int t = 0; t < numBlocks; ++t)
      rhsBlocks[static_cast<size_t>(i)].push_back(
          b[static_cast<size_t>(i)].segment(t * blockDim, blockDim));
  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  DeviceArray<double> rhs = UploadRhs(rhsBlocks);
  BatchedBlockTridiagonalCholeskySolver blockSolver;
  blockSolver.factorize(batchSize, numBlocks, blockDim, &diagonal, &offDiagonal);
  blockSolver.solve(1, &rhs);
  const auto blockResult = DownloadRhs(rhs, batchSize, numBlocks, blockDim, 1);

  for (int i = 0; i < batchSize; ++i) {
    Vector actual(n);
    for (int t = 0; t < numBlocks; ++t)
      actual.segment(t * blockDim, blockDim) =
          blockResult[static_cast<size_t>(i)][static_cast<size_t>(t)].col(0);
    EXPECT(assert_equal(expected[static_cast<size_t>(i)], actual, 1e-9));
  }
}

// A diagonal block that is not SPD (here, exactly zero) must be rejected
// with a clear error.
TEST(BatchedBlockTridiagonalCholeskySolver, RejectsNonPositiveDefiniteBlock) {
  if (!HasCudaDevice()) return;
  BlockItem item;
  item.diagonal.push_back(Matrix::Zero(2, 2));
  item.diagonal.push_back(Matrix::Identity(2, 2) * 3.0);
  item.offDiagonal.push_back(Matrix::Identity(2, 2) * 0.1);
  std::vector<BlockItem> batch = {item};

  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  BatchedBlockTridiagonalCholeskySolver solver;
  CHECK_EXCEPTION(solver.factorize(1, 2, 2, &diagonal, &offDiagonal),
                 std::runtime_error);
}

// numBlocks <= 1 (no off-diagonal blocks possible) and mismatched array
// sizes must both be rejected before any CUDA call.
TEST(BatchedBlockTridiagonalCholeskySolver, RejectsMalformedBatchDimensions) {
  if (!HasCudaDevice()) return;
  const std::vector<BlockItem> batch = {MakeRandomBlockItem(3, 2, 42)};
  DeviceArray<double> diagonal = UploadDiagonal(batch);
  DeviceArray<double> offDiagonal = UploadOffDiagonal(batch);
  BatchedBlockTridiagonalCholeskySolver solver;
  CHECK_EXCEPTION(solver.factorize(1, 1, 2, &diagonal, &offDiagonal),
                 std::invalid_argument);
  CHECK_EXCEPTION(solver.factorize(2, 3, 2, &diagonal, &offDiagonal),
                 std::invalid_argument);
}

}  // namespace batched_block_tridiagonal_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
