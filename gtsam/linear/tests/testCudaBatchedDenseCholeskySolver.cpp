/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaBatchedDenseCholeskySolver.cpp
 * @brief   Unit tests for the batched dense cuSOLVER Cholesky backend
 * @author  Shaji
 * @date    Nov 20, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <cuda_runtime_api.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>

#include <Eigen/Cholesky>
#include <stdexcept>
#include <vector>

using namespace gtsam;
using namespace gtsam::cuda;

/* ************************************************************************* */
namespace batched_dense_cholesky_fixture {

bool HasCudaDevice() {
  int count = 0;
  const cudaError_t status = cudaGetDeviceCount(&count);
  if (status != cudaSuccess) {
    // Do not leave a discovery error pending for a subsequent test.
    (void)cudaGetLastError();
    return false;
  }
  return count > 0;
}

// Three fixed, hand-built 3x3 SPD matrices (symmetric and diagonally
// dominant, so SPD by construction) and matching right-hand sides. Not
// randomized, so test failures are exactly reproducible.
std::vector<Matrix> MakeSpdBatch() {
  Matrix H0(3, 3);
  H0 << 4.0, 0.3, 0.1, 0.3, 3.0, 0.2, 0.1, 0.2, 2.5;
  Matrix H1(3, 3);
  H1 << 5.0, -0.4, 0.2, -0.4, 2.0, 0.1, 0.2, 0.1, 3.5;
  Matrix H2(3, 3);
  H2 << 3.0, 0.1, -0.2, 0.1, 4.0, 0.3, -0.2, 0.3, 2.0;
  return {H0, H1, H2};
}

std::vector<Vector> MakeRhsBatch() {
  Vector b0(3);
  b0 << 1.0, 2.0, 3.0;
  Vector b1(3);
  b1 << -1.0, 0.5, 2.5;
  Vector b2(3);
  b2 << 0.0, 1.0, -2.0;
  return {b0, b1, b2};
}

// Uploads a batch of column-major dense matrices into one contiguous
// DeviceArray, in the layout BatchedDenseCholeskySolver expects.
DeviceArray<double> UploadBatch(const std::vector<Matrix>& items) {
  const int batchSize = static_cast<int>(items.size());
  const int dimension = static_cast<int>(items.front().rows());
  std::vector<double> flat(static_cast<size_t>(batchSize) *
                           static_cast<size_t>(dimension) *
                           static_cast<size_t>(dimension));
  for (int b = 0; b < batchSize; ++b) {
    const Matrix& item = items[static_cast<size_t>(b)];
    const size_t offset = static_cast<size_t>(b) *
                         static_cast<size_t>(dimension) *
                         static_cast<size_t>(dimension);
    std::copy(item.data(), item.data() + item.size(), flat.begin() + offset);
  }
  DeviceArray<double> device;
  device.upload(flat);
  return device;
}

DeviceArray<double> UploadRhsBatch(const std::vector<Vector>& items, int k,
                                  int column) {
  // Builds one column's worth of the (batchSize, dimension, k) layout
  // solve() expects, with only `column` populated and the rest zero --
  // sufficient for the single-column tests below.
  const int batchSize = static_cast<int>(items.size());
  const int dimension = static_cast<int>(items.front().size());
  std::vector<double> flat(static_cast<size_t>(batchSize) *
                           static_cast<size_t>(dimension) *
                           static_cast<size_t>(k),
                           0.0);
  for (int b = 0; b < batchSize; ++b) {
    const Vector& item = items[static_cast<size_t>(b)];
    const size_t offset = static_cast<size_t>(b) * dimension * k +
                         static_cast<size_t>(column) * dimension;
    std::copy(item.data(), item.data() + item.size(), flat.begin() + offset);
  }
  DeviceArray<double> device;
  device.upload(flat);
  return device;
}

std::vector<Vector> DownloadSolution(DeviceArray<double>* device,
                                     int batchSize, int dimension, int k,
                                     int column) {
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));
  std::vector<double> host;
  device->download(&host);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));
  std::vector<Vector> result(static_cast<size_t>(batchSize));
  for (int b = 0; b < batchSize; ++b) {
    Vector v(dimension);
    const size_t offset = static_cast<size_t>(b) * dimension * k +
                         static_cast<size_t>(column) * dimension;
    std::copy(host.begin() + offset, host.begin() + offset + dimension,
             v.data());
    result[static_cast<size_t>(b)] = v;
  }
  return result;
}

}  // namespace batched_dense_cholesky_fixture
/* ************************************************************************* */

using namespace batched_dense_cholesky_fixture;

// Verifies BatchedDenseCholeskySolver::FactorizeAndSolveMatchesIndependentSolves.
TEST(BatchedDenseCholeskySolver, FactorizeAndSolveMatchesIndependentSolves) {
  if (!HasCudaDevice()) return;
  const std::vector<Matrix> H = MakeSpdBatch();
  const std::vector<Vector> rhs = MakeRhsBatch();
  const int batchSize = static_cast<int>(H.size());
  const int dimension = static_cast<int>(H.front().rows());

  DeviceArray<double> deviceH = UploadBatch(H);
  BatchedDenseCholeskySolver solver;
  solver.factorize(batchSize, dimension, &deviceH);
  DeviceArray<double> deviceRhs = UploadRhsBatch(rhs, /*k=*/1, /*column=*/0);
  solver.solve(1, &deviceRhs);
  const std::vector<Vector> actual =
      DownloadSolution(&deviceRhs, batchSize, dimension, 1, 0);

  for (int b = 0; b < batchSize; ++b) {
    const Vector expected =
        H[static_cast<size_t>(b)].ldlt().solve(rhs[static_cast<size_t>(b)]);
    EXPECT(assert_equal(expected, actual[static_cast<size_t>(b)], 1e-9));
  }
  LONGS_EQUAL(batchSize, solver.batchSize());
  LONGS_EQUAL(dimension, solver.dimension());
}

// Verifies BatchedDenseCholeskySolver::SolveReusesFactorizationAcrossCalls --
// the "factorize once, solve many RHS" pattern BatchedDeviceGradients relies
// on for its differentiable backward.
TEST(BatchedDenseCholeskySolver, SolveReusesFactorizationAcrossCalls) {
  if (!HasCudaDevice()) return;
  const std::vector<Matrix> H = MakeSpdBatch();
  const std::vector<Vector> rhsFirst = MakeRhsBatch();
  std::vector<Vector> rhsSecond = rhsFirst;
  for (auto& v : rhsSecond) v = v.reverse();  // a distinct second RHS batch
  const int batchSize = static_cast<int>(H.size());
  const int dimension = static_cast<int>(H.front().rows());

  DeviceArray<double> deviceH = UploadBatch(H);
  BatchedDenseCholeskySolver solver;
  solver.factorize(batchSize, dimension, &deviceH);

  DeviceArray<double> deviceFirst =
      UploadRhsBatch(rhsFirst, /*k=*/1, /*column=*/0);
  solver.solve(1, &deviceFirst);
  const std::vector<Vector> actualFirst =
      DownloadSolution(&deviceFirst, batchSize, dimension, 1, 0);

  DeviceArray<double> deviceSecond =
      UploadRhsBatch(rhsSecond, /*k=*/1, /*column=*/0);
  solver.solve(1, &deviceSecond);  // no refactorize() in between
  const std::vector<Vector> actualSecond =
      DownloadSolution(&deviceSecond, batchSize, dimension, 1, 0);

  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    EXPECT(assert_equal(H[i].ldlt().solve(rhsFirst[i]), actualFirst[i], 1e-9));
    EXPECT(
        assert_equal(H[i].ldlt().solve(rhsSecond[i]), actualSecond[i], 1e-9));
  }
}

// The nrhs=1-per-call loop inside solve(k>1, ...) must match k
// independent single-column solves.
TEST(BatchedDenseCholeskySolver, MultiColumnSolveMatchesPerColumnReference) {
  if (!HasCudaDevice()) return;
  const std::vector<Matrix> H = MakeSpdBatch();
  const int batchSize = static_cast<int>(H.size());
  const int dimension = static_cast<int>(H.front().rows());
  constexpr int k = 2;

  std::vector<Vector> rhsColumn0 = MakeRhsBatch();
  std::vector<Vector> rhsColumn1 = rhsColumn0;
  for (auto& v : rhsColumn1) v = v.reverse();

  std::vector<double> flatRhs(static_cast<size_t>(batchSize) *
                             static_cast<size_t>(dimension) * k);
  for (int b = 0; b < batchSize; ++b) {
    const size_t base = static_cast<size_t>(b) * dimension * k;
    const Vector& c0 = rhsColumn0[static_cast<size_t>(b)];
    const Vector& c1 = rhsColumn1[static_cast<size_t>(b)];
    std::copy(c0.data(), c0.data() + dimension, flatRhs.begin() + base);
    std::copy(c1.data(), c1.data() + dimension,
             flatRhs.begin() + base + dimension);
  }

  DeviceArray<double> deviceH = UploadBatch(H);
  BatchedDenseCholeskySolver solver;
  solver.factorize(batchSize, dimension, &deviceH);
  DeviceArray<double> deviceRhs;
  deviceRhs.upload(flatRhs);
  solver.solve(k, &deviceRhs);

  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));
  std::vector<double> host;
  deviceRhs.download(&host);
  GTSAM_CUDA_CHECK(cudaStreamSynchronize(nullptr));

  for (int b = 0; b < batchSize; ++b) {
    const size_t i = static_cast<size_t>(b);
    const size_t base = static_cast<size_t>(b) * dimension * k;
    Vector actualColumn0(dimension), actualColumn1(dimension);
    std::copy(host.begin() + base, host.begin() + base + dimension,
             actualColumn0.data());
    std::copy(host.begin() + base + dimension,
             host.begin() + base + 2 * dimension, actualColumn1.data());
    EXPECT(assert_equal(H[i].ldlt().solve(rhsColumn0[i]), actualColumn0, 1e-9));
    EXPECT(assert_equal(H[i].ldlt().solve(rhsColumn1[i]), actualColumn1, 1e-9));
  }
}

// Verifies BatchedDenseCholeskySolver::FactorizeThrowsForNonPositiveDefinite.
TEST(BatchedDenseCholeskySolver, FactorizeThrowsForNonPositiveDefinite) {
  if (!HasCudaDevice()) return;
  Matrix indefinite(2, 2);
  indefinite << 1.0, 2.0, 2.0, 1.0;  // eigenvalues 3 and -1: not SPD
  DeviceArray<double> deviceH = UploadBatch({indefinite});
  BatchedDenseCholeskySolver solver;
  CHECK_EXCEPTION(solver.factorize(1, 2, &deviceH), std::runtime_error);
}

// Verifies BatchedDenseCholeskySolver::FactorizeThrowsForMismatchedBufferSize.
TEST(BatchedDenseCholeskySolver, FactorizeThrowsForMismatchedBufferSize) {
  if (!HasCudaDevice()) return;
  DeviceArray<double> tooSmall;
  tooSmall.upload({1.0, 2.0, 3.0});  // 3 elements, claiming batchSize=1, dim=2 (needs 4)
  BatchedDenseCholeskySolver solver;
  CHECK_EXCEPTION(solver.factorize(1, 2, &tooSmall), std::invalid_argument);
}

// Verifies BatchedDenseCholeskySolver::SolveThrowsBeforeFactorize.
TEST(BatchedDenseCholeskySolver, SolveThrowsBeforeFactorize) {
  if (!HasCudaDevice()) return;
  DeviceArray<double> rhs;
  rhs.upload({1.0, 2.0});
  BatchedDenseCholeskySolver solver;
  CHECK_EXCEPTION(solver.solve(1, &rhs), std::logic_error);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
