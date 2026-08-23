/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCudaLinearSolver.cpp
 * @brief   Unit tests for the shared CUDA linear-solver session
 * @author  Ruogu Li
 * @date    Aug 15, 2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/linear/cuda/internal/BlockOrdering.h>

#include <limits>
#include <map>
#include <stdexcept>
#include <optional>

using namespace gtsam::cuda;
using gtsam::KeyInfo;
using gtsam::Ordering;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

/* ************************************************************************* */
namespace linear_solver_fixture {

class TwoByTwoOperator final : public LinearOperator {
 public:
  int dimension() const override { return 2; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override {
    double hostInput[2];
    double hostOutput[2];
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(hostInput, input, sizeof(hostInput),
                                     cudaMemcpyDeviceToHost, stream));
    GTSAM_CUDA_CHECK(cudaStreamSynchronize(stream));
    hostOutput[0] = 4.0 * hostInput[0] + hostInput[1];
    hostOutput[1] = hostInput[0] + 3.0 * hostInput[1];
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(output, hostOutput, sizeof(hostOutput),
                                     cudaMemcpyHostToDevice, stream));
  }
};

class ZeroOperator final : public LinearOperator {
 public:
  int dimension() const override { return 2; }
  void apply(const double*, double* output,
             cudaStream_t stream) const override {
    GTSAM_CUDA_CHECK(cudaMemsetAsync(output, 0, 2 * sizeof(double), stream));
  }
};

class IdentityOperator final : public LinearOperator {
 public:
  int dimension() const override { return 2; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override {
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(output, input, 2 * sizeof(double),
                                     cudaMemcpyDeviceToDevice, stream));
  }
};

class IdentityPreconditioner final : public Preconditioner {
 public:
  int dimension() const override { return 2; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override {
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(output, input, 2 * sizeof(double),
                                     cudaMemcpyDeviceToDevice, stream));
  }
};

// Verifies LinearSolverSession::InvalidatesPcgWarmStartWhenOperatorChanges.
TEST(LinearSolverSession, InvalidatesPcgWarmStartWhenOperatorChanges) {
  LinearSolverOptions solverOptions;
  solverOptions.backend = LinearSolverType::Pcg;
  LinearSolverSession session(solverOptions);
  PcgOptions pcgOptions;
  pcgOptions.maxIterations = 2;
  pcgOptions.relativeTolerance = 1e-12;
  pcgOptions.convergenceCheckInterval = 2;
  session.analyze(2, pcgOptions);

  DeviceArray<double> rhs;
  DeviceArray<double> solution(2);
  rhs.upload({1.0, 2.0});
  solution.zero();
  session.solve(TwoByTwoOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);
  session.invalidateWarmStart();
  session.solve(TwoByTwoOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);
  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-12);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-12);
  LONGS_EQUAL(2, session.stats().solveCount);
  EXPECT(session.stats().lastPcgConverged);
}

// Verifies LinearSolverSession::ReportsPcgNumericalBreakdown.
TEST(LinearSolverSession, ReportsPcgNumericalBreakdown) {
  LinearSolverOptions solverOptions;
  solverOptions.backend = LinearSolverType::Pcg;
  LinearSolverSession session(solverOptions);
  PcgOptions pcgOptions;
  pcgOptions.maxIterations = 2;
  pcgOptions.relativeTolerance = 1e-12;
  pcgOptions.convergenceCheckInterval = 1;
  session.analyze(2, pcgOptions);

  DeviceArray<double> rhs;
  DeviceArray<double> solution(2);
  rhs.upload({1.0, 2.0});
  solution.zero();
  session.solve(ZeroOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);

  CHECK(!session.stats().lastPcgConverged);
  CHECK(session.stats().lastPcgBreakdown);
  EXPECT_LONGS_EQUAL(1, session.stats().pcgBreakdownCount);
  EXPECT_LONGS_EQUAL(0, session.stats().pcgMaxIterationHits);
}

// Verifies LinearSolverSession::RecognizesConvergenceBeforeDelayedCheck.
TEST(LinearSolverSession, RecognizesConvergenceBeforeDelayedCheck) {
  LinearSolverOptions solverOptions;
  solverOptions.backend = LinearSolverType::Pcg;
  LinearSolverSession session(solverOptions);
  PcgOptions pcgOptions;
  pcgOptions.maxIterations = 10;
  pcgOptions.relativeTolerance = 1e-12;
  pcgOptions.convergenceCheckInterval = 10;
  session.analyze(2, pcgOptions);

  DeviceArray<double> rhs;
  DeviceArray<double> solution(2);
  rhs.upload({1.0, 2.0});
  solution.zero();
  session.solve(IdentityOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);

  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  EXPECT(session.stats().lastPcgConverged);
  CHECK(!session.stats().lastPcgBreakdown);
  EXPECT_LONGS_EQUAL(0, session.stats().pcgBreakdownCount);
  DOUBLES_EQUAL(1.0, actual[0], 1e-12);
  DOUBLES_EQUAL(2.0, actual[1], 1e-12);
}

// Verifies DeviceSparseSpdSystem::RestoresUndampedDiagonalBetweenAttempts.
TEST(DeviceSparseSpdSystem, RestoresUndampedDiagonalBetweenAttempts) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(3, {0, 2, 4, 5}, {0, 1, 1, 2, 2});
  system.values().upload({4.0, 1.0, 5.0, 2.0, 6.0});
  system.captureUndampedDiagonal();

  system.restoreAndAddDiagonal(2.0);
  system.restoreAndAddDiagonal(5.0);

  std::vector<double> actual;
  system.values().download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  const std::vector<double> expected{9.0, 1.0, 10.0, 2.0, 11.0};
  EXPECT(expected == actual);

  const SparseSpdSystemView view = system.view();
  LONGS_EQUAL(3, view.dimension);
  LONGS_EQUAL(5, view.nonzeros);
  EXPECT(view.triangle == SparseTriangle::Upper);
}

// Verifies DeviceSparseSpdSystem::RejectsMissingDiagonalOnCapture.
TEST(DeviceSparseSpdSystem, RejectsMissingDiagonalOnCapture) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, {0, 1, 2}, {1, 1});
  system.values().upload({1.0, 2.0});
  CHECK_EXCEPTION(system.captureUndampedDiagonal(), std::runtime_error);
}

// Verifies BlockOrdering::ExpandsKeysToScalars.
TEST(BlockOrdering, ExpandsKeysToScalars) {
  const KeyInfo layout(
      std::map<gtsam::Key, size_t>{{X(1), 2}, {L(4), 3}, {X(7), 1}},
      Ordering{X(1), L(4), X(7)});
  const std::vector<int> expected{2, 3, 4, 0, 1, 5};
  EXPECT(expected ==
         compileScalarPermutation(layout, Ordering{L(4), X(1), X(7)}));
  EXPECT((std::vector<int>{0, 2, 5, 6} == cudaBlockOffsets(layout)));
}

// Verifies BlockOrdering::RejectsInvalidOrderings.
TEST(BlockOrdering, RejectsInvalidOrderings) {
  const KeyInfo layout(
      std::map<gtsam::Key, size_t>{{X(1), 2}, {X(2), 2}});
  CHECK_EXCEPTION(compileScalarPermutation(layout, Ordering{X(1)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      compileScalarPermutation(layout, Ordering{X(1), X(1)}),
      std::invalid_argument);
  CHECK_EXCEPTION(
      compileScalarPermutation(layout, Ordering{X(1), X(3)}),
      std::invalid_argument);
}

// Verifies BlockOrdering::RejectsInvalidLayouts.
TEST(BlockOrdering, RejectsInvalidLayouts) {
  EXPECT((std::vector<int>{0} == cudaBlockOffsets(KeyInfo{})));

  KeyInfo noncontiguous(
      std::map<gtsam::Key, size_t>{{X(1), 2}, {X(2), 1}});
  noncontiguous.at(X(2)).start = 3;
  CHECK_EXCEPTION(cudaBlockOffsets(noncontiguous), std::invalid_argument);

  const KeyInfo zeroDimension(
      std::map<gtsam::Key, size_t>{{X(1), 0}});
  CHECK_EXCEPTION(cudaBlockOffsets(zeroDimension), std::invalid_argument);

  const KeyInfo intOverflow(std::map<gtsam::Key, size_t>{
      {X(1), static_cast<size_t>(std::numeric_limits<int>::max()) + 1}});
  CHECK_EXCEPTION(cudaBlockOffsets(intOverflow), std::invalid_argument);

  KeyInfo invalidIndex(std::map<gtsam::Key, size_t>{{X(1), 1}});
  invalidIndex.at(X(1)).index = 1;
  CHECK_EXCEPTION(cudaBlockOffsets(invalidIndex), std::invalid_argument);

  KeyInfo invalidCardinality(
      std::map<gtsam::Key, size_t>{{X(1), 1}, {X(2), 1}});
  invalidCardinality.erase(X(2));
  CHECK_EXCEPTION(cudaBlockOffsets(invalidCardinality), std::invalid_argument);

  KeyInfo staleTotal(std::map<gtsam::Key, size_t>{{X(1), 2}});
  staleTotal.at(X(1)).dim = 1;
  CHECK_EXCEPTION(cudaBlockOffsets(staleTotal), std::invalid_argument);

  CHECK_EXCEPTION(compileScalarPermutation(zeroDimension, Ordering{X(1)}),
                  std::invalid_argument);
}

// Verifies LinearSolver::CapabilityMatrix.
TEST(LinearSolver, CapabilityMatrix) {
  EXPECT(LinearSolverSession::supports(
      LinearSolverType::DenseCholesky, LinearSystemKind::Dense));
  EXPECT(LinearSolverSession::supports(LinearSolverType::Cudss,
                                           LinearSystemKind::Sparse));
  EXPECT(LinearSolverSession::supports(LinearSolverType::Pcg,
                                           LinearSystemKind::Operator));

  EXPECT(!LinearSolverSession::supports(
      LinearSolverType::DenseCholesky, LinearSystemKind::Sparse));
  EXPECT(!LinearSolverSession::supports(LinearSolverType::Cudss,
                                           LinearSystemKind::Dense));
  EXPECT(!LinearSolverSession::supports(LinearSolverType::Pcg,
                                           LinearSystemKind::Sparse));
}

// Verifies LinearSolver::RejectsRepresentationMismatch.
TEST(LinearSolver, RejectsRepresentationMismatch) {
  LinearSolverOptions options;
  options.backend = LinearSolverType::DenseCholesky;
  CHECK_EXCEPTION(LinearSolverSession::validate(
                      options, LinearSystemKind::Sparse),
                  std::invalid_argument);
}

// Verifies LinearSolverSession::DispatchesDensePreparedSystem.
TEST(LinearSolverSession, DispatchesDensePreparedSystem) {
  LinearSolverOptions options;
  options.backend = LinearSolverType::DenseCholesky;
  LinearSolverSession session(options);
  session.analyze(2);

  DeviceArray<double> matrix;
  DeviceArray<double> rhs;
  DeviceArray<double> solution;
  matrix.upload({4.0, 1.0, 1.0, 3.0});
  rhs.upload({1.0, 2.0});
  session.solve(DenseSpdSystemView{2, 2, matrix.data(), rhs.data()},
                &solution);

  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-12);
  LONGS_EQUAL(1, session.stats().analysisCount);
  LONGS_EQUAL(1, session.stats().solveCount);
}

#if GTSAM_ENABLE_CUDSS
// Verifies LinearSolverSession::DispatchesOrderedSparsePreparedSystem.
TEST(LinearSolverSession, DispatchesOrderedSparsePreparedSystem) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, {0, 2, 3}, {0, 1, 1});
  system.values().upload({2.0, 0.5, 2.0});
  system.rhs().upload({1.0, 1.0});
  DeviceArray<double> solution;
  LinearSolverOptions options;
  options.backend = LinearSolverType::Cudss;
  LinearSolverSession session(options);
  session.analyze(system, &solution, std::vector<int>{1, 0});
  session.solve(system, &solution);
  system.values().upload({3.0, 0.25, 2.5});
  system.rhs().upload({2.0, -1.0});
  session.solve(system, &solution);
  EXPECT(session.stats().userOrderingApplied);
  LONGS_EQUAL(1, session.stats().analysisCount);
  LONGS_EQUAL(2, session.stats().factorizationCount);
  LONGS_EQUAL(2, session.stats().solveCount);
}
#endif

}  // namespace linear_solver_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
