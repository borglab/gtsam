#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/cuda/CudaBlockOrdering.h>
#include <gtsam/linear/cuda/CudaDenseCholeskySolver.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>
#include <gtsam/linear/cuda/CudaPcgSolver.h>
#include <gtsam/linear/cuda/CudssSpdSolver.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <stdexcept>
#include <optional>

using namespace gtsam::cuda;
using gtsam::Ordering;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

namespace {

class TwoByTwoOperator final : public CudaLinearOperator {
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

class IdentityPreconditioner final : public CudaPreconditioner {
 public:
  int dimension() const override { return 2; }
  void apply(const double* input, double* output,
             cudaStream_t stream) const override {
    GTSAM_CUDA_CHECK(cudaMemcpyAsync(output, input, 2 * sizeof(double),
                                     cudaMemcpyDeviceToDevice, stream));
  }
};

}  // namespace

TEST(CudaPcgSolver, SolvesThroughGenericOperatorInterface) {
  CudaPcgOptions options;
  options.maxIterations = 2;
  options.relativeTolerance = 1e-12;
  options.convergenceCheckInterval = 2;
  options.warmStart = false;
  CudaPcgSolver solver;
  solver.initialize(2, options);

  CudaDeviceArray<double> rhs;
  CudaDeviceArray<double> solution(2);
  rhs.upload({1.0, 2.0});
  solution.zero();
  solver.solve(TwoByTwoOperator{}, IdentityPreconditioner{}, rhs.data(),
               &solution);

  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-12);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-12);
  EXPECT(solver.stats().lastPcgConverged);
  LONGS_EQUAL(2, solver.stats().pcgIterationsTotal);
  LONGS_EQUAL(3, solver.stats().pcgHostConvergenceChecks);
}

TEST(CudaLinearSolverSession, InvalidatesPcgWarmStartWhenOperatorChanges) {
  CudaLinearSolverOptions solverOptions;
  solverOptions.backend = CudaLinearSolverType::Pcg;
  CudaLinearSolverSession session(solverOptions);
  CudaPcgOptions pcgOptions;
  pcgOptions.maxIterations = 2;
  pcgOptions.relativeTolerance = 1e-12;
  pcgOptions.convergenceCheckInterval = 2;
  session.analyze(2, pcgOptions);

  CudaDeviceArray<double> rhs;
  CudaDeviceArray<double> solution(2);
  rhs.upload({1.0, 2.0});
  solution.zero();
  session.solve(TwoByTwoOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);
  session.invalidateWarmStart();
  session.solve(TwoByTwoOperator{}, IdentityPreconditioner{}, rhs.data(),
                &solution);
  LONGS_EQUAL(2, session.stats().solveCount);
  EXPECT(session.stats().lastPcgConverged);
}

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

  const CudaSparseSpdSystemView view = system.view();
  LONGS_EQUAL(3, view.dimension);
  LONGS_EQUAL(5, view.nonzeros);
  EXPECT(view.triangle == CudaSparseTriangle::Upper);
}

TEST(DeviceSparseSpdSystem, RejectsMissingDiagonalOnCapture) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, {0, 1, 2}, {1, 1});
  system.values().upload({1.0, 2.0});
  CHECK_EXCEPTION(system.captureUndampedDiagonal(), std::runtime_error);
}

TEST(CudaDenseCholeskySolver, SolvesTwoByTwoSpdSystem) {
  CudaDeviceArray<double> matrix;
  CudaDeviceArray<double> rhs;
  CudaDeviceArray<double> solution;
  matrix.upload({4.0, 1.0, 1.0, 3.0});
  rhs.upload({1.0, 2.0});

  CudaDenseCholeskySolver solver;
  solver.solve({2, 2, matrix.data(), rhs.data()}, &solution);

  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  LONGS_EQUAL(2, actual.size());
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-12);
  DOUBLES_EQUAL(7.0 / 11.0, actual[1], 1e-12);
}

TEST(CudssSpdSolver, AppliesRequestedPermutationWithoutChangingSolution) {
  DeviceSparseSpdSystem automaticSystem;
  automaticSystem.uploadPattern(4, {0, 2, 4, 6, 7},
                                {0, 1, 1, 2, 2, 3, 3});
  automaticSystem.values().upload({4.0, 1.0, 3.0, 1.0, 3.0, 1.0, 2.0});
  automaticSystem.rhs().upload({1.0, 2.0, 3.0, 4.0});
  DeviceSparseSpdSystem orderedSystem;
  orderedSystem.uploadPattern(4, {0, 2, 4, 6, 7},
                              {0, 1, 1, 2, 2, 3, 3});
  orderedSystem.values().upload({4.0, 1.0, 3.0, 1.0, 3.0, 1.0, 2.0});
  orderedSystem.rhs().upload({1.0, 2.0, 3.0, 4.0});

  CudaDeviceArray<double> automaticSolution;
  CudssSpdSolver automaticSolver;
  automaticSolver.analyze(automaticSystem, &automaticSolution);
  automaticSolver.solve(automaticSystem, &automaticSolution);

  CudaDeviceArray<double> orderedSolution;
  CudssSpdSolver orderedSolver;
  orderedSolver.analyze(orderedSystem, &orderedSolution,
                        std::vector<int>{2, 3, 0, 1});
  orderedSolver.solve(orderedSystem, &orderedSolution);

  std::vector<double> automatic;
  std::vector<double> ordered;
  automaticSolution.download(&automatic);
  orderedSolution.download(&ordered);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  LONGS_EQUAL(automatic.size(), ordered.size());
  for (size_t i = 0; i < automatic.size(); ++i) {
    DOUBLES_EQUAL(automatic[i], ordered[i], 1e-12);
  }
  EXPECT(!automaticSolver.stats().userOrderingApplied);
  EXPECT(orderedSolver.stats().userOrderingApplied);
  EXPECT(std::vector<int>({2, 3, 0, 1}) ==
         orderedSolver.appliedPermutation());
  LONGS_EQUAL(1, orderedSolver.stats().analysisCount);
  LONGS_EQUAL(1, orderedSolver.stats().solveCount);
}

TEST(CudssSpdSolver, RejectsInvalidPermutation) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, {0, 2, 3}, {0, 1, 1});
  system.values().upload({2.0, 0.5, 2.0});
  system.rhs().upload({1.0, 1.0});
  CudaDeviceArray<double> solution;
  CudssSpdSolver solver;
  CHECK_EXCEPTION(solver.analyze(system, &solution, std::vector<int>{0}),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      solver.analyze(system, &solution, std::vector<int>{0, 0}),
      std::invalid_argument);
  CHECK_EXCEPTION(
      solver.analyze(system, &solution, std::vector<int>{0, 2}),
      std::invalid_argument);
}

TEST(CudaBlockOrdering, ExpandsKeysToScalars) {
  const CudaBlockLayout layout{{X(1), 0, 2},
                               {L(4), 2, 3},
                               {X(7), 5, 1}};
  const std::vector<int> expected{2, 3, 4, 0, 1, 5};
  EXPECT(expected ==
         CompileCudaScalarPermutation(layout, Ordering{L(4), X(1), X(7)}));
}

TEST(CudaBlockOrdering, RejectsInvalidOrderings) {
  const CudaBlockLayout layout{{X(1), 0, 2}, {X(2), 2, 2}};
  CHECK_EXCEPTION(CompileCudaScalarPermutation(layout, Ordering{X(1)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      CompileCudaScalarPermutation(layout, Ordering{X(1), X(1)}),
      std::invalid_argument);
  CHECK_EXCEPTION(
      CompileCudaScalarPermutation(layout, Ordering{X(1), X(3)}),
      std::invalid_argument);
}

TEST(CudaBlockOrdering, RejectsInvalidLayouts) {
  CHECK_EXCEPTION(CompileCudaScalarPermutation(
                      {{X(1), 0, 2}, {X(2), 3, 1}},
                      Ordering{X(1), X(2)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(CompileCudaScalarPermutation(
                      {{X(1), 0, 2}, {X(1), 2, 1}},
                      Ordering{X(1), X(1)}),
                  std::invalid_argument);
  CHECK_EXCEPTION(CompileCudaScalarPermutation(
                      {{X(1), 0, 0}}, Ordering{X(1)}),
                  std::invalid_argument);
}

TEST(CudaLinearSolver, CapabilityMatrix) {
  EXPECT(CudaLinearSolverSession::Supports(
      CudaLinearSolverType::DenseCholesky, CudaLinearSystemKind::Dense));
  EXPECT(CudaLinearSolverSession::Supports(CudaLinearSolverType::Cudss,
                                           CudaLinearSystemKind::Sparse));
  EXPECT(CudaLinearSolverSession::Supports(CudaLinearSolverType::Pcg,
                                           CudaLinearSystemKind::Operator));

  EXPECT(!CudaLinearSolverSession::Supports(
      CudaLinearSolverType::DenseCholesky, CudaLinearSystemKind::Sparse));
  EXPECT(!CudaLinearSolverSession::Supports(CudaLinearSolverType::Cudss,
                                           CudaLinearSystemKind::Dense));
  EXPECT(!CudaLinearSolverSession::Supports(CudaLinearSolverType::Pcg,
                                           CudaLinearSystemKind::Sparse));
}

TEST(CudaLinearSolver, RejectsOrderingForNonCudssBackend) {
  CudaLinearSolverOptions options;
  options.backend = CudaLinearSolverType::Pcg;
  options.useUserOrdering = true;
  CHECK_EXCEPTION(CudaLinearSolverSession::Validate(
                      options, CudaLinearSystemKind::Operator),
                  std::invalid_argument);
}

TEST(CudaLinearSolver, RejectsRepresentationMismatch) {
  CudaLinearSolverOptions options;
  options.backend = CudaLinearSolverType::DenseCholesky;
  CHECK_EXCEPTION(CudaLinearSolverSession::Validate(
                      options, CudaLinearSystemKind::Sparse),
                  std::invalid_argument);
}

TEST(CudaLinearSolverSession, DispatchesDensePreparedSystem) {
  CudaLinearSolverOptions options;
  options.backend = CudaLinearSolverType::DenseCholesky;
  CudaLinearSolverSession session(options);
  session.analyze(2);

  CudaDeviceArray<double> matrix;
  CudaDeviceArray<double> rhs;
  CudaDeviceArray<double> solution;
  matrix.upload({4.0, 1.0, 1.0, 3.0});
  rhs.upload({1.0, 2.0});
  session.solve(CudaDenseSpdSystemView{2, 2, matrix.data(), rhs.data()},
                &solution);

  std::vector<double> actual;
  solution.download(&actual);
  GTSAM_CUDA_CHECK(cudaDeviceSynchronize());
  DOUBLES_EQUAL(1.0 / 11.0, actual[0], 1e-12);
  LONGS_EQUAL(1, session.stats().analysisCount);
  LONGS_EQUAL(1, session.stats().solveCount);
}

TEST(CudaLinearSolverSession, DispatchesOrderedSparsePreparedSystem) {
  DeviceSparseSpdSystem system;
  system.uploadPattern(2, {0, 2, 3}, {0, 1, 1});
  system.values().upload({2.0, 0.5, 2.0});
  system.rhs().upload({1.0, 1.0});
  CudaDeviceArray<double> solution;
  CudaLinearSolverOptions options;
  options.backend = CudaLinearSolverType::Cudss;
  options.useUserOrdering = true;
  CudaLinearSolverSession session(options);
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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
