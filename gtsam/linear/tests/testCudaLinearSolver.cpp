#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/cuda/CudaBlockOrdering.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>
#include <gtsam/linear/cuda/DeviceSparseSpdSystem.h>

#include <stdexcept>

using namespace gtsam::cuda;
using gtsam::Ordering;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
