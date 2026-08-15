#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/cuda/CudaBlockOrdering.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <stdexcept>

using namespace gtsam::cuda;
using gtsam::Ordering;
using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

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
