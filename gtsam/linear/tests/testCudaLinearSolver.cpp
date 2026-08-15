#include <CppUnitLite/TestHarness.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>

#include <stdexcept>

using namespace gtsam::cuda;

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
