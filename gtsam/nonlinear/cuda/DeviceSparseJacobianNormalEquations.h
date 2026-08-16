#pragma once

#include <cuda_runtime_api.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>
#include <gtsam/linear/cuda/CudaLinearSolver.h>
#include <gtsam/nonlinear/cuda/DevicePcgSolver.h>
#include <gtsam/nonlinear/cuda/DeviceSparseNormalEquations.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

struct DeviceSparseNormalEquationCapability {
  bool supported = false;
  std::string detail;
};

enum class DeviceNormalSolverBackend { Cudss, Pcg };

/**
 * Linear-solver selection for the persistent device pipeline. In Pcg mode
 * the normal matrix H is never formed: SpGEMM pattern discovery, stable-H
 * storage, and cuDSS are all skipped, and columnBlockOffsets must hold the
 * variable-block boundaries (numBlocks+1 ascending scalar-column offsets
 * ending at the plan's column count) for the block-Jacobi preconditioner.
 */
struct DeviceNormalSolverOptions {
  DeviceNormalSolverBackend backend = DeviceNormalSolverBackend::Cudss;
  DevicePcgOptions pcg;
  std::vector<int> columnBlockOffsets;
  std::vector<int> scalarPermutation;
};

struct LinearizedModelErrors {
  double oldError = 0.0;
  double newError = 0.0;

  double change() const { return oldError - newError; }
};

struct DeviceSparseJacobianAttemptResult {
  Vector delta;
  LinearizedModelErrors model;
  int pcgIterations = 0;
  bool pcgConverged = true;
};

/**
 * Cumulative device-stage profile for one persistent sparse-Jacobian system.
 *
 * CUDA durations are measured with events on the fixed stream. The cuDSS
 * DATA_INFO field is host wall time for the mandatory status boundary and
 * overlaps the factor-and-solve stage, so timing fields are not intended to
 * be summed into an exclusive total. Pending iteration/attempt event spans
 * are harvested by downloadAttemptResult() after its existing stream
 * synchronization. Transfer counters report logical bytes even when timing
 * collection is disabled.
 */
struct DeviceSparseJacobianProfile {
  // One-time setup.
  double initializeWall = 0.0;
  double patternH2d = 0.0;
  double structureSetup = 0.0;
  double setupD2h = 0.0;
  // Structural analysis is cached, so this records only the effective first
  // cuDSS analysis rather than every analyze() call.
  double cudssAnalysis = 0.0;

  // Per outer linearization.
  double numericH2d = 0.0;
  double transposeUpdate = 0.0;
  double normalJtJ = 0.0;
  double normalJtb = 0.0;
  double diagonalExtraction = 0.0;
  double oldModelError = 0.0;
  double dampingPreparation = 0.0;

  // Per lambda attempt.
  double dampingApplication = 0.0;
  double cudssFactorAndSolve = 0.0;
  double cudssDataInfoBoundaryWall = 0.0;
  double newModelError = 0.0;
  double attemptD2h = 0.0;
  double attemptHostBuild = 0.0;

  // PCG backend only; zero in cuDSS mode. Wall times measured on the host
  // around the internally synchronizing solve.
  double pcgPreconditionerBuild = 0.0;
  double pcgSolve = 0.0;
  size_t pcgIterationsTotal = 0;
  size_t pcgSolveCount = 0;
  size_t pcgMaxIterationHits = 0;

  size_t patternH2dBytes = 0;
  size_t numericH2dBytes = 0;
  size_t setupD2hBytes = 0;
  size_t attemptD2hBytes = 0;

  size_t totalH2dBytes() const { return patternH2dBytes + numericH2dBytes; }
  size_t totalD2hBytes() const { return setupD2hBytes + attemptD2hBytes; }
};

class GTSAM_EXPORT DeviceSparseJacobianNormalEquations {
 public:
  DeviceSparseJacobianNormalEquations();
  ~DeviceSparseJacobianNormalEquations();

  DeviceSparseJacobianNormalEquations(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations& operator=(
      const DeviceSparseJacobianNormalEquations&) = delete;
  DeviceSparseJacobianNormalEquations(
      DeviceSparseJacobianNormalEquations&&) noexcept;
  DeviceSparseJacobianNormalEquations& operator=(
      DeviceSparseJacobianNormalEquations&&) noexcept;

  static DeviceSparseNormalEquationCapability preflightCapability();
  static DeviceSparseNormalEquationCapability preflightCapability(
      DeviceNormalSolverBackend backend);

  // The borrowed fixed stream must outlive this object; destruction waits for
  // it before releasing descriptors, workspaces, and device allocations.
  void initialize(const SparseJacobianPlan& plan, cudaStream_t stream = nullptr,
                  bool collectProfile = false,
                  const DeviceNormalSolverOptions& solverOptions = {});
  // This upload is asynchronous. The pinned host storage must remain alive
  // and unmodified until the fixed stream reaches the queued copies.
  void uploadNumerics(const HostSparseJacobian& host,
                      cudaStream_t stream = nullptr);
  void formUndampedSystem(cudaStream_t stream = nullptr);

  // Every stream-taking operation after initialize() must receive the fixed
  // stream. Consumers of system() must use that stream, or establish ordering
  // with an event, before reading or modifying its storage.
  // formUndampedSystem(), prepareDamping(), analyze(), and solveAndEvaluate()
  // are asynchronous except for cuDSS's required numerical-factorization
  // status boundary.
  // downloadAttemptResult() performs the one synchronization needed to return
  // host-owned values.
  void prepareDamping(bool diagonalDamping, double minDiagonal,
                      double maxDiagonal, cudaStream_t stream = nullptr);
  void analyze(cudaStream_t stream = nullptr);
  void solveAndEvaluate(double lambda, cudaStream_t stream = nullptr);
  void applyExplicitDamping(double lambda, cudaStream_t stream = nullptr);
  void evaluateSolvedDelta(cudaStream_t stream = nullptr);
  size_t analysisCount() const;
  DeviceSparseJacobianAttemptResult downloadAttemptResult(
      cudaStream_t stream = nullptr) const;

  const DeviceSparseJacobianProfile& profile() const;
  const CudaLinearSolveStats& linearSolveStats() const;
  const std::vector<int>& appliedScalarPermutation() const;

  // True only in cuDSS mode, where the normal matrix H is materialized;
  // system() throws std::logic_error otherwise.
  bool hasNormalMatrix() const;
  DeviceSparseNormalEquations& mutableSystem();
  const DeviceSparseNormalEquations& system() const;
  CudaDeviceArray<double>& deviceDelta();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace gtsam::cuda
