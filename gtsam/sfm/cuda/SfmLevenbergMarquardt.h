/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SfmLevenbergMarquardt.h
 * @brief   Levenberg-Marquardt bundle adjustment that iterates on the GPU
 * @author  Ruogu Li
 * @date    Jun 17, 2026
 */

#pragma once

#include <gtsam/dllexport.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/sfm/SfmEliminationMode.h>
#include <gtsam/sfm/cuda/SfmTypes.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace gtsam::cuda {

class SfmLevenbergMarquardtOptimizer;
struct SfmLevenbergMarquardtResult;

/// Parameters for CUDA-resident SFM Levenberg-Marquardt optimization.
class GTSAM_EXPORT SfmLevenbergMarquardtParams
    : public LevenbergMarquardtParams {
 public:
  using OptimizerType = SfmLevenbergMarquardtOptimizer;

  /// Fills the per-iteration and per-attempt profiles in the result. This costs
  /// a stream synchronization per measured stage, so it slows the run down and
  /// should be off when timing.
  bool enableDetailedProfiling;
  /// Numerical backend for the reduced Schur-complement system.
  LinearSolverOptions linear{
      gtsam::cuda::LinearSolverType::DenseCholesky};
  /// Whether to solve the full system or eliminate landmarks first.
  SfmEliminationMode eliminationMode = SfmEliminationMode::Schur;
  /// The inherited optional Ordering contains camera keys only.
  PcgOptions pcg;

  /// Creates parameters using GTSAM's legacy LM defaults.
  SfmLevenbergMarquardtParams();

  /// Returns parameters initialized with GTSAM's legacy LM defaults.
  static SfmLevenbergMarquardtParams legacyDefaults();
  /// Returns parameters initialized with GTSAM's Ceres-style LM defaults.
  static SfmLevenbergMarquardtParams ceresDefaults();

  /// Returns the numerical backend for the reduced Schur system.
  gtsam::cuda::LinearSolverType getLinearSolver() const {
    return linear.backend;
  }
  /// Sets the numerical backend for the reduced Schur system.
  void setLinearSolver(gtsam::cuda::LinearSolverType solver) {
    linear.backend = solver;
  }
  /// Returns whether landmarks are eliminated before the linear solve.
  SfmEliminationMode getEliminationMode() const { return eliminationMode; }
  /// Selects full-system or Schur-complement elimination.
  void setEliminationMode(SfmEliminationMode mode) { eliminationMode = mode; }
  /// Returns the lower clamp applied to damping diagonals.
  double getMinDiagonal() const { return dampingParams.minDiagonal; }
  /// Returns the upper clamp applied to damping diagonals.
  double getMaxDiagonal() const { return dampingParams.maxDiagonal; }
  /// Sets the lower clamp applied to damping diagonals.
  void setMinDiagonal(double value) { dampingParams.minDiagonal = value; }
  /// Sets the upper clamp applied to damping diagonals.
  void setMaxDiagonal(double value) { dampingParams.maxDiagonal = value; }
  /// Prints this parameter set.
  void print(const std::string& str = "") const override;
  /// Tests numerical equality with another parameter set.
  bool equals(const SfmLevenbergMarquardtParams& other,
              double tol = 1e-9) const;
  /// Return a polymorphic copy preserving CUDA and elimination settings.
  std::shared_ptr<NonlinearOptimizerParams> clone() const {
    return std::make_shared<SfmLevenbergMarquardtParams>(*this);
  }
};

/**
 * Levenberg-Marquardt bundle adjustment that runs the whole iteration on the
 * GPU.
 *
 * Where SparseLevenbergMarquardtOptimizer linearizes a general factor graph on
 * the CPU and solves the normal equations on the device, this optimizer
 * converts the graph once into device-resident SFM data and then keeps
 * linearization, damping, Schur elimination, the linear solve, retraction, and
 * error evaluation on the device for every LM attempt. Only scalar convergence
 * quantities cross back to the host.
 *
 * The graph must contain only `GeneralSFMFactor<SfmCamera, Point3>` or
 * `BatchFactor<GeneralSFMFactor<SfmCamera, Point3>, 2>` factors, over values
 * holding `SfmCamera` (that is, `PinholeCamera<Cal3Bundler>`) cameras and
 * `Point3` landmarks; any other factor type throws. Values of other types are
 * carried through untouched. Landmarks are eliminated with a Schur complement,
 * and `SfmLevenbergMarquardtParams::linear` selects how the reduced camera
 * system is solved; see `docs/CUDA_LINEAR_SOLVERS.md`.
 *
 * As elsewhere in GTSAM the parameter type selects the optimizer, so passing
 * `SfmLevenbergMarquardtParams` is what routes a problem here — including
 * through `GncOptimizer<GncParams<SfmLevenbergMarquardtParams>>` for robust
 * bundle adjustment. `optimize()` may be called again to continue from the
 * values reached by the previous run.
 *
 * The profiling and result types reachable through `result()` are defined
 * below the class, since inspecting them is not part of ordinary use.
 */
class GTSAM_EXPORT SfmLevenbergMarquardtOptimizer {
 public:
  /// Creates an optimizer for a supported factor graph and initial values.
  SfmLevenbergMarquardtOptimizer(
      const NonlinearFactorGraph& graph, const Values& initialValues,
      const SfmLevenbergMarquardtParams& params =
          SfmLevenbergMarquardtParams());
  ~SfmLevenbergMarquardtOptimizer();

  /// Runs optimization and returns the final values.
  const Values& optimize();

  /// Returns the latest optimized values, or the initial values before optimize().
  const Values& values() const { return values_; }
  /// Returns the nonlinear objective at the current values.
  double error() const { return graph_.error(values_); }
  /// Returns the number of accepted nonlinear iterations in the latest run.
  size_t iterations() const;

  /// Returns the optimizer parameters.
  const SfmLevenbergMarquardtParams& params() const { return params_; }
  /// Returns profiling and convergence results from the latest run.
  const SfmLevenbergMarquardtResult& result() const;

 private:
  NonlinearFactorGraph graph_;
  Values values_;
  SfmLevenbergMarquardtParams params_;
  // Held indirectly so the result and profiling types can be defined after
  // this class instead of ahead of it.
  std::unique_ptr<SfmLevenbergMarquardtResult> result_;
};

/**
 * One trial step at one damping value: what it cost, and what LM decided.
 *
 * LM may try several lambdas per iteration before it accepts a step or gives up,
 * and each of those tries is one of these. The timings are only filled when
 * SfmLevenbergMarquardtParams::enableDetailedProfiling is set, and are wall time
 * around a stream synchronization rather than device time.
 */
struct SfmLevenbergMarquardtAttemptProfile {
  /// Nonlinear iteration this attempt belongs to, counting from zero.
  int iteration = 0;
  /// Index of the attempt within that iteration.
  int attempt = 0;
  /// Damping value tried.
  double lambda = 0.0;
  /// Wall time for the whole attempt.
  double totalElapsed = 0.0;
  /// Time in the dense Cholesky solve of the reduced system.
  double denseSchurSolveElapsed = 0.0;
  /// Time assembling the reduced camera system from the device Jacobians.
  double normalEquationsElapsed = 0.0;
  /// Time adding the damping term to the reduced system's diagonal.
  double addDampingElapsed = 0.0;
  /// Time in cuDSS symbolic analysis, nonzero only when it had to be redone.
  double cudssAnalyzeElapsed = 0.0;
  /// Time in the cuDSS numeric factorization and solve.
  double cudssSolveElapsed = 0.0;
  /// Time evaluating the quadratic model's predicted error.
  double linearizedErrorElapsed = 0.0;
  /// Time retracting the values by the computed step.
  double applyDeltaElapsed = 0.0;
  /// Time evaluating the nonlinear error at the trial point.
  double trialErrorElapsed = 0.0;
  /// Time in the lambda update rule.
  double lambdaUpdateElapsed = 0.0;
  /// Quadratic model's error at the current point.
  double oldLinearizedError = 0.0;
  /// Quadratic model's error at the trial point.
  double newLinearizedError = 0.0;
  /// Decrease the quadratic model predicted, old minus new.
  double linearizedCostChange = 0.0;
  /// Nonlinear error actually measured at the trial point.
  double trialError = 0.0;
  /// Decrease actually achieved, current minus trial.
  double costChange = 0.0;
  /// Achieved over predicted decrease, the gain ratio LM tests.
  double modelFidelity = 0.0;
  /// PCG iterations taken, when the backend is PCG.
  size_t pcgIterations = 0;
  /// Whether this attempt's solve went through PCG.
  bool pcgSolve = false;
  /// Whether PCG met its tolerance rather than hitting its iteration cap.
  bool pcgConverged = false;
  /// Whether PCG stopped early on a non-positive curvature or zero denominator.
  bool pcgBreakdown = false;
  /// Whether a trial point was formed and evaluated at all.
  bool attemptedTrial = false;
  /// Whether the step passed LM's acceptance test.
  bool stepSuccessful = false;
  /// Whether the step was taken, updating the values.
  bool accepted = false;
  /// Whether LM stopped trying further lambdas after this attempt.
  bool stopSearchingLambda = false;
  /// Whether lambda hit its upper bound, which ends the lambda search.
  bool lambdaUpperBoundReached = false;
  /// Whether optimization terminated on this attempt.
  bool terminated = false;
};

/**
 * One nonlinear iteration: its error and damping endpoints, the work done once
 * per iteration, and the attempts made inside it.
 *
 * Only filled when SfmLevenbergMarquardtParams::enableDetailedProfiling is set.
 */
struct SfmLevenbergMarquardtIterationProfile {
  /// Index of this iteration, counting from zero.
  int iteration = 0;
  /// Nonlinear error on entry.
  double startError = 0.0;
  /// Nonlinear error on exit.
  double endError = 0.0;
  /// Damping value on entry.
  double startLambda = 0.0;
  /// Damping value on exit.
  double endLambda = 0.0;
  /// Wall time for the whole iteration, attempts included.
  double totalElapsed = 0.0;
  /// Time computing the damping diagonal, done once for all attempts.
  double dampingDiagonalElapsed = 0.0;
  /// Once-per-attempt reduced-system construction.
  double normalEquationsElapsed = 0.0;
  /// Time committing an accepted trial point as the new values.
  double acceptTrialElapsed = 0.0;
  /// Whether any attempt in this iteration was accepted.
  bool acceptedStep = false;
  /// Whether optimization terminated in this iteration.
  bool terminated = false;
  /// One entry per lambda tried, in order.
  std::vector<SfmLevenbergMarquardtAttemptProfile> attemptProfiles;
};

/**
 * Everything one optimization run reports back: the answer, its convergence
 * quantities, and a timing breakdown.
 *
 * The counts, errors, and the four coarse timings — totalMeasuredElapsed,
 * setupElapsed, solveLoopElapsed, downloadElapsed — are always filled. Every
 * finer breakdown, meaning the per-stage times, the transfer byte counts, and
 * the two profile vectors, needs
 * SfmLevenbergMarquardtParams::enableDetailedProfiling and is otherwise zero or
 * empty, since measuring a stage costs a stream synchronization.
 * Backend-specific fields — the cuDSS times, the PCG counters inside
 * linearSolveStats — stay zero when a different backend is in use.
 */
struct SfmLevenbergMarquardtResult {
  /// Elimination mode requested for this run.
  SfmEliminationMode eliminationMode = SfmEliminationMode::Schur;
  /// Backend that solved the reduced camera system.
  LinearSolverType linearBackend = gtsam::cuda::LinearSolverType::DenseCholesky;
  /// Whether that backend saw the system as dense or sparse.
  LinearSystemKind linearSystemKind = LinearSystemKind::Dense;
  /// Side length of the reduced camera system.
  size_t linearSystemDimension = 0;
  /// Nonzeros in it: dimension squared when dense, the reduced CSR's nonzero
  /// count for cuDSS, and zero for the matrix-free PCG operator.
  size_t linearSystemNonzeros = 0;
  /// Nonlinear error at the initial values.
  double initialError = 0.0;
  /// Nonlinear error at the returned values.
  double finalError = 0.0;
  /// Total time attributed to this run, setup and solve loop together.
  double totalMeasuredElapsed = 0.0;
  /// Time converting a NonlinearFactorGraph into packed SFM data. This and the
  /// three fields below are filled only by SfmLevenbergMarquardtOptimizer, and
  /// stay zero when optimizeSfm() is called with SfmData directly.
  double graphConversionElapsed = 0.0;
  /// Time inside the optimizeSfm() call that does the actual work.
  double graphBackendCallElapsed = 0.0;
  /// Time destroying the converted data, which frees the packed measurements.
  double graphConvertedDataDestructionElapsed = 0.0;
  /// Time merging optimized cameras and points back into the caller's Values.
  double graphValueMergeElapsed = 0.0;
  /// Time before the first iteration: context, uploads, initial error.
  double setupElapsed = 0.0;
  /// Time in the nonlinear iteration loop.
  double solveLoopElapsed = 0.0;
  /// Time creating the CUDA context, stream, and library handles.
  double contextElapsed = 0.0;
  /// Time packing and uploading the initial cameras and points.
  double packValuesElapsed = 0.0;
  /// Of that, time building the host-side packed arrays.
  double packValuesHostBuildElapsed = 0.0;
  /// Of that, time allocating their device storage.
  double packValuesDeviceAllocElapsed = 0.0;
  /// Of that, time in the host-to-device copy.
  double packValuesH2dCopyElapsed = 0.0;
  /// Bytes that copy moved.
  size_t packValuesH2dBytes = 0;
  /// Time allocating the trial-point buffers reused every attempt.
  double allocateTrialElapsed = 0.0;
  /// Time building and uploading the projection batch, the observations and
  /// their noise models.
  double projectionBatchElapsed = 0.0;
  /// Of that, time building the host-side batch.
  double projectionBatchHostBuildElapsed = 0.0;
  /// Of that, time allocating its device storage.
  double projectionBatchDeviceAllocElapsed = 0.0;
  /// Of that, time in the host-to-device copy.
  double projectionBatchH2dCopyElapsed = 0.0;
  /// Bytes that copy moved.
  size_t projectionBatchH2dBytes = 0;
  /// Time evaluating the error at the initial values.
  double initialErrorElapsed = 0.0;
  /// Time constructing the dense Schur solver, including its cuSOLVER handle
  /// and workspace.
  double denseSchurSolverConstructionElapsed = 0.0;
  /// Time in the first cuDSS symbolic analysis, separated out because later
  /// iterations reuse it.
  double firstCudssAnalyzeElapsed = 0.0;
  /// Time downloading the final values, zero for the no-download entry points.
  double downloadElapsed = 0.0;
  /// Of that, time allocating the host receive buffers.
  double downloadHostAllocElapsed = 0.0;
  /// Of that, time in the device-to-host copy.
  double downloadD2hCopyElapsed = 0.0;
  /// Of that, time rebuilding a gtsam::Values from the packed arrays.
  double downloadValuesBuildElapsed = 0.0;
  /// Bytes that copy moved.
  size_t downloadD2hBytes = 0;
  /// Time in every host-to-device copy of the run.
  double totalH2dCopyElapsed = 0.0;
  /// Bytes those copies moved.
  size_t totalH2dBytes = 0;
  /// Time in every device-to-host copy of the run.
  double totalD2hCopyElapsed = 0.0;
  /// Bytes those copies moved.
  size_t totalD2hBytes = 0;
  /// Time computing damping diagonals, summed over iterations.
  double dampingDiagonalElapsed = 0.0;
  /// Time in dense Cholesky solves, summed over attempts.
  double denseSchurSolveElapsed = 0.0;
  /// Time assembling reduced systems, summed over attempts.
  double normalEquationsElapsed = 0.0;
  /// Time applying damping, summed over attempts.
  double addDampingElapsed = 0.0;
  /// Time in cuDSS symbolic analysis, summed over attempts.
  double cudssAnalyzeElapsed = 0.0;
  /// Time in cuDSS factorization and solve, summed over attempts.
  double cudssSolveElapsed = 0.0;
  /// Time evaluating predicted errors, summed over attempts.
  double linearizedErrorElapsed = 0.0;
  /// Time retracting trial points, summed over attempts.
  double applyDeltaElapsed = 0.0;
  /// Time evaluating trial errors, summed over attempts.
  double trialErrorElapsed = 0.0;
  /// Time committing accepted steps, summed over iterations.
  double acceptTrialElapsed = 0.0;
  /// Time in the lambda update rule, summed over attempts.
  double lambdaUpdateElapsed = 0.0;
  /// Nonlinear iterations run.
  int iterations = 0;
  /// Trial steps attempted across all of them.
  int innerIterations = 0;
  /// How many of those were accepted.
  int acceptedSteps = 0;
  /// Damping value on exit.
  double finalLambda = 0.0;
  /// Linear-solver counters and device timings for the whole run.
  LinearSolveStats linearSolveStats;
  /// Scalar column permutation handed to cuDSS, compiled from the parameters'
  /// camera-key Ordering. Empty for every other backend, and when no Ordering
  /// was given.
  std::vector<int> appliedScalarPermutation;
  /// One entry per nonlinear iteration, only when detailed profiling is on.
  std::vector<SfmLevenbergMarquardtIterationProfile> iterationProfiles;
  /// The optimized values, empty for the no-download entry points.
  Values optimizedValues;
};

/**
 * A general SFM factor graph flattened into the arrays the device path wants.
 *
 * convertGeneralSfmGraph() produces this: measurements go into an SfmData, the
 * Keys they came from are kept in the two vectors so results can be merged back
 * into the caller's Values, and the noise models are reduced to whitening and
 * robust-loss descriptions indexed the same way as the tracks in `data`. The two
 * flags let the optimizer skip whitening and reweighting kernels for the common
 * unit-noise case.
 */
struct SfmFactorGraphData {
  /// Cameras, tracks, and their measurements.
  SfmData data;
  /// Key of each camera, parallel to data.cameras.
  std::vector<Key> cameraKeys;
  /// Key of each landmark, parallel to data.tracks.
  std::vector<Key> pointKeys;
  /// Whitening matrix per measurement, indexed as data.tracks[t].measurements.
  std::vector<std::vector<SfmSqrtInfo2>> sqrtInfoByTrack;
  /// Robust loss per measurement, indexed the same way.
  std::vector<std::vector<SfmRobustModel>> robustModelsByTrack;
  /// Whether any measurement needs whitening at all.
  bool hasNonUnitNoise = false;
  /// Whether any measurement carries a robust loss.
  bool hasRobustNoise = false;
};

/// Converts a supported general factor graph into packed SFM input data.
GTSAM_EXPORT SfmFactorGraphData convertGeneralSfmGraph(
    const NonlinearFactorGraph& graph, const Values& initialValues);

/// Optimizes SFM data and downloads the optimized values.
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params);

/// Optimizes SFM data without downloading the final device values.
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data,
    const SfmLevenbergMarquardtParams& params);

/// Optimizes SFM data using explicit camera and point keys.
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfm(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params);

/// Optimizes keyed SFM data without downloading final device values.
GTSAM_EXPORT SfmLevenbergMarquardtResult optimizeSfmWithoutValueDownload(
    const SfmData& data, const std::vector<Key>& cameraKeys,
    const std::vector<Key>& pointKeys,
    const SfmLevenbergMarquardtParams& params);

}  // namespace gtsam::cuda
