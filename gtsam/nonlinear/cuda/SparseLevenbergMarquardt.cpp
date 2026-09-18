/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SparseLevenbergMarquardt.cpp
 * @brief   Levenberg-Marquardt with a persistent CUDA sparse linear system
 * @author  Ruogu Li
 * @date    Jul 14, 2026
 */

#include <cuda_runtime_api.h>
#include <gtsam/base/cuda/Context.h>
#include <gtsam/base/types.h>
#include <gtsam/config.h>
#include <gtsam/linear/cuda/LinearSolver.h>
#include <gtsam/linear/cuda/internal/BlockOrdering.h>
#include <gtsam/linear/cuda/internal/PcgSolver.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/cuda/SparseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/internal/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/nonlinear/cuda/internal/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/internal/PcgLmPolicy.h>
#include <gtsam/nonlinear/cuda/internal/SparseJacobianPlan.h>
#include <gtsam/nonlinear/cuda/internal/StreamingSparseJacobianLinearizer.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtPolicy.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>

namespace gtsam::cuda {
namespace {

using Clock = std::chrono::steady_clock;

double secondsSince(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

Clock::time_point maybeStartTiming(bool enabled) {
  return enabled ? Clock::now() : Clock::time_point{};
}

void accumulateTiming(bool enabled, const Clock::time_point& start,
                      double* total) {
  if (enabled) *total += secondsSince(start);
}

template <class Function>
decltype(auto) runDeviceStage(const char* stage, Function&& function) {
  try {
    return std::forward<Function>(function)();
  } catch (const std::exception& error) {
    throw std::runtime_error(std::string("CUDA sparse LM failure during ") +
                             stage + ": " + error.what());
  } catch (...) {
    throw std::runtime_error(std::string("CUDA sparse LM failure during ") +
                             stage + ": unknown exception");
  }
}

std::string runtimeDiscoveryDetail(cudaError_t status) {
  std::ostringstream message;
  message << "CUDA device discovery failed: " << cudaGetErrorName(status)
          << " (" << cudaGetErrorString(status) << ")";
  return message.str();
}

std::string directFailureDetail(const NonlinearFactorGraph& graph,
                                const DirectJacobianStatus& status) {
  std::ostringstream message;
  message << "direct sparse Jacobian linearization is unsupported";
  if (status.factorIndex != std::numeric_limits<size_t>::max()) {
    message << " at factor " << status.factorIndex;
    if (status.factorIndex < graph.size() && graph[status.factorIndex]) {
      // Bound to a reference first because typeid over an operand with side
      // effects warns under -Wpotentially-evaluated-expression.
      const NonlinearFactor& factor = *graph[status.factorIndex];
      message << " (" << demangle(typeid(factor).name()) << ")";
    }
  }
  if (!status.detail.empty()) {
    message << ": " << status.detail;
  }
  return message.str();
}

void requireFiniteInitialError(double error) {
  if (!std::isfinite(error)) {
    throw std::runtime_error(
        "CUDA sparse LM initial nonlinear error is non-finite");
  }
}

void requireFiniteAttempt(const DeviceSparseJacobianAttemptResult& attempt) {
  if (!std::isfinite(attempt.model.oldError) ||
      !std::isfinite(attempt.model.newError) ||
      !std::isfinite(attempt.model.change())) {
    throw std::runtime_error(
        "CUDA sparse LM model-error evaluation produced a non-finite value");
  }
  for (Eigen::Index index = 0; index < attempt.delta.size(); ++index) {
    if (!std::isfinite(attempt.delta(index))) {
      throw std::runtime_error(
          "CUDA sparse LM solve produced a non-finite delta");
    }
  }
}

struct LevenbergMarquardtState {
  double lambda = 0.0;
  double currentFactor = 0.0;
  size_t acceptedIterations = 0;
  size_t totalInnerAttempts = 0;
};

}  // namespace

struct SparseLevenbergMarquardtOptimizer::Impl {
  std::shared_ptr<const NonlinearFactorGraph> graph;
  Values initialValues;
  Values currentValues;
  SparseLevenbergMarquardtParams parameters;
  SparseLevenbergMarquardtResult optimizationResult;
  double currentError = std::numeric_limits<double>::quiet_NaN();
  bool optimized = false;
  Clock::time_point totalWallStart;
  bool totalWallTimingActive = false;

  // Destruction is in reverse declaration order. In particular, device is
  // destroyed (and synchronizes its fixed stream) before the pinned host
  // buffer and before the stream-owning context.
  std::unique_ptr<Context> context;
  std::unique_ptr<KeyInfo> keyInfo;
  std::unique_ptr<SparseJacobianPlan> plan;
  std::unique_ptr<HostSparseJacobian> host;
  std::unique_ptr<StreamingSparseJacobianLinearizer> linearizer;
  std::unique_ptr<DeviceSparseJacobianNormalEquations> device;
  std::unique_ptr<LinearSolverSession> linearSession;

  Impl(const NonlinearFactorGraph& inputGraph, const Values& inputValues,
       const SparseLevenbergMarquardtParams& inputParameters)
      : graph(inputGraph.cloneShared()),
        initialValues(inputValues),
        currentValues(inputValues),
        parameters(inputParameters) {
    if (!graph) {
      throw std::invalid_argument(
          "SparseLevenbergMarquardtOptimizer graph clone is null");
    }
  }

  void startTotalWallTiming() {
    totalWallTimingActive = parameters.collectTiming;
    if (totalWallTimingActive) totalWallStart = Clock::now();
  }

  void finishTotalWallTiming() {
    if (!totalWallTimingActive) return;
    optimizationResult.timings.totalWall += secondsSince(totalWallStart);
    totalWallTimingActive = false;
  }

  void snapshotDeviceProfile() {
    SparseLevenbergMarquardtSystemSize& size = optimizationResult.systemSize;
    size.factors = graph->size();
    if (plan) {
      size.jacobianRows = static_cast<size_t>(plan->rows());
      size.jacobianColumns = static_cast<size_t>(plan->columns());
      size.jacobianNonzeros = static_cast<size_t>(plan->nonzeros());
    }
    if (!device) {
      return;
    }

    size.normalNonzeros =
        device->hasNormalMatrix()
            ? static_cast<size_t>(device->system().nonzeros())
            : 0;
    const DeviceSparseJacobianProfile& deviceProfile = device->profile();
    optimizationResult.linearSolveStats = linearSession->stats();
    SparseLevenbergMarquardtTransferCounts& transfers = optimizationResult.transfers;
    transfers.patternH2dBytes = deviceProfile.patternH2dBytes;
    transfers.numericH2dBytes = deviceProfile.numericH2dBytes;
    transfers.setupD2hBytes = deviceProfile.setupD2hBytes;
    transfers.attemptD2hBytes = deviceProfile.attemptD2hBytes;
    transfers.pcgD2hBytes = optimizationResult.linearSolveStats.pcgD2hBytes;

    SparseLevenbergMarquardtStageTimings& timings = optimizationResult.timings;
    timings.deviceInitializeWall = deviceProfile.initializeWall;
    timings.patternH2d = deviceProfile.patternH2d;
    timings.structureSetup = deviceProfile.structureSetup;
    timings.setupD2h = deviceProfile.setupD2h;
    timings.numericH2d = deviceProfile.numericH2d;
    timings.transposeUpdate = deviceProfile.transposeUpdate;
    timings.normalJtJ = deviceProfile.normalJtJ;
    timings.normalJtb = deviceProfile.normalJtb;
    timings.diagonalExtraction = deviceProfile.diagonalExtraction;
    timings.oldModelError = deviceProfile.oldModelError;
    timings.dampingPreparation = deviceProfile.dampingPreparation;
    timings.dampingApplication = deviceProfile.dampingApplication;
    timings.pcgPreconditionerBuild = deviceProfile.pcgPreconditionerBuild;
    if (parameters.collectTiming &&
        optimizationResult.linearSolveStats.backend ==
            LinearSolverType::Cudss) {
      timings.cudssAnalysis =
          optimizationResult.linearSolveStats.analysisSeconds;
      timings.cudssFactorAndSolve =
          optimizationResult.linearSolveStats.factorizationSeconds +
          optimizationResult.linearSolveStats.solveSeconds;
      timings.cudssDataInfoBoundaryWall =
          optimizationResult.linearSolveStats.dataInfoBoundarySeconds;
    } else if (optimizationResult.linearSolveStats.backend ==
               LinearSolverType::Pcg) {
      timings.pcgSolve = optimizationResult.linearSolveStats.solveSeconds;
      timings.pcgD2h = optimizationResult.linearSolveStats.pcgD2hSeconds;
      optimizationResult.pcgIterationsTotal =
          optimizationResult.linearSolveStats.pcgIterationsTotal;
      optimizationResult.pcgSolves =
          optimizationResult.linearSolveStats.solveCount;
      optimizationResult.pcgMaxIterationHits =
          optimizationResult.linearSolveStats.pcgMaxIterationHits;
    }
    timings.newModelError = deviceProfile.newModelError;
    timings.attemptD2h = deviceProfile.attemptD2h;
    timings.attemptHostBuild = deviceProfile.attemptHostBuild;
  }

  void clearDeviceState() {
    linearSession.reset();
    device.reset();
    linearizer.reset();
    host.reset();
    plan.reset();
    keyInfo.reset();
    context.reset();
  }

  void updateFinalState(const LevenbergMarquardtState& state) {
    snapshotDeviceProfile();
    optimizationResult.iterations = state.acceptedIterations;
    optimizationResult.acceptedSteps = state.acceptedIterations;
    optimizationResult.lambdaAttempts = state.totalInnerAttempts;
    optimizationResult.finalError = currentError;
    optimizationResult.finalLambda = state.lambda;
    optimizationResult.cudssAnalyses =
        linearSession ? linearSession->stats().analysisCount : 0;
  }

  const Values& finish(SparseLevenbergMarquardtTerminationReason reason,
                       const LevenbergMarquardtState& state) {
    optimizationResult.termination = reason;
    updateFinalState(state);
    finishTotalWallTiming();
    optimized = true;
    return currentValues;
  }

  const Values& runCpuFallback(SparseLevenbergMarquardtFallbackReason reason,
                               const DirectJacobianStatus& status,
                               const std::string& detail,
                               const LevenbergMarquardtState& state) {
    if (!parameters.fallbackOnUnsupported) {
      throw std::runtime_error(detail);
    }

    updateFinalState(state);
    finishTotalWallTiming();
    clearDeviceState();
    LevenbergMarquardtParams cpuParameters = parameters;
    LevenbergMarquardtOptimizer cpu(*graph, initialValues, cpuParameters);
    currentValues = cpu.optimize();
    currentError = graph->error(currentValues);
    optimizationResult.backend = SparseLevenbergMarquardtBackend::CpuFallback;
    optimizationResult.fallbackReason = reason;
    optimizationResult.fallbackStatus = status;
    optimizationResult.fallbackDetail = detail;
    optimizationResult.finalError = currentError;
    optimized = true;
    return currentValues;
  }

  const Values& optimize() {
    if (optimized) {
      return currentValues;
    }

    clearDeviceState();
    currentValues = initialValues;
    optimizationResult = {};
    optimizationResult.systemSize.factors = graph->size();
    optimizationResult.finalLambda = parameters.lambdaInitial;
    startTotalWallTiming();

    Clock::time_point stageStart = maybeStartTiming(parameters.collectTiming);
    currentError = graph->error(currentValues);
    accumulateTiming(parameters.collectTiming, stageStart,
                     &optimizationResult.timings.initialError);
    requireFiniteInitialError(currentError);
    optimizationResult.initialError = currentError;
    optimizationResult.finalError = currentError;

    LevenbergMarquardtState state{parameters.lambdaInitial, parameters.lambdaFactor};
    if (currentError <= parameters.errorTol) {
      return finish(SparseLevenbergMarquardtTerminationReason::ErrorThreshold, state);
    }
    if (parameters.maxIterations == 0) {
      return finish(SparseLevenbergMarquardtTerminationReason::MaxIterations, state);
    }

    int deviceCount = 0;
    const cudaError_t discoveryStatus = cudaGetDeviceCount(&deviceCount);
    if (discoveryStatus != cudaSuccess) {
      const std::string detail = runtimeDiscoveryDetail(discoveryStatus);
      // Do not leave a discovery error pending for the CPU fallback.
      (void)cudaGetLastError();
      return runCpuFallback(SparseLevenbergMarquardtFallbackReason::RuntimeUnavailable, {},
                            detail, state);
    }
    if (deviceCount <= 0) {
      return runCpuFallback(SparseLevenbergMarquardtFallbackReason::RuntimeUnavailable, {},
                            "no CUDA device is available", state);
    }

    if (parameters.linear.backend == LinearSolverType::DenseCholesky) {
      throw std::invalid_argument(
          "general CUDA sparse LM does not support dense Cholesky");
    }
    const DeviceNormalSolverBackend deviceBackend =
        parameters.linear.backend == LinearSolverType::Pcg
            ? DeviceNormalSolverBackend::Pcg
            : DeviceNormalSolverBackend::Cudss;
#if !GTSAM_ENABLE_CUDSS
    if (deviceBackend == DeviceNormalSolverBackend::Cudss) {
      return runCpuFallback(SparseLevenbergMarquardtFallbackReason::CudssUnavailable, {},
                            "cuDSS support is not compiled", state);
    }
#endif
    const DeviceSparseJacobianCapability capability =
        DeviceSparseJacobianNormalEquations::preflightCapability(deviceBackend);
    if (!capability.supported) {
      return runCpuFallback(SparseLevenbergMarquardtFallbackReason::ToolkitUnsupported,
                            {}, capability.detail, state);
    }

    const Clock::time_point planStart =
        maybeStartTiming(parameters.collectTiming);
    try {
      keyInfo = std::make_unique<KeyInfo>(initialValues.dims());
      plan = std::make_unique<SparseJacobianPlan>(*graph, *keyInfo);
      if (plan->rows() <= 0 || plan->columns() <= 0 || plan->nonzeros() <= 0) {
        throw std::invalid_argument(
            "direct sparse Jacobian plan must have positive dimensions");
      }
    } catch (const std::invalid_argument& error) {
      accumulateTiming(parameters.collectTiming, planStart,
                       &optimizationResult.timings.plan);
      return runCpuFallback(
          SparseLevenbergMarquardtFallbackReason::PlanIncompatible, {},
          std::string("sparse Jacobian plan is incompatible: ") + error.what(),
          state);
    }
    accumulateTiming(parameters.collectTiming, planStart,
                     &optimizationResult.timings.plan);
    snapshotDeviceProfile();

    DeviceNormalSolverOptions solverOptions;
    solverOptions.backend = deviceBackend;
    if (parameters.ordering) {
      if (deviceBackend != DeviceNormalSolverBackend::Cudss) {
        throw std::invalid_argument(
            "CUDA sparse LM GTSAM ordering is supported only by cuDSS");
      }
      solverOptions.scalarPermutation =
          compileScalarPermutation(*keyInfo, *parameters.ordering);
      optimizationResult.appliedScalarPermutation =
          solverOptions.scalarPermutation;
    }
    if (deviceBackend == DeviceNormalSolverBackend::Pcg) {
      solverOptions.pcg.maxIterations = parameters.pcg.maxIterations;
      solverOptions.pcg.relativeTolerance = parameters.pcg.relativeTolerance;
      solverOptions.pcg.warmStart = parameters.pcg.warmStart;
      solverOptions.pcg.convergenceCheckInterval =
          parameters.pcg.convergenceCheckInterval;
      solverOptions.pcg.preconditioner =
          DevicePcgPreconditioner::BlockJacobi;
      solverOptions.columnBlockOffsets = cudaBlockOffsets(*keyInfo);
    }

    stageStart = maybeStartTiming(parameters.collectTiming);
    runDeviceStage("persistent setup", [&] {
      context = std::make_unique<Context>();
      host = std::make_unique<HostSparseJacobian>(*plan);
      linearizer = std::make_unique<StreamingSparseJacobianLinearizer>();
      device = std::make_unique<DeviceSparseJacobianNormalEquations>();
      device->initialize(*plan, context->stream(), parameters.collectTiming,
                         solverOptions);
      linearSession =
          std::make_unique<LinearSolverSession>(parameters.linear);
      if (deviceBackend == DeviceNormalSolverBackend::Pcg) {
        linearSession->analyze(plan->columns(), parameters.pcg, context->stream(),
                               parameters.collectTiming);
      }
    });
    accumulateTiming(parameters.collectTiming, stageStart,
                     &optimizationResult.timings.persistentSetupWall);
    snapshotDeviceProfile();

    while (true) {
      const double previousError = currentError;
      const size_t acceptedBefore = state.acceptedIterations;
      ++optimizationResult.outerLinearizations;

      stageStart = maybeStartTiming(parameters.collectTiming);
      host->clear();
      accumulateTiming(parameters.collectTiming, stageStart,
                       &optimizationResult.timings.hostZero);

      stageStart = maybeStartTiming(parameters.collectTiming);
      StreamingLinearizationProfile streamingProfile;
      const DirectJacobianStatus linearizationStatus = linearizer->linearize(
          *graph, currentValues, *keyInfo, *plan, host.get(), nullptr,
          parameters.validateStructureEveryIteration,
          parameters.collectTiming ? &streamingProfile : nullptr);
      if (parameters.collectTiming) {
        optimizationResult.timings.factorLinearizationAndPackingWall +=
            secondsSince(stageStart);
        optimizationResult.timings.factorLinearizationCpuSum +=
            streamingProfile.factorLinearizationCpuSum;
        optimizationResult.timings.csrPackingCpuSum +=
            streamingProfile.csrPackingCpuSum;
      }
      if (!linearizationStatus.ok()) {
        const std::string detail =
            directFailureDetail(*graph, linearizationStatus);
        if (linearizationStatus.failure ==
            DirectJacobianFailure::NonFiniteValues) {
          throw std::runtime_error(
              "CUDA sparse LM failure during factor linearization: " + detail);
        }
        const bool globalStructuralMismatch =
            linearizationStatus.failure ==
                DirectJacobianFailure::StructuralMismatch &&
            linearizationStatus.factorIndex ==
                std::numeric_limits<size_t>::max();
        return runCpuFallback(
            globalStructuralMismatch
                ? SparseLevenbergMarquardtFallbackReason::PlanIncompatible
                : SparseLevenbergMarquardtFallbackReason::DirectJacobianUnsupported,
            linearizationStatus, detail, state);
      }

      runDeviceStage("numeric upload",
                   [&] { device->uploadNumerics(*host, context->stream()); });

      runDeviceStage("normal-equation formation",
                   [&] { device->formUndampedSystem(context->stream()); });

      runDeviceStage("damping preparation", [&] {
        device->prepareDamping(parameters.dampingParams.diagonalDamping,
                               parameters.dampingParams.minDiagonal,
                               parameters.dampingParams.maxDiagonal,
                               context->stream());
      });

      if (deviceBackend == DeviceNormalSolverBackend::Pcg) {
        linearSession->invalidateWarmStart();
      }

      runDeviceStage("linear analysis", [&] {
        if (linearSession &&
            linearSession->stats().analysisCount == 0) {
          if (solverOptions.scalarPermutation.empty()) {
            linearSession->analyze(device->mutableSystem(),
                                   &device->deviceDelta(), context->stream());
          } else {
            linearSession->analyze(device->mutableSystem(),
                                   &device->deviceDelta(),
                                   solverOptions.scalarPermutation,
                                   context->stream());
          }
        }
      });

      bool accepted = false;
      SparseLevenbergMarquardtTerminationReason innerTermination =
          SparseLevenbergMarquardtTerminationReason::None;
      size_t attemptWithinOuter = 0;
      while (true) {
        SparseLevenbergMarquardtAttemptRecord attemptRecord;
        attemptRecord.acceptedIterationsBeforeAttempt = acceptedBefore;
        attemptRecord.attempt = attemptWithinOuter++;
        attemptRecord.lambda = state.lambda;

        runDeviceStage("CUDA linear solve", [&] {
          if (deviceBackend == DeviceNormalSolverBackend::Cudss) {
            device->applyExplicitDamping(state.lambda, context->stream());
            linearSession->solve(device->mutableSystem(),
                                 &device->deviceDelta(), context->stream());
          } else {
            device->prepareOperatorSystem(state.lambda, context->stream());
            linearSession->solve(
                device->linearOperator(), device->preconditioner(),
                device->deviceRhs(), &device->deviceDelta(),
                context->stream());
          }
        });

        bool rejectPcgStep = false;
        if (deviceBackend == DeviceNormalSolverBackend::Pcg) {
          const LinearSolveStats& stats = linearSession->stats();
          attemptRecord.pcgSolve = true;
          attemptRecord.pcgIterations = stats.lastPcgIterations;
          attemptRecord.pcgConverged = stats.lastPcgConverged;
          attemptRecord.pcgBreakdown = stats.lastPcgBreakdown;
          rejectPcgStep = classifyPcgLmStep(stats) ==
                          PcgLmStepDisposition::RejectAndRetry;
        }
        ++state.totalInnerAttempts;

        if (rejectPcgStep) {
          snapshotDeviceProfile();
          linearSession->invalidateWarmStart();
          gtsam::internal::increaseLevenbergMarquardtLambda(
              parameters, &state.lambda, &state.currentFactor);
          if (state.lambda >= parameters.lambdaUpperBound) {
            innerTermination =
                SparseLevenbergMarquardtTerminationReason::LambdaUpperBound;
          }
          if (parameters.collectAttemptTrace) {
            optimizationResult.attemptTrace.push_back(attemptRecord);
          }
          if (innerTermination !=
              SparseLevenbergMarquardtTerminationReason::None) {
            break;
          }
          continue;
        }

        runDeviceStage("linear model evaluation", [&] {
          device->evaluateSolvedDelta(context->stream());
        });
        DeviceSparseJacobianAttemptResult deviceAttempt = runDeviceStage(
            "attempt result download",
            [&] { return device->downloadAttemptResult(context->stream()); });
        if (deviceBackend == DeviceNormalSolverBackend::Pcg) {
          deviceAttempt.pcgIterations =
              static_cast<int>(attemptRecord.pcgIterations);
          deviceAttempt.pcgConverged = attemptRecord.pcgConverged;
        }
        snapshotDeviceProfile();

        requireFiniteAttempt(deviceAttempt);
        const double oldLinearizedError = deviceAttempt.model.oldError;
        const double linearizedChange = deviceAttempt.model.change();
        attemptRecord.linearizedChange = linearizedChange;

        bool stopSearchingLambda = false;
        bool stepSuccessful = false;
        double costChange = 0.0;
        double modelFidelity = 0.0;
        Values trialValues;
        double trialError = std::numeric_limits<double>::infinity();

        if (linearizedChange >= 0.0) {
          const VectorValues delta =
              buildVectorValues(deviceAttempt.delta, *keyInfo);
          stageStart = maybeStartTiming(parameters.collectTiming);
          trialValues = currentValues.retract(delta);
          accumulateTiming(parameters.collectTiming, stageStart,
                           &optimizationResult.timings.retract);

          stageStart = maybeStartTiming(parameters.collectTiming);
          trialError = graph->error(trialValues);
          accumulateTiming(parameters.collectTiming, stageStart,
                           &optimizationResult.timings.nonlinearTrialError);
          if (!std::isfinite(trialError)) {
            throw std::runtime_error(
                "CUDA sparse LM failure during nonlinear trial error: "
                "non-finite error");
          }

          costChange = currentError - trialError;
          if (linearizedChange >
              std::numeric_limits<double>::epsilon() * oldLinearizedError) {
            modelFidelity = costChange / linearizedChange;
            if (!std::isfinite(modelFidelity)) {
              throw std::runtime_error(
                  "CUDA sparse LM model fidelity is non-finite");
            }
            stepSuccessful = modelFidelity > parameters.minModelFidelity;
          }

          const double minimumAbsoluteTolerance =
              parameters.relativeErrorTol * currentError;
          if (std::abs(costChange) < minimumAbsoluteTolerance) {
            stopSearchingLambda = true;
          }
        }

        attemptRecord.nonlinearChange = costChange;
        attemptRecord.modelFidelity = modelFidelity;
        attemptRecord.accepted = stepSuccessful;

        if (stepSuccessful) {
          currentValues = std::move(trialValues);
          currentError = trialError;
          ++state.acceptedIterations;
          gtsam::internal::decreaseLevenbergMarquardtLambda(
              parameters, modelFidelity, &state.lambda,
              &state.currentFactor);
          accepted = true;
        } else if (stopSearchingLambda) {
          innerTermination = SparseLevenbergMarquardtTerminationReason::SmallCostChange;
        } else {
          gtsam::internal::increaseLevenbergMarquardtLambda(
              parameters, &state.lambda, &state.currentFactor);
          if (state.lambda >= parameters.lambdaUpperBound) {
            innerTermination = SparseLevenbergMarquardtTerminationReason::LambdaUpperBound;
          }
        }

        if (parameters.collectAttemptTrace) {
          optimizationResult.attemptTrace.push_back(attemptRecord);
        }
        if (accepted ||
            innerTermination != SparseLevenbergMarquardtTerminationReason::None) {
          break;
        }
      }

      if (parameters.iterationHook) {
        parameters.iterationHook(state.acceptedIterations, previousError,
                                 currentError);
      }

      if (!accepted) {
        return finish(innerTermination, state);
      }
      if (state.acceptedIterations >= parameters.maxIterations) {
        return finish(SparseLevenbergMarquardtTerminationReason::MaxIterations, state);
      }
      if (checkConvergence(parameters, previousError, currentError)) {
        return finish(SparseLevenbergMarquardtTerminationReason::Converged, state);
      }
    }
  }
};

SparseLevenbergMarquardtOptimizer::SparseLevenbergMarquardtOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const SparseLevenbergMarquardtParams& params)
    : impl_(std::make_unique<Impl>(graph, initialValues, params)) {}

SparseLevenbergMarquardtOptimizer::
    ~SparseLevenbergMarquardtOptimizer() = default;

const Values& SparseLevenbergMarquardtOptimizer::optimize() {
  return impl_->optimize();
}

const Values& SparseLevenbergMarquardtOptimizer::values() const {
  return impl_->currentValues;
}

double SparseLevenbergMarquardtOptimizer::error() const {
  return impl_->currentError;
}

const SparseLevenbergMarquardtParams&
SparseLevenbergMarquardtOptimizer::params() const {
  return impl_->parameters;
}

const SparseLevenbergMarquardtResult&
SparseLevenbergMarquardtOptimizer::result() const {
  return impl_->optimizationResult;
}

}  // namespace gtsam::cuda
