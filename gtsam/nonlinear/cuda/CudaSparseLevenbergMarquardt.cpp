#include <gtsam/nonlinear/cuda/CudaSparseLevenbergMarquardt.h>

#include <gtsam/base/cuda/CudaContext.h>
#include <gtsam/base/types.h>
#include <gtsam/config.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/cuda/DeviceSparseJacobianNormalEquations.h>
#include <gtsam/nonlinear/cuda/HostSparseJacobian.h>
#include <gtsam/nonlinear/cuda/SparseJacobianPlan.h>

#include <cuda_runtime_api.h>

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

double SecondsSince(const Clock::time_point& start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

template <class Function>
decltype(auto) RunCudaStage(const char* stage, Function&& function) {
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

std::string CudaDiscoveryDetail(cudaError_t status) {
  std::ostringstream message;
  message << "CUDA device discovery failed: " << cudaGetErrorName(status)
          << " (" << cudaGetErrorString(status) << ")";
  return message.str();
}

std::string DirectFailureDetail(const NonlinearFactorGraph& graph,
                                const DirectJacobianStatus& status) {
  std::ostringstream message;
  message << "direct sparse Jacobian linearization is unsupported";
  if (status.factorIndex != std::numeric_limits<size_t>::max()) {
    message << " at factor " << status.factorIndex;
    if (status.factorIndex < graph.size() && graph[status.factorIndex]) {
      message << " (" << demangle(typeid(*graph[status.factorIndex]).name())
              << ")";
    }
  }
  if (!status.detail.empty()) {
    message << ": " << status.detail;
  }
  return message.str();
}

void RequireFiniteInitialError(double error) {
  if (!std::isfinite(error)) {
    throw std::runtime_error(
        "CUDA sparse LM initial nonlinear error is non-finite");
  }
}

void RequireFiniteAttempt(const DeviceSparseJacobianAttemptResult& attempt) {
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

struct CudaLmState {
  double lambda = 0.0;
  double currentFactor = 0.0;
  size_t acceptedIterations = 0;
  size_t totalInnerAttempts = 0;
};

}  // namespace

struct CudaSparseLevenbergMarquardtOptimizer::Impl {
  std::shared_ptr<const NonlinearFactorGraph> graph;
  Values initialValues;
  Values currentValues;
  CudaSparseLevenbergMarquardtParams parameters;
  CudaSparseLevenbergMarquardtResult optimizationResult;
  double currentError = std::numeric_limits<double>::quiet_NaN();
  bool optimized = false;

  // Destruction is in reverse declaration order. In particular, device is
  // destroyed (and synchronizes its fixed stream) before the pinned host
  // buffer and before the stream-owning context.
  std::unique_ptr<CudaContext> context;
  std::unique_ptr<SparseJacobianColumnLayout> layout;
  std::unique_ptr<SparseJacobianPlan> plan;
  std::unique_ptr<HostSparseJacobian> host;
  std::unique_ptr<StreamingSparseJacobianLinearizer> linearizer;
  std::unique_ptr<DeviceSparseJacobianNormalEquations> device;

  Impl(const NonlinearFactorGraph& inputGraph, const Values& inputValues,
       const CudaSparseLevenbergMarquardtParams& inputParameters)
      : graph(inputGraph.cloneShared()),
        initialValues(inputValues),
        currentValues(inputValues),
        parameters(inputParameters) {
    if (!graph) {
      throw std::invalid_argument(
          "CudaSparseLevenbergMarquardtOptimizer graph clone is null");
    }
  }

  void clearCudaState() {
    device.reset();
    linearizer.reset();
    host.reset();
    plan.reset();
    layout.reset();
    context.reset();
  }

  void updateFinalState(const CudaLmState& state) {
    optimizationResult.iterations = state.acceptedIterations;
    optimizationResult.acceptedSteps = state.acceptedIterations;
    optimizationResult.lambdaAttempts = state.totalInnerAttempts;
    optimizationResult.finalError = currentError;
    optimizationResult.finalLambda = state.lambda;
    optimizationResult.cudssAnalyses =
        device ? device->analysisCount() : 0;
  }

  const Values& finish(CudaSparseLmTerminationReason reason,
                       const CudaLmState& state) {
    optimizationResult.termination = reason;
    updateFinalState(state);
    optimized = true;
    return currentValues;
  }

  const Values& runCpuFallback(CudaSparseLmFallbackReason reason,
                               const DirectJacobianStatus& status,
                               const std::string& detail,
                               const CudaLmState& state) {
    if (!parameters.fallbackOnUnsupported) {
      throw std::runtime_error(detail);
    }

    updateFinalState(state);
    clearCudaState();
    LevenbergMarquardtParams cpuParameters = parameters;
    LevenbergMarquardtOptimizer cpu(*graph, initialValues, cpuParameters);
    currentValues = cpu.optimize();
    currentError = graph->error(currentValues);
    optimizationResult.backend = CudaSparseLmBackend::CpuFallback;
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

    clearCudaState();
    currentValues = initialValues;
    optimizationResult = {};
    optimizationResult.finalLambda = parameters.lambdaInitial;

    currentError = graph->error(currentValues);
    RequireFiniteInitialError(currentError);
    optimizationResult.initialError = currentError;
    optimizationResult.finalError = currentError;

    CudaLmState state{parameters.lambdaInitial, parameters.lambdaFactor};
    if (currentError <= parameters.errorTol) {
      return finish(CudaSparseLmTerminationReason::ErrorThreshold, state);
    }
    if (parameters.maxIterations == 0) {
      return finish(CudaSparseLmTerminationReason::MaxIterations, state);
    }

    int deviceCount = 0;
    const cudaError_t discoveryStatus = cudaGetDeviceCount(&deviceCount);
    if (discoveryStatus != cudaSuccess) {
      const std::string detail = CudaDiscoveryDetail(discoveryStatus);
      // Do not leave a discovery error pending for the CPU fallback.
      (void)cudaGetLastError();
      return runCpuFallback(CudaSparseLmFallbackReason::CudaUnavailable, {},
                            detail, state);
    }
    if (deviceCount <= 0) {
      return runCpuFallback(CudaSparseLmFallbackReason::CudaUnavailable, {},
                            "no CUDA device is available", state);
    }

#if !GTSAM_ENABLE_CUDSS
    return runCpuFallback(CudaSparseLmFallbackReason::CudssUnavailable, {},
                          "cuDSS support is not compiled", state);
#else
    const DeviceSparseNormalEquationCapability capability =
        DeviceSparseJacobianNormalEquations::preflightCapability();
    if (!capability.supported) {
      return runCpuFallback(
          CudaSparseLmFallbackReason::CudaToolkitUnsupported, {},
          capability.detail, state);
    }
#endif

    const Clock::time_point planStart = Clock::now();
    try {
      layout =
          std::make_unique<SparseJacobianColumnLayout>(initialValues);
      plan = std::make_unique<SparseJacobianPlan>(*graph, *layout);
      if (plan->rows() <= 0 || plan->columns() <= 0 ||
          plan->nonzeros() <= 0) {
        throw std::invalid_argument(
            "direct sparse Jacobian plan must have positive dimensions");
      }
    } catch (const std::invalid_argument& error) {
      return runCpuFallback(CudaSparseLmFallbackReason::PlanIncompatible, {},
                            std::string("sparse Jacobian plan is incompatible: ") +
                                error.what(),
                            state);
    }
    if (parameters.collectTiming) {
      optimizationResult.timings.plan += SecondsSince(planStart);
    }

    RunCudaStage("persistent setup", [&] {
      context = std::make_unique<CudaContext>();
      host = std::make_unique<HostSparseJacobian>(*plan);
      linearizer =
          std::make_unique<StreamingSparseJacobianLinearizer>();
      device =
          std::make_unique<DeviceSparseJacobianNormalEquations>();
      device->initialize(*plan, context->stream());
    });

    while (true) {
      const double previousError = currentError;
      const size_t acceptedBefore = state.acceptedIterations;
      ++optimizationResult.outerLinearizations;

      Clock::time_point stageStart = Clock::now();
      host->clear();
      if (parameters.collectTiming) {
        optimizationResult.timings.hostZero += SecondsSince(stageStart);
      }

      stageStart = Clock::now();
      const DirectJacobianStatus linearizationStatus = linearizer->linearize(
          *graph, currentValues, *layout, *plan, host.get(), nullptr,
          parameters.validateStructureEveryIteration);
      if (parameters.collectTiming) {
        optimizationResult.timings.factorLinearizationAndPackingWall +=
            SecondsSince(stageStart);
      }
      if (!linearizationStatus.ok()) {
        const std::string detail =
            DirectFailureDetail(*graph, linearizationStatus);
        if (linearizationStatus.failure ==
            DirectJacobianFailure::NonFiniteValues) {
          throw std::runtime_error(
              "CUDA sparse LM failure during factor linearization: " +
              detail);
        }
        const bool globalStructuralMismatch =
            linearizationStatus.failure ==
                DirectJacobianFailure::StructuralMismatch &&
            linearizationStatus.factorIndex ==
                std::numeric_limits<size_t>::max();
        return runCpuFallback(
            globalStructuralMismatch
                ? CudaSparseLmFallbackReason::PlanIncompatible
                : CudaSparseLmFallbackReason::DirectJacobianUnsupported,
            linearizationStatus, detail, state);
      }

      stageStart = Clock::now();
      RunCudaStage("numeric upload", [&] {
        device->uploadNumerics(*host, context->stream());
      });
      if (parameters.collectTiming) {
        optimizationResult.timings.upload += SecondsSince(stageStart);
      }

      stageStart = Clock::now();
      RunCudaStage("normal-equation formation", [&] {
        device->formUndampedSystem(context->stream());
      });
      if (parameters.collectTiming) {
        optimizationResult.timings.normalEquations +=
            SecondsSince(stageStart);
      }

      stageStart = Clock::now();
      RunCudaStage("damping preparation", [&] {
        device->prepareDamping(parameters.dampingParams.diagonalDamping,
                               parameters.dampingParams.minDiagonal,
                               parameters.dampingParams.maxDiagonal,
                               context->stream());
      });
      if (parameters.collectTiming) {
        optimizationResult.timings.damping += SecondsSince(stageStart);
      }

      stageStart = Clock::now();
      RunCudaStage("cuDSS analysis", [&] {
        device->analyze(context->stream());
      });
      if (parameters.collectTiming) {
        optimizationResult.timings.cudssAnalysis +=
            SecondsSince(stageStart);
      }

      bool accepted = false;
      CudaSparseLmTerminationReason innerTermination =
          CudaSparseLmTerminationReason::None;
      size_t attemptWithinOuter = 0;
      while (true) {
        CudaSparseLmAttemptRecord attemptRecord;
        attemptRecord.acceptedIterationsBeforeAttempt = acceptedBefore;
        attemptRecord.attempt = attemptWithinOuter++;
        attemptRecord.lambda = state.lambda;

        stageStart = Clock::now();
        RunCudaStage("cuDSS factorization and solve", [&] {
          device->solveAndEvaluate(state.lambda, context->stream());
        });
        if (parameters.collectTiming) {
          optimizationResult.timings.cudssFactorAndSolve +=
              SecondsSince(stageStart);
        }

        stageStart = Clock::now();
        DeviceSparseJacobianAttemptResult deviceAttempt =
            RunCudaStage("attempt result download", [&] {
              return device->downloadAttemptResult(context->stream());
            });
        if (parameters.collectTiming) {
          optimizationResult.timings.deltaDownload +=
              SecondsSince(stageStart);
        }
        ++state.totalInnerAttempts;

        RequireFiniteAttempt(deviceAttempt);
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
              layout->toVectorValues(deviceAttempt.delta);
          stageStart = Clock::now();
          trialValues = currentValues.retract(delta);
          if (parameters.collectTiming) {
            optimizationResult.timings.retract += SecondsSince(stageStart);
          }

          stageStart = Clock::now();
          trialError = graph->error(trialValues);
          if (parameters.collectTiming) {
            optimizationResult.timings.nonlinearTrialError +=
                SecondsSince(stageStart);
          }
          if (!std::isfinite(trialError)) {
            throw std::runtime_error(
                "CUDA sparse LM failure during nonlinear trial error: "
                "non-finite error");
          }

          costChange = currentError - trialError;
          if (linearizedChange >
              std::numeric_limits<double>::epsilon() *
                  oldLinearizedError) {
            modelFidelity = costChange / linearizedChange;
            if (!std::isfinite(modelFidelity)) {
              throw std::runtime_error(
                  "CUDA sparse LM model fidelity is non-finite");
            }
            stepSuccessful =
                modelFidelity > parameters.minModelFidelity;
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
          if (parameters.useFixedLambdaFactor) {
            state.lambda /= state.currentFactor;
          } else {
            state.lambda *= std::max(
                1.0 / 3.0,
                1.0 - std::pow(2.0 * modelFidelity - 1.0, 3));
            state.currentFactor *= 2.0;
          }
          state.lambda =
              std::max(parameters.lambdaLowerBound, state.lambda);
          accepted = true;
        } else if (stopSearchingLambda) {
          innerTermination =
              CudaSparseLmTerminationReason::SmallCostChange;
        } else {
          state.lambda *= state.currentFactor;
          if (!parameters.useFixedLambdaFactor) {
            state.currentFactor *= 2.0;
          }
          if (state.lambda >= parameters.lambdaUpperBound) {
            innerTermination =
                CudaSparseLmTerminationReason::LambdaUpperBound;
          }
        }

        if (parameters.collectAttemptTrace) {
          optimizationResult.attemptTrace.push_back(attemptRecord);
        }
        if (accepted ||
            innerTermination != CudaSparseLmTerminationReason::None) {
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
        return finish(CudaSparseLmTerminationReason::MaxIterations, state);
      }
      if (checkConvergence(parameters, previousError, currentError)) {
        return finish(CudaSparseLmTerminationReason::Converged, state);
      }
    }
  }
};

CudaSparseLevenbergMarquardtOptimizer::
    CudaSparseLevenbergMarquardtOptimizer(
        const NonlinearFactorGraph& graph, const Values& initialValues,
        const CudaSparseLevenbergMarquardtParams& params)
    : impl_(std::make_unique<Impl>(graph, initialValues, params)) {}

CudaSparseLevenbergMarquardtOptimizer::
    ~CudaSparseLevenbergMarquardtOptimizer() = default;

const Values& CudaSparseLevenbergMarquardtOptimizer::optimize() {
  return impl_->optimize();
}

const Values& CudaSparseLevenbergMarquardtOptimizer::values() const {
  return impl_->currentValues;
}

double CudaSparseLevenbergMarquardtOptimizer::error() const {
  return impl_->currentError;
}

const CudaSparseLevenbergMarquardtParams&
CudaSparseLevenbergMarquardtOptimizer::params() const {
  return impl_->parameters;
}

const CudaSparseLevenbergMarquardtResult&
CudaSparseLevenbergMarquardtOptimizer::result() const {
  return impl_->optimizationResult;
}

}  // namespace gtsam::cuda
