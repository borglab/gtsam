/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeBatchedDenseLM.cpp
 * @brief   Benchmark BatchedDenseLevenbergMarquardtOptimizer/
 *          BatchedDeviceGradients against DFG-TAMP's window-graph shape
 *          (4 slots x 64 dims = 256 columns, B=8 graphs).
 *
 * With --num-gpus > 1, this measures independent per-device throughput,
 * not an integrated multi-GPU training step: one host thread runs on each
 * device (RunOnDevice), each with its own completely independent
 * BatchedDenseLevenbergMarquardtOptimizer batch and its own
 * BatchedDenseSolverOptions::deviceId. There is no library-level device
 * pool, no distributed batch dispatch, and no cross-GPU result or gradient
 * aggregation anywhere in gtsam/nonlinear/cuda/ or gtsam/linear/cuda/ --
 * those APIs each accept one deviceId and solve a batch for that device
 * only. The "Overall wall-clock"/"Throughput" lines below report the sum
 * of N independent single-GPU jobs run concurrently, which is a real and
 * useful number for capacity planning, but it is not evidence of
 * multi-GPU training orchestration; a caller wanting that must build the
 * per-device dispatch and aggregation themselves, as this file's own
 * --num-gpus path does (PR #2937 review, "multi-GPU orchestration is in
 * the timing executable, not the GTSAM optimizer or gradient API").
 * @date    Nov 20, 2026
 */

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/cuda/internal/BatchedDenseCholeskySolver.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/cuda/BatchedDenseLevenbergMarquardt.h>
#include <gtsam/nonlinear/cuda/BatchedDeviceGradients.h>
#include <gtsam/slam/PriorFactor.h>

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace gtsam;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kNumSlots = 4;
constexpr int kSlotDimension = 64;
constexpr int kTotalDimension = kNumSlots * kSlotDimension;  // 256, matches
                                                              // CLAUDE.md's
                                                              // real window
                                                              // graph shape.
constexpr int kStructuralFactors = 10;
constexpr int kAnchorFactors = kNumSlots;  // 4, one per slot.

/// Coupling factor between two 64-dim slot variables: error(x1,x2) =
/// x1-(x2+0.1*x2^2)-offset. The x2^2 term gives real curvature, unlike a
/// plain (affine) BetweenFactor, so repeated LM iterations do real work.
class NonlinearCouplingFactor : public NoiseModelFactorN<Vector, Vector> {
 public:
  using Base = NoiseModelFactorN<Vector, Vector>;

  NonlinearCouplingFactor(Key key1, Key key2, const Vector& offset,
                          const SharedNoiseModel& model)
      : Base(model, key1, key2), offset_(offset) {}

  Vector evaluateError(const Vector& x1, const Vector& x2,
                       OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    const Vector coupled = x2 + 0.1 * x2.array().square().matrix();
    if (H1) *H1 = Matrix::Identity(x1.size(), x1.size());
    if (H2) {
      *H2 = -(Matrix::Identity(x2.size(), x2.size()) +
             (0.2 * x2.array()).matrix().asDiagonal().toDenseMatrix());
    }
    return x1 - coupled - offset_;
  }

 private:
  Vector offset_;
};

/// Builds one DFG-TAMP-shaped window graph: 4 slot keys x 64 dims, 10
/// structural coupling factors + 4 anchor priors, matching CLAUDE.md's
/// "4 slots x 64-dim = 256 columns, 14 factors: 10 structural + 4 anchor".
NonlinearFactorGraph buildWindowGraph(std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, 1.0);
  NonlinearFactorGraph graph;
  const auto noiseModel = noiseModel::Isotropic::Sigma(kSlotDimension, 0.1);

  // 10 structural factors over 4 slots (reusing slot pairs to reach 10
  // edges among 4 nodes, matching the real transition graph's edge count).
  static const int structuralPairs[kStructuralFactors][2] = {
      {0, 1}, {1, 2}, {2, 3}, {0, 2}, {1, 3},
      {0, 3}, {0, 1}, {1, 2}, {2, 3}, {0, 2},
  };
  for (int f = 0; f < kStructuralFactors; ++f) {
    Vector offset(kSlotDimension);
    for (int d = 0; d < kSlotDimension; ++d) offset(d) = 0.05 * noise(*rng);
    graph.emplace_shared<NonlinearCouplingFactor>(
        Symbol('x', structuralPairs[f][0]), Symbol('x', structuralPairs[f][1]),
        offset, noiseModel);
  }
  // 4 anchor priors, one per slot.
  for (int slot = 0; slot < kNumSlots; ++slot) {
    Vector anchor(kSlotDimension);
    for (int d = 0; d < kSlotDimension; ++d) anchor(d) = noise(*rng);
    graph.emplace_shared<PriorFactor<Vector>>(Symbol('x', slot), anchor,
                                              noiseModel);
  }
  return graph;
}

Values buildInitialValues(std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, 1.0);
  Values values;
  for (int slot = 0; slot < kNumSlots; ++slot) {
    Vector x(kSlotDimension);
    for (int d = 0; d < kSlotDimension; ++d) x(d) = noise(*rng);
    values.insert(Symbol('x', slot), x);
  }
  return values;
}

struct RunOptions {
  int deviceOffset = 0;
  int numGpus = 1;
  int bPerGpu = 8;
  int nTags = 43;
  int maxIterations = 40;
  double targetSeconds = 1200.0;  // 20 minutes, matching the user's 1-GPU
                                  // sizing target.
  int repeats = -1;  // -1 means calibrate; >=0 means run exactly this many.
  std::string outputPath;
  std::string backend = "gpu";  // "gpu" or "cpu"
  bool realTolerances = false;
  bool verify = false;
  int verifyBatches = 1;
};

std::string RequireValue(int argc, char** argv, int* index) {
  if (*index + 1 >= argc) {
    throw std::invalid_argument(std::string("missing value for ") +
                                argv[*index]);
  }
  return std::string(argv[++*index]);
}

RunOptions ParseOptions(int argc, char** argv) {
  RunOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--device-offset") {
      options.deviceOffset = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--num-gpus") {
      options.numGpus = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--b-per-gpu") {
      options.bPerGpu = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--n-tags") {
      options.nTags = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--max-iterations") {
      options.maxIterations = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--target-seconds") {
      options.targetSeconds = std::stod(RequireValue(argc, argv, &index));
    } else if (argument == "--repeats") {
      options.repeats = std::stoi(RequireValue(argc, argv, &index));
    } else if (argument == "--output") {
      options.outputPath = RequireValue(argc, argv, &index);
    } else if (argument == "--backend") {
      options.backend = RequireValue(argc, argv, &index);
      if (options.backend != "gpu" && options.backend != "cpu" &&
          options.backend != "compare") {
        throw std::invalid_argument("--backend must be gpu, cpu, or compare");
      }
    } else if (argument == "--real-tolerances") {
      options.realTolerances = true;
    } else if (argument == "--verify") {
      options.verify = true;
    } else if (argument == "--verify-batches") {
      options.verifyBatches = std::stoi(RequireValue(argc, argv, &index));
    } else {
      throw std::invalid_argument("unknown argument " + argument);
    }
  }
  return options;
}

LevenbergMarquardtParams ForwardParams(int maxIterations, bool realTolerances) {
  LevenbergMarquardtParams params;
  params.maxIterations = static_cast<size_t>(maxIterations);
  if (!realTolerances) {
    // Zeroed so the benchmark always runs exactly maxIterations outer
    // iterations, for a controllable duration; --real-tolerances lets
    // converged items stop early instead.
    params.relativeErrorTol = 0.0;
    params.absoluteErrorTol = 0.0;
    params.errorTol = 0.0;
  }
  return params;
}

/// One forward+backward "training step": forward LM solve for every
/// graph, then nTags backward adjoint queries against the converged batch.
double RunOneStep(int bPerGpu, int nTags, int maxIterations,
                  bool realTolerances,
                  const cuda::BatchedDenseSolverOptions& deviceOptions,
                  std::mt19937* rng) {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> initialValues;
  graphs.reserve(static_cast<size_t>(bPerGpu));
  initialValues.reserve(static_cast<size_t>(bPerGpu));
  for (int b = 0; b < bPerGpu; ++b) {
    graphs.push_back(buildWindowGraph(rng));
    initialValues.push_back(buildInitialValues(rng));
  }

  const auto start = Clock::now();
  cuda::BatchedDenseLevenbergMarquardtOptimizer forward(
      graphs, initialValues, ForwardParams(maxIterations, realTolerances),
      deviceOptions);
  const std::vector<Values>& converged = forward.optimize();
  const double forwardSeconds =
      std::chrono::duration<double>(Clock::now() - start).count();

  const auto backwardStart = Clock::now();
  cuda::BatchedDeviceGradients gradients(graphs, converged, deviceOptions);
  std::normal_distribution<double> noise(0.0, 1.0);
  for (int tag = 0; tag < nTags; ++tag) {
    std::vector<VectorValues> cotangents(static_cast<size_t>(bPerGpu));
    for (int b = 0; b < bPerGpu; ++b) {
      VectorValues cotangent;
      for (int slot = 0; slot < kNumSlots; ++slot) {
        Vector v(kSlotDimension);
        for (int d = 0; d < kSlotDimension; ++d) v(d) = noise(*rng);
        cotangent.insert(Symbol('x', slot), v);
      }
      cotangents[static_cast<size_t>(b)] = cotangent;
    }
    gradients.adjoint(cotangents);
  }
  const double backwardSeconds =
      std::chrono::duration<double>(Clock::now() - backwardStart).count();

  static bool printedDiagnostics = false;
  if (!printedDiagnostics) {
    printedDiagnostics = true;
    size_t totalAttempts = 0, totalAccepted = 0;
    for (const auto& r : forward.results()) {
      totalAttempts += r.lambdaAttempts;
      totalAccepted += r.acceptedIterations;
    }
    const auto& stats = forward.linearSolveStats();
    const auto& hostTimings = forward.hostTimings();
    std::cerr << "[diagnostics] forward=" << forwardSeconds
              << "s backward=" << backwardSeconds
              << "s  totalLambdaAttempts(item-rounds)=" << totalAttempts
              << " totalAcceptedSteps=" << totalAccepted
              << " (maxIterations=" << maxIterations << " x bPerGpu="
              << bPerGpu << ")\n"
              << "[diagnostics] forward solver calls: factorizations="
              << stats.factorizationCount << " solves=" << stats.solveCount
              << " factorizationSeconds=" << stats.factorizationSeconds
              << " solveSeconds=" << stats.solveSeconds << "\n"
              << "[diagnostics] host breakdown: linearizeAndForm="
              << hostTimings.linearizeAndFormSeconds
              << "s marshal=" << hostTimings.marshalSeconds
              << "s retractAndError=" << hostTimings.retractAndErrorSeconds
              << "s gpuRoundTrip=" << hostTimings.gpuRoundTripSeconds
              << "s\n";
  }
  return forwardSeconds + backwardSeconds;
}

/// CPU baseline: ordinary LevenbergMarquardtOptimizer/Gradients, one
/// graph at a time, sharing the GPU path's graphs/params for a fair comparison.
double RunOneStepCpu(int bPerGpu, int nTags, int maxIterations,
                    bool realTolerances, std::mt19937* rng) {
  std::vector<NonlinearFactorGraph> graphs;
  std::vector<Values> initialValues;
  graphs.reserve(static_cast<size_t>(bPerGpu));
  initialValues.reserve(static_cast<size_t>(bPerGpu));
  for (int b = 0; b < bPerGpu; ++b) {
    graphs.push_back(buildWindowGraph(rng));
    initialValues.push_back(buildInitialValues(rng));
  }

  const auto start = Clock::now();
  std::vector<Values> converged(static_cast<size_t>(bPerGpu));
  for (int b = 0; b < bPerGpu; ++b) {
    LevenbergMarquardtOptimizer optimizer(graphs[static_cast<size_t>(b)],
                                          initialValues[static_cast<size_t>(b)],
                                          ForwardParams(maxIterations, realTolerances));
    converged[static_cast<size_t>(b)] = optimizer.optimize();
  }

  std::normal_distribution<double> noise(0.0, 1.0);
  for (int b = 0; b < bPerGpu; ++b) {
    Gradients gradients(graphs[static_cast<size_t>(b)],
                        converged[static_cast<size_t>(b)]);
    for (int tag = 0; tag < nTags; ++tag) {
      VectorValues cotangent;
      for (int slot = 0; slot < kNumSlots; ++slot) {
        Vector v(kSlotDimension);
        for (int d = 0; d < kSlotDimension; ++d) v(d) = noise(*rng);
        cotangent.insert(Symbol('x', slot), v);
      }
      gradients.adjoint(cotangent);
    }
  }
  return std::chrono::duration<double>(Clock::now() - start).count();
}

struct GpuResult {
  int deviceId = 0;
  int repeats = 0;
  double wallSeconds = 0.0;
  double meanStepSeconds = 0.0;
};

/// Runs repeats independent RunOneStep() calls, all pinned to deviceId via
/// BatchedDenseSolverOptions::deviceId, on this host thread. One call to
/// this function per GPU is the entirety of this file's "multi-GPU"
/// support: it is per-device batched solving dispatched by N independent
/// host threads, not a single integrated multi-GPU training step -- see
/// this file's top docstring.
void RunOnDevice(int deviceId, int bPerGpu, int nTags, int maxIterations,
                 bool realTolerances, int repeats, unsigned seed,
                 GpuResult* result) {
  cudaSetDevice(deviceId);
  cuda::BatchedDenseSolverOptions deviceOptions;
  deviceOptions.deviceId = deviceId;
  std::mt19937 rng(seed);

  std::vector<double> stepSeconds;
  stepSeconds.reserve(static_cast<size_t>(repeats));
  const auto wallStart = Clock::now();
  for (int r = 0; r < repeats; ++r) {
    stepSeconds.push_back(RunOneStep(bPerGpu, nTags, maxIterations,
                                     realTolerances, deviceOptions, &rng));
  }
  result->deviceId = deviceId;
  result->repeats = repeats;
  result->wallSeconds =
      std::chrono::duration<double>(Clock::now() - wallStart).count();
  result->meanStepSeconds =
      std::accumulate(stepSeconds.begin(), stepSeconds.end(), 0.0) /
      static_cast<double>(stepSeconds.size());
}

/// Numerical-agreement check over nBatches batches: compares GPU-batched
/// vs CPU final error/Values/iterations and adjoint() output. Not
/// bit-identical by design; close agreement is reported, not asserted.
void RunCorrectnessCheck(int bPerGpu, int maxIterations, bool realTolerances,
                         int deviceId, int nBatches) {
  cuda::BatchedDenseSolverOptions deviceOptions;
  deviceOptions.deviceId = deviceId;

  double maxErrorDiff = 0.0, maxValueDiff = 0.0, maxAdjointDiff = 0.0;
  double sumGpuIterations = 0.0, sumCpuIterations = 0.0;
  size_t totalGraphs = 0;

  for (int batch = 0; batch < nBatches; ++batch) {
    std::mt19937 rng(static_cast<unsigned>(42 + batch));
    std::vector<NonlinearFactorGraph> graphs;
    std::vector<Values> initialValues;
    for (int b = 0; b < bPerGpu; ++b) {
      graphs.push_back(buildWindowGraph(&rng));
      initialValues.push_back(buildInitialValues(&rng));
    }

    cuda::BatchedDenseLevenbergMarquardtOptimizer gpuOptimizer(
        graphs, initialValues, ForwardParams(maxIterations, realTolerances),
        deviceOptions);
    const std::vector<Values>& gpuValues = gpuOptimizer.optimize();
    const auto& gpuResults = gpuOptimizer.results();

    std::vector<Values> cpuValues(static_cast<size_t>(bPerGpu));
    for (int b = 0; b < bPerGpu; ++b) {
      const size_t i = static_cast<size_t>(b);
      LevenbergMarquardtOptimizer cpuOptimizer(
          graphs[i], initialValues[i],
          ForwardParams(maxIterations, realTolerances));
      cpuValues[i] = cpuOptimizer.optimize();

      const double gpuError = graphs[i].error(gpuValues[i]);
      const double cpuError = graphs[i].error(cpuValues[i]);
      const double errorDiff = std::abs(gpuError - cpuError);
      const double valueDiff =
          gpuValues[i].localCoordinates(cpuValues[i]).vector().norm();
      maxErrorDiff = std::max(maxErrorDiff, errorDiff);
      maxValueDiff = std::max(maxValueDiff, valueDiff);
      sumGpuIterations += static_cast<double>(gpuResults[i].acceptedIterations);
      sumCpuIterations += static_cast<double>(cpuOptimizer.iterations());
      ++totalGraphs;

      std::cout << "batch " << batch << " graph " << b << ": gpuError="
                << gpuError << " cpuError=" << cpuError
                << " |errDiff|=" << errorDiff << " |valueDiff|=" << valueDiff
                << " gpuIters=" << gpuResults[i].acceptedIterations
                << " cpuIters=" << cpuOptimizer.iterations() << "\n";

      std::vector<NonlinearFactorGraph> singleGraph{graphs[i]};
      std::vector<Values> singleValue{cpuValues[i]};
      cuda::BatchedDeviceGradients gpuGradients(singleGraph, singleValue,
                                                deviceOptions);
      Gradients cpuGradients(graphs[i], cpuValues[i]);

      std::normal_distribution<double> noise(0.0, 1.0);
      VectorValues cotangent;
      KeyVector orderedKeys;
      for (int slot = 0; slot < kNumSlots; ++slot) {
        Vector v(kSlotDimension);
        for (int d = 0; d < kSlotDimension; ++d) v(d) = noise(rng);
        const Key key = Symbol('x', slot);
        cotangent.insert(key, v);
        orderedKeys.push_back(key);
      }
      const std::vector<VectorValues> gpuAdjoint =
          gpuGradients.adjoint(std::vector<VectorValues>{cotangent});
      const VectorValues cpuAdjoint = cpuGradients.adjoint(cotangent);
      const double adjointDiff = (gpuAdjoint[0].vector(orderedKeys) -
                                 cpuAdjoint.vector(orderedKeys))
                                    .norm();
      maxAdjointDiff = std::max(maxAdjointDiff, adjointDiff);
    }
  }

  std::cout << "\n=== Summary over " << nBatches << " batches x " << bPerGpu
            << " graphs (" << totalGraphs << " total) ===\n"
            << "  max |errorDiff|=" << maxErrorDiff
            << "  max |valueDiff|=" << maxValueDiff
            << "  max |adjointDiff|=" << maxAdjointDiff << "\n"
            << "  mean GPU accepted iterations=" << (sumGpuIterations / totalGraphs)
            << "  mean CPU iterations=" << (sumCpuIterations / totalGraphs) << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const RunOptions options = ParseOptions(argc, argv);

    int availableDevices = 0;
    cudaGetDeviceCount(&availableDevices);
    std::cout << "CUDA devices visible: " << availableDevices << "\n";
    std::cout << "Requested: backend=" << options.backend
              << " device-offset=" << options.deviceOffset
              << " num-gpus=" << options.numGpus
              << " b-per-gpu=" << options.bPerGpu
              << " n-tags=" << options.nTags
              << " max-iterations=" << options.maxIterations << "\n";

    if (options.verify) {
      RunCorrectnessCheck(options.bPerGpu, options.maxIterations,
                          options.realTolerances, options.deviceOffset,
                          options.verifyBatches);
      return 0;
    }

    if (options.backend == "cpu") {
      int repeats = options.repeats;
      std::mt19937 rng(999);
      if (repeats < 0) {
        RunOneStepCpu(options.bPerGpu, options.nTags, options.maxIterations,
                      options.realTolerances, &rng);  // warmup, discarded
        const double calibratedStep =
            RunOneStepCpu(options.bPerGpu, options.nTags,
                          options.maxIterations, options.realTolerances, &rng);
        repeats = std::max(1, static_cast<int>(std::ceil(
                                  options.targetSeconds / calibratedStep)));
        std::cout << "Calibration step: " << calibratedStep
                  << "s -> repeats for ~" << options.targetSeconds
                  << "s target: " << repeats << "\n";
      }

      std::vector<double> stepSeconds;
      stepSeconds.reserve(static_cast<size_t>(repeats));
      const auto wallStart = Clock::now();
      for (int r = 0; r < repeats; ++r) {
        stepSeconds.push_back(RunOneStepCpu(options.bPerGpu, options.nTags,
                                            options.maxIterations,
                                            options.realTolerances, &rng));
      }
      const double wallSeconds =
          std::chrono::duration<double>(Clock::now() - wallStart).count();
      const double meanStep =
          std::accumulate(stepSeconds.begin(), stepSeconds.end(), 0.0) /
          static_cast<double>(stepSeconds.size());

      std::ostringstream report;
      report << std::fixed << std::setprecision(4);
      report << "=== Results (CPU, repeats=" << repeats << ") ===\n";
      report << "  CPU: wall=" << wallSeconds << "s  mean_step=" << meanStep
             << "s  repeats=" << repeats << "\n";
      const double totalGraphsSolved =
          static_cast<double>(options.bPerGpu) * static_cast<double>(repeats);
      report << "Total independent graphs solved: " << totalGraphsSolved
             << "\n";
      report << "Throughput: " << (totalGraphsSolved / wallSeconds)
             << " graphs/second\n";

      std::cout << report.str();
      if (!options.outputPath.empty()) {
        std::ofstream out(options.outputPath);
        out << report.str();
      }
      return 0;
    }

    if (options.backend == "compare") {
      // PR #2937 review: "Suggested regression/benchmark: Report
      // linearization, assembly, transfer, factorization, solve, and
      // total time separately, and compare against both scalar CPU and
      // batched CPU baselines on identical graph batches." Same seed for
      // both rngs -> buildWindowGraph()/buildInitialValues() (called in
      // the same order by RunOneStep and RunOneStepCpu) produce a
      // bit-identical graph batch for each side's timed call.
      std::mt19937 rngGpu(2024);
      std::mt19937 rngCpu(2024);
      cuda::BatchedDenseSolverOptions deviceOptions;
      deviceOptions.deviceId = options.deviceOffset;

      RunOneStep(options.bPerGpu, options.nTags, options.maxIterations,
                options.realTolerances, deviceOptions, &rngGpu);  // warmup
      const double gpuSeconds = RunOneStep(
          options.bPerGpu, options.nTags, options.maxIterations,
          options.realTolerances, deviceOptions, &rngGpu);

      RunOneStepCpu(options.bPerGpu, options.nTags, options.maxIterations,
                   options.realTolerances, &rngCpu);  // warmup
      const double cpuSeconds = RunOneStepCpu(
          options.bPerGpu, options.nTags, options.maxIterations,
          options.realTolerances, &rngCpu);

      std::ostringstream report;
      report << std::fixed << std::setprecision(4);
      report << "=== GPU vs scalar-CPU comparison, identical graph batch "
               << "(seed=2024, b-per-gpu=" << options.bPerGpu << ") ===\n";
      report << "batched GPU total (forward+backward): " << gpuSeconds
             << "s\n";
      report << "scalar CPU total (forward+backward):  " << cpuSeconds
             << "s\n";
      report << "speedup (cpu/gpu): " << (cpuSeconds / gpuSeconds) << "x\n";
      report << "See the preceding stderr \"[diagnostics]\" lines (from "
               << "RunOneStep's own, already-instrumented timed call) for "
               << "the GPU-side phase breakdown: linearizeAndForm, marshal, "
               << "retractAndError, gpuRoundTrip host seconds, plus "
               << "factorizationSeconds/solveSeconds from LinearSolveStats "
               << "(the \"assembly\"/\"transfer\"/\"factorization\"/\"solve\" "
               << "split the review asked for).\n";
      report << "NOTE: this reports a scalar CPU baseline (one graph at a "
               << "time via LevenbergMarquardtOptimizer/Gradients) only. A "
               << "separate batched/vectorized or multi-threaded CPU "
               << "baseline is intentionally not included: an earlier "
               << "version of this benchmark had CPU multithreading and it "
               << "was deliberately removed to keep this PR scoped to GPU "
               << "work only (see this file's git history); adding one back "
               << "would be new non-GPU engineering scope, not a benchmark "
               << "change.\n";

      std::cout << report.str();
      if (!options.outputPath.empty()) {
        std::ofstream out(options.outputPath);
        out << report.str();
      }
      return 0;
    }

    int repeats = options.repeats;
    if (repeats < 0) {
      // Warmup step pays one-time setup outside the measurement, then
      // extrapolate repeat count from one timed step to fill targetSeconds.
      cuda::BatchedDenseSolverOptions calibrationOptions;
      calibrationOptions.deviceId = options.deviceOffset;
      std::mt19937 rng(12345);
      RunOneStep(options.bPerGpu, options.nTags, options.maxIterations,
                options.realTolerances, calibrationOptions,
                &rng);  // warmup, discarded
      const double calibratedStep =
          RunOneStep(options.bPerGpu, options.nTags, options.maxIterations,
                    options.realTolerances, calibrationOptions, &rng);
      repeats = std::max(
          1, static_cast<int>(std::ceil(options.targetSeconds / calibratedStep)));
      std::cout << "Calibration step: " << calibratedStep
                << "s -> repeats for ~" << options.targetSeconds
                << "s target: " << repeats << "\n";
    }

    std::vector<std::thread> threads;
    std::vector<GpuResult> results(static_cast<size_t>(options.numGpus));
    const auto overallStart = Clock::now();
    for (int g = 0; g < options.numGpus; ++g) {
      threads.emplace_back(RunOnDevice, options.deviceOffset + g,
                           options.bPerGpu, options.nTags,
                           options.maxIterations, options.realTolerances,
                           repeats, static_cast<unsigned>(1000 * g + 7),
                           &results[static_cast<size_t>(g)]);
    }
    for (auto& t : threads) t.join();
    const double overallWallSeconds =
        std::chrono::duration<double>(Clock::now() - overallStart).count();

    std::ostringstream report;
    report << std::fixed << std::setprecision(4);
    report << "=== Results (repeats=" << repeats << " per GPU) ===\n";
    for (const auto& r : results) {
      report << "  GPU " << r.deviceId << ": wall=" << r.wallSeconds
             << "s  mean_step=" << r.meanStepSeconds << "s  repeats="
             << r.repeats << "\n";
    }
    report << "Overall wall-clock across all " << options.numGpus
           << " GPU thread(s): " << overallWallSeconds << "s\n";
    const double totalGraphsSolved = static_cast<double>(options.numGpus) *
                                     static_cast<double>(options.bPerGpu) *
                                     static_cast<double>(repeats);
    report << "Total independent graphs solved: " << totalGraphsSolved
           << "\n";
    report << "Throughput: "
           << (totalGraphsSolved / overallWallSeconds)
           << " graphs/second\n";

    std::cout << report.str();
    if (!options.outputPath.empty()) {
      std::ofstream out(options.outputPath);
      out << report.str();
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "timeBatchedDenseLM failed: " << error.what() << "\n";
    return 1;
  }
}
