/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file PoseSLAMSDPComparison.cpp
 * @brief Compare direct and QCQP SDP construction on ICRA ring/chain graphs.
 */

#include <gtsam/certifiable/LiftedSDPProblem.h>
#include <gtsam/certifiable/internal/DirectSDP.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

using namespace gtsam;
using Clock = std::chrono::steady_clock;

struct Options {
  std::string problem = "se3_ring";
  std::string solver = "chordal_direct";
  size_t N = 20;
  double odometrySigma = 0.01;
  double sampleSigma = 0.01;
  double absoluteSigma = 0.1;
  double maxTime = 2000.0;
  double solverTolerance = 1e-10;
  uint32_t measurementSeed = 42;
  uint32_t absoluteSeed = 314;
  uint32_t randomSeed = 42;
  ChordalOrderingType ordering = ChordalOrderingType::Metis;
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string name(argv[i]);
    if (i + 1 == argc) throw std::invalid_argument("Missing value for " + name);
    const std::string value(argv[++i]);
    if (name == "--problem") options.problem = value;
    else if (name == "--solver") options.solver = value;
    else if (name == "--N") options.N = std::stoul(value);
    else if (name == "--odometry-noise") options.odometrySigma = std::stod(value);
    else if (name == "--sample-noise") options.sampleSigma = std::stod(value);
    else if (name == "--absolute-measurement-noise") options.absoluteSigma = std::stod(value);
    else if (name == "--measurement-seed") options.measurementSeed = std::stoul(value);
    else if (name == "--absolute-seed") options.absoluteSeed = std::stoul(value);
    else if (name == "--random-seed") options.randomSeed = std::stoul(value);
    else if (name == "--max-time") options.maxTime = std::stod(value);
    else if (name == "--solver-tolerance") options.solverTolerance = std::stod(value);
    else if (name == "--ordering" && value == "metis") options.ordering = ChordalOrderingType::Metis;
    else if (name == "--ordering" && value == "colamd") options.ordering = ChordalOrderingType::Colamd;
    else throw std::invalid_argument("Unknown option or value: " + name + " " + value);
  }
  const std::vector<std::string> solvers{
      "chordal_qcqp", "monolithic_qcqp", "chordal_direct", "monolithic_direct",
      "local_ground_truth", "local_random"};
  if (std::find(solvers.begin(), solvers.end(), options.solver) == solvers.end()) {
    throw std::invalid_argument("Unknown solver " + options.solver);
  }
  if (options.N < 3 || !(options.odometrySigma > 0) ||
      !(options.absoluteSigma > 0) || !(options.sampleSigma >= 0) ||
      !(options.maxTime > 0) || !std::isfinite(options.odometrySigma) ||
      !std::isfinite(options.absoluteSigma) || !std::isfinite(options.sampleSigma) ||
      !std::isfinite(options.maxTime) || !(options.solverTolerance > 0) ||
      !std::isfinite(options.solverTolerance)) {
    throw std::invalid_argument("Require N >= 3, positive weights/time, and nonnegative sample noise");
  }
  return options;
}

double secondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

template <typename T>
struct ProblemData {
  NonlinearFactorGraph graph;
  std::vector<T> groundTruth;
  std::string fingerprint;
};

template <typename T>
ProblemData<T> makeProblem(const Options& options) {
  constexpr int N = T::LieAlgebra::RowsAtCompileTime;
  constexpr double pi = 3.14159265358979323846;
  const double radius = 0.2 / (2.0 * std::sin(pi / 100.0));
  const double angle = 2.0 * pi / options.N;
  const double length = 2.0 * radius * std::sin(angle / 2.0);
  T step;
  if constexpr (std::is_same_v<T, Pose2>) step = Pose2(length, 0.0, angle);
  else step = Pose3(Rot3::Rz(angle), Point3(length, 0.0, 0.0));
  ProblemData<T> data;
  data.groundTruth.emplace_back();
  for (size_t i = 1; i < options.N; ++i) data.groundTruth.push_back(data.groundTruth.back().compose(step));
  data.graph.template emplace_shared<FrobeniusPrior<T>>(
      0, data.groundTruth[0].matrix(), noiseModel::Constrained::All(N * N));
  const auto odometryNoise = noiseModel::Isotropic::Sigma(T::dimension, options.odometrySigma);
  const auto samplingNoise = noiseModel::Isotropic::Sigma(T::dimension, options.sampleSigma);
  Sampler odometrySampler(samplingNoise, options.measurementSeed);
  std::ostringstream input;
  input << std::setprecision(17) << options.problem << ' ' << options.N << ' '
        << options.odometrySigma;
  if constexpr (std::is_same_v<T, Pose2>) {
    const auto absoluteNoise = noiseModel::Isotropic::Sigma(T::dimension, options.absoluteSigma);
    Sampler absoluteSampler(absoluteNoise, options.absoluteSeed);
    input << ' ' << options.absoluteSigma;
    for (size_t i = 0; i < options.N; ++i) {
      const T measured = data.groundTruth[i].retract(absoluteSampler.sample());
      data.graph.template emplace_shared<FrobeniusPrior<T>>(i, measured.matrix(), absoluteNoise);
      input << ' ' << i << ' ' << measured.matrix();
    }
  }
  const size_t edges = std::is_same_v<T, Pose2> ? options.N - 1 : options.N;
  for (size_t i = 0; i < edges; ++i) {
    const size_t j = (i + 1) % options.N;
    const T exact = data.groundTruth[i].between(data.groundTruth[j]);
    const T measured = exact.retract(odometrySampler.sample());
    data.graph.template emplace_shared<FrobeniusBetweenFactor<T>>(i, j, measured, odometryNoise);
    input << ' ' << i << ' ' << j << ' ' << measured.matrix();
  }
  uint64_t hash = 14695981039346656037ull;
  for (unsigned char byte : input.str()) hash = (hash ^ byte) * 1099511628211ull;
  std::ostringstream fingerprint;
  fingerprint << std::hex << hash;
  data.fingerprint = fingerprint.str();
  return data;
}

template <typename T>
Values initialValues(const ProblemData<T>& data, const Options& options) {
  std::mt19937 generator(options.randomSeed);
  std::uniform_real_distribution<double> translation(-2.0, 2.0);
  std::uniform_real_distribution<double> angle(-3.14159265358979323846, 3.14159265358979323846);
  Values values;
  for (size_t i = 0; i < options.N; ++i) {
    T pose = data.groundTruth[i];
    if (options.solver == "local_random") {
      if constexpr (std::is_same_v<T, Pose2>) {
        pose = Pose2(Rot2::fromAngle(angle(generator)),
                     Point2(translation(generator), translation(generator)));
      } else {
        pose = Pose3(Rot3::Random(generator),
                     Point3(translation(generator), translation(generator), translation(generator)));
      }
    }
    values.insert(i, pose);
  }
  // Initialize the exactly known pose at its anchor for both local starts;
  // all random draws are still consumed identically to the ICRA initializer.
  values.update(0, data.groundTruth[0]);
  return values;
}

struct Result {
  Values values;
  double construction = 0.0, solve = 0.0, recovery = 0.0, total = 0.0;
  double objective = std::numeric_limits<double>::quiet_NaN();
  double dualObjective = std::numeric_limits<double>::quiet_NaN();
  double constraintResidual = 0.0, homogenizationResidual = 0.0;
  std::string status = "local", solutionStatus = "local";
  std::vector<double> eigenRatios, rankResiduals, minimumEigenvalues;
  std::map<std::string, double> primalResiduals;
  size_t rankOneCount = 0;
  size_t historicalRankOneCount = 0;
};

template <typename T, typename SDP>
void solveSDP(SDP& problem, const ProblemData<T>& data, const Options& options,
              Result* result) {
  const std::map<std::string, double> parameters{
      {"optimizerMaxTime", options.maxTime},
      {"intpntCoTolRelGap", options.solverTolerance},
      {"intpntCoTolPfeas", options.solverTolerance},
      {"intpntCoTolDfeas", options.solverTolerance},
      {"intpntCoTolInfeas", options.solverTolerance}};
  if (!problem.solve(parameters)) throw std::runtime_error("SDP solve failed");
  result->solve = problem.solveTimeSeconds();
  result->objective = problem.objectiveValue();
  result->dualObjective = problem.dualObjectiveValue();
  result->status = problem.problemStatus();
  result->solutionStatus = problem.solutionStatus();
  result->primalResiduals = problem.primalResiduals();
  const auto start = Clock::now();
  for (const auto& [key, pose] : ExtractQcqpValues<T, 1>(problem.qcqpValues())) {
    result->values.insert(key, pose);
  }
  const auto coefficients = internal::buildDirectSDP(data.graph);
  std::map<Key, Matrix> diagonal;
  for (Key key : problem.orderedKeys()) {
    const Matrix gram = problem.liftedBlock(key, key);
    diagonal.emplace(key, gram);
    Eigen::SelfAdjointEigenSolver<Matrix> spectrum(gram);
    if (spectrum.info() != Eigen::Success) throw std::runtime_error("Gram eigendecomposition failed");
    const auto eigenvalues = spectrum.eigenvalues();
    const double largest = eigenvalues.tail(1)(0);
    const double second = eigenvalues(eigenvalues.size() - 2);
    const double ratio = largest / second;
    const double residual = eigenvalues.head(eigenvalues.size() - 1).cwiseAbs().maxCoeff() /
                            std::max(1.0, std::abs(largest));
    result->eigenRatios.push_back(ratio);
    if (ratio >= 1e5) ++result->historicalRankOneCount;
    result->rankResiduals.push_back(residual);
    result->minimumEigenvalues.push_back(eigenvalues(0));
    if (residual <= 1e-5 && largest > 0) ++result->rankOneCount;
    result->homogenizationResidual = std::max(result->homogenizationResidual, std::abs(gram(0, 0) - 1.0));
  }
  for (const auto& constraint : coefficients.constraints) {
    const double residual = std::abs(constraint.matrix.cwiseProduct(diagonal.at(constraint.key)).sum() - constraint.rhs);
    result->constraintResidual = std::max(result->constraintResidual, residual);
  }
  for (const auto& [key, target] : coefficients.anchors) {
    result->constraintResidual = std::max(result->constraintResidual,
        (diagonal.at(key) - target*target.transpose()).cwiseAbs().maxCoeff());
  }
  for (const auto& factor : data.graph) {
    if (factor->keys().size() == 2) {
      const Matrix block = problem.liftedBlock(factor->keys()[0], factor->keys()[1]);
      result->homogenizationResidual = std::max(result->homogenizationResidual, std::abs(block(0, 0) - 1.0));
    }
  }
  result->recovery = secondsSince(start);
}

void number(double value) {
  if (std::isfinite(value)) std::cout << value;
  else std::cout << "null";
}

void numbers(const std::vector<double>& values) {
  std::cout << '[';
  for (size_t i = 0; i < values.size(); ++i) {
    if (i) std::cout << ',';
    number(values[i]);
  }
  std::cout << ']';
}

template <typename T>
int run(const Options& options) {
  const auto data = makeProblem<T>(options);
  Result result;
  const auto start = Clock::now();
  if (options.solver == "local_random" || options.solver == "local_ground_truth") {
    const Values initial = initialValues(data, options);
    result.construction = secondsSince(start);
    LevenbergMarquardtParams parameters;
    const auto solveStart = Clock::now();
    LevenbergMarquardtOptimizer optimizer(data.graph, initial, parameters);
    result.values = optimizer.optimize();
    result.solve = secondsSince(solveStart);
  } else if (options.solver == "monolithic_direct") {
    MosekMonolithicSDP problem(data.graph);
    result.construction = secondsSince(start);
    solveSDP(problem, data, options, &result);
  } else if (options.solver == "chordal_direct") {
    MosekChordalSDP problem(data.graph, options.ordering);
    result.construction = secondsSince(start);
    solveSDP(problem, data, options, &result);
  } else {
    const QcqpProblem qcqp(data.graph);
    if (options.solver == "monolithic_qcqp") {
      MosekMonolithicSDP problem(qcqp);
      result.construction = secondsSince(start);
      solveSDP(problem, data, options, &result);
    } else {
      MosekChordalSDP problem(qcqp, options.ordering);
      result.construction = secondsSince(start);
      solveSDP(problem, data, options, &result);
    }
  }
  result.total = secondsSince(start);
  double nonlinearObjective = 0.0;
  for (size_t i = 1; i < data.graph.size(); ++i) nonlinearObjective += data.graph[i]->error(result.values);
  std::vector<double> poseErrors;
  double average = 0.0;
  for (size_t i = 0; i < options.N; ++i) {
    const double error = data.groundTruth[i].localCoordinates(result.values.at<T>(i)).norm();
    poseErrors.push_back(error);
    average += error / options.N;
  }
  const double anchorResidual = (result.values.at<T>(0).matrix() - data.groundTruth[0].matrix()).norm();
  if (anchorResidual > 1e-6) throw std::runtime_error("Recovered solution violates exact root anchor");
  std::cout << std::setprecision(17) << "BENCHMARK_RESULT {\"problem\":\"" << options.problem
            << "\",\"solver\":\"" << options.solver << "\",\"N\":" << options.N
            << ",\"fingerprint\":\"" << data.fingerprint << "\",\"problem_status\":\"" << result.status
            << "\",\"solution_status\":\"" << result.solutionStatus << "\",\"objective_value\":";
  number(result.objective);
  std::cout << ",\"dual_objective_value\":"; number(result.dualObjective);
  std::cout << ",\"nonlinear_objective\":"; number(nonlinearObjective);
  std::cout << ",\"average_pose_error_norm\":"; number(average);
  std::cout << ",\"anchor_residual\":"; number(anchorResidual);
  std::cout << ",\"constraint_residual\":"; number(result.constraintResidual);
  std::cout << ",\"homogenization_residual\":"; number(result.homogenizationResidual);
  std::cout << ",\"construction_seconds\":" << result.construction
            << ",\"solve_time_seconds\":" << result.solve << ",\"recovery_seconds\":" << result.recovery
            << ",\"total_seconds\":" << result.total << ",\"rank_one_pose_count\":";
  if (result.eigenRatios.empty()) std::cout << "null";
  else std::cout << result.rankOneCount;
  std::cout << ",\"historical_rank_one_pose_count\":";
  if (result.eigenRatios.empty()) std::cout << "null";
  else std::cout << result.historicalRankOneCount;
  std::cout << ",\"pose_error_norms\":"; numbers(poseErrors);
  std::cout << ",\"eigenvalue_ratios\":"; numbers(result.eigenRatios);
  std::cout << ",\"rank_residuals\":"; numbers(result.rankResiduals);
  std::cout << ",\"minimum_eigenvalues\":"; numbers(result.minimumEigenvalues);
  for (const auto& [name, residual] : result.primalResiduals) {
    std::cout << ",\"solver_residual_" << name << "\":";
    number(residual);
  }
  std::cout << "}" << std::endl;
  return 0;
}

int main(int argc, char** argv) {
  try {
    const auto options = parseOptions(argc, argv);
    if (options.problem == "se2_chain") return run<Pose2>(options);
    if (options.problem == "se3_ring") return run<Pose3>(options);
    throw std::invalid_argument("Unknown problem " + options.problem);
  } catch (const std::exception& error) {
    std::cerr << "BENCHMARK_ERROR " << error.what() << std::endl;
    return 1;
  }
}
