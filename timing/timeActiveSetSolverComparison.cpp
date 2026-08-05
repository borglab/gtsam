/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timeActiveSetSolverComparison.cpp
 * @brief   Compare sparse and dense active-set QP subproblem solves.
 * @author  Frank Dellaert
 */

#include <gtsam/constrained/ActiveSetSolver.h>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace gtsam;

namespace {

/* ************************************************************************* */
Matrix Matrix1D(double value) { return (Matrix(1, 1) << value).finished(); }

/* ************************************************************************* */
Vector Vector1D(double value) { return (Vector(1) << value).finished(); }

/* ************************************************************************* */
struct BenchmarkProblem {
  QpProblem constrained;
  Values initial;
};

/* ************************************************************************* */
ActiveSetSolverParams::shared_ptr QpParams(
    ActiveSetSolverParams::QpSubproblemSolver subproblemSolver) {
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpSubproblemSolver = subproblemSolver;
  return params;
}

/* ************************************************************************* */
BenchmarkProblem BuildSparseChainQp(size_t variables, size_t activeStride) {
  if (variables < 2) {
    throw std::invalid_argument("variables must be at least two");
  }
  if (activeStride == 0) {
    throw std::invalid_argument("activeStride must be positive");
  }

  BenchmarkProblem problem;
  const Matrix unaryHessian = Matrix1D(1.0);
  const Matrix chainHessian = Matrix1D(0.2);
  const Vector unaryLinearTerm = Vector1D(1.0);
  const Vector zero = Vector1D(0.0);

  for (size_t i = 0; i < variables; ++i) {
    const Key key = i;
    const HessianFactor unary(key, unaryHessian, unaryLinearTerm, 0.0);
    problem.constrained.addCost(unary);
    problem.initial.insert(key, zero);
  }

  for (size_t i = 0; i + 1 < variables; ++i) {
    const HessianFactor between(i, i + 1, chainHessian, -chainHessian, zero,
                                chainHessian, zero, 0.0);
    problem.constrained.addCost(between);
  }

  for (size_t i = 0; i < variables; i += activeStride) {
    problem.constrained.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(i, Matrix1D(1.0), Vector1D(0.0))));
  }

  return problem;
}

/* ************************************************************************* */
BenchmarkProblem BuildQuadrotorAllocationQp() {
  BenchmarkProblem problem;
  constexpr Key rotorThrustKey = 0;
  constexpr double armLength = 0.25;
  constexpr double yawMomentScale = 0.05;

  const Matrix allocation =
      (Matrix(4, 4) << 1.0, 1.0, 1.0, 1.0, armLength, -armLength, armLength,
       -armLength, armLength, armLength, -armLength, -armLength, yawMomentScale,
       -yawMomentScale, -yawMomentScale, yawMomentScale)
          .finished();
  const Vector desiredWrench = (Vector(4) << 3.4, 0.15, 0.10, 0.0).finished();
  problem.constrained.addCost(
      JacobianFactor(rotorThrustKey, allocation, desiredWrench));

  const double sqrtEffortWeight = 1e-3;
  problem.constrained.addCost(
      JacobianFactor(rotorThrustKey, sqrtEffortWeight * Matrix::Identity(4, 4),
                     Vector::Zero(4)));

  const Vector upper = Vector::Ones(4);
  problem.constrained.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(rotorThrustKey, Matrix::Identity(4, 4), upper)));
  problem.constrained.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      rotorThrustKey, -Matrix::Identity(4, 4), Vector::Zero(4))));

  problem.initial.insert(rotorThrustKey, 0.5 * Vector::Ones(4));
  return problem;
}

/* ************************************************************************* */
template <typename FUNCTION>
double TimeSeconds(FUNCTION&& function, size_t repeats) {
  const auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < repeats; ++i) {
    function();
  }
  const auto end = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(end - start).count() /
         static_cast<double>(repeats);
}

/* ************************************************************************* */
size_t CountActiveRows(const std::vector<bool>& activeRows) {
  size_t count = 0;
  for (bool active : activeRows) {
    if (active) {
      ++count;
    }
  }
  return count;
}

/* ************************************************************************* */
struct TimingResult {
  double denseSeconds = 0.0;
  double sparseSeconds = 0.0;
  size_t denseIterations = 0;
  size_t sparseIterations = 0;
  size_t denseActiveRows = 0;
  size_t sparseActiveRows = 0;
};

/* ************************************************************************* */
TimingResult TimeColdSolverConstruction(const BenchmarkProblem& problem,
                                        size_t repeats) {
  const auto denseParams =
      QpParams(ActiveSetSolverParams::QpSubproblemSolver::Dense);
  const auto sparseParams =
      QpParams(ActiveSetSolverParams::QpSubproblemSolver::Sparse);

  const auto denseRun = [&]() {
    return ActiveSetSolver(problem.constrained, denseParams)
        .optimizeWithState(problem.initial);
  };
  const auto sparseRun = [&]() {
    return ActiveSetSolver(problem.constrained, sparseParams)
        .optimizeWithState(problem.initial);
  };

  TimingResult result;
  result.denseSeconds = TimeSeconds(denseRun, repeats);
  result.sparseSeconds = TimeSeconds(sparseRun, repeats);

  const auto denseState = denseRun().second;
  const auto sparseState = sparseRun().second;
  result.denseIterations = denseState.iterations;
  result.sparseIterations = sparseState.iterations;
  result.denseActiveRows = CountActiveRows(denseState.activeInequalityRows);
  result.sparseActiveRows = CountActiveRows(sparseState.activeInequalityRows);
  return result;
}

/* ************************************************************************* */
TimingResult TimeWarmReusedSolver(const BenchmarkProblem& problem,
                                  size_t repeats) {
  const auto denseParams =
      QpParams(ActiveSetSolverParams::QpSubproblemSolver::Dense);
  const auto sparseParams =
      QpParams(ActiveSetSolverParams::QpSubproblemSolver::Sparse);

  const ActiveSetSolver denseSolver(problem.constrained, denseParams);
  const ActiveSetSolver sparseSolver(problem.constrained, sparseParams);
  const auto denseSolved = denseSolver.optimizeWithState(problem.initial);
  const auto sparseSolved = sparseSolver.optimizeWithState(problem.initial);
  const Values& denseInitial = denseSolved.first;
  const ActiveSetSolver::State& denseWarmStart = denseSolved.second;
  const Values& sparseInitial = sparseSolved.first;
  const ActiveSetSolver::State& sparseWarmStart = sparseSolved.second;

  const auto denseRun = [&]() {
    return denseSolver.optimizeWithState(denseInitial, denseWarmStart);
  };
  const auto sparseRun = [&]() {
    return sparseSolver.optimizeWithState(sparseInitial, sparseWarmStart);
  };

  TimingResult result;
  result.denseSeconds = TimeSeconds(denseRun, repeats);
  result.sparseSeconds = TimeSeconds(sparseRun, repeats);

  const auto denseState = denseRun().second;
  const auto sparseState = sparseRun().second;
  result.denseIterations = denseState.iterations;
  result.sparseIterations = sparseState.iterations;
  result.denseActiveRows = CountActiveRows(denseState.activeInequalityRows);
  result.sparseActiveRows = CountActiveRows(sparseState.activeInequalityRows);
  return result;
}

/* ************************************************************************* */
void PrintTimingResult(const std::string& scenario, size_t variables,
                       size_t inequalityRows, size_t repeats,
                       const TimingResult& result) {
  std::cout << scenario << "," << variables << "," << inequalityRows << ","
            << result.denseActiveRows << "," << result.sparseActiveRows << ","
            << repeats << "," << std::setprecision(9) << result.denseSeconds
            << "," << result.sparseSeconds << ","
            << (result.sparseSeconds / result.denseSeconds) << ","
            << result.denseIterations << "," << result.sparseIterations
            << std::endl;
}

/* ************************************************************************* */
std::vector<size_t> ParseSizes(int argc, char** argv) {
  if (argc <= 1) {
    return {32, 64, 128, 256, 512};
  }

  std::vector<size_t> sizes;
  sizes.reserve(static_cast<size_t>(argc - 1));
  for (int i = 1; i < argc; ++i) {
    sizes.push_back(static_cast<size_t>(std::stoul(argv[i])));
  }
  return sizes;
}

/* ************************************************************************* */
size_t Repeats(size_t variables) {
  if (variables <= 64) {
    return 25;
  }
  if (variables <= 256) {
    return 10;
  }
  if (variables <= 512) {
    return 3;
  }
  return 1;
}

}  // namespace

/* ************************************************************************* */
int main(int argc, char** argv) {
  const std::vector<size_t> sizes = ParseSizes(argc, argv);
  constexpr size_t activeStride = 4;

  std::cout << "scenario,variables,inequality_rows,dense_active_rows,"
               "sparse_active_rows,repeats,dense_seconds,sparse_seconds,"
               "sparse_over_dense,dense_iterations,sparse_iterations"
            << std::endl;

  for (const size_t variables : sizes) {
    const BenchmarkProblem problem =
        BuildSparseChainQp(variables, activeStride);

    const size_t repeats = Repeats(variables);
    const TimingResult result = TimeColdSolverConstruction(problem, repeats);
    PrintTimingResult("sparse_chain_cold", variables,
                      (variables + activeStride - 1) / activeStride, repeats,
                      result);
  }

  const BenchmarkProblem quadrotorProblem = BuildQuadrotorAllocationQp();
  constexpr size_t quadrotorVariables = 4;
  constexpr size_t quadrotorInequalityRows = 8;
  constexpr size_t quadrotorRepeats = 10000;
  const TimingResult quadrotorResult =
      TimeWarmReusedSolver(quadrotorProblem, quadrotorRepeats);
  PrintTimingResult("quadrotor_allocation_warm", quadrotorVariables,
                    quadrotorInequalityRows, quadrotorRepeats, quadrotorResult);

  return 0;
}
