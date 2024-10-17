/**
 * @file   testNonlinearConjugateGradientOptimizer.cpp
 * @brief  Test nonlinear CG optimizer
 * @author Yong-Dian Jian
 * @author Varun Agrawal
 * @date   June 11, 2012
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

// Generate a small PoseSLAM problem
std::tuple<NonlinearFactorGraph, Values> generateProblem() {
  // 1. Create graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add Gaussian prior
  Pose2 priorMean(0.0, 0.0, 0.0);  // prior at origin
  SharedDiagonal priorNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, priorMean, priorNoise);

  // 2b. Add odometry factors
  SharedDiagonal odometryNoise =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, Pose2(2.0, 0.0, 0.0),
                                             odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, Pose2(2.0, 0.0, M_PI_2),
                                             odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(3, 4, Pose2(2.0, 0.0, M_PI_2),
                                             odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2>>(4, 5, Pose2(2.0, 0.0, M_PI_2),
                                             odometryNoise);

  // 2c. Add pose constraint
  SharedDiagonal constraintUncertainty =
      noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  graph.emplace_shared<BetweenFactor<Pose2>>(5, 2, Pose2(2.0, 0.0, M_PI_2),
                                             constraintUncertainty);

  // 3. Create the data structure to hold the initialEstimate estimate to the
  // solution
  Values initialEstimate;
  Pose2 x1(0.5, 0.0, 0.2);
  initialEstimate.insert(1, x1);
  Pose2 x2(2.3, 0.1, -0.2);
  initialEstimate.insert(2, x2);
  Pose2 x3(4.1, 0.1, M_PI_2);
  initialEstimate.insert(3, x3);
  Pose2 x4(4.0, 2.0, M_PI);
  initialEstimate.insert(4, x4);
  Pose2 x5(2.1, 2.1, -M_PI_2);
  initialEstimate.insert(5, x5);

  return {graph, initialEstimate};
}

/* ************************************************************************* */
TEST(NonlinearConjugateGradientOptimizer, Optimize) {
  const auto [graph, initialEstimate] = generateProblem();
  //  cout << "initial error = " << graph.error(initialEstimate) << endl;

  NonlinearOptimizerParams param;
  param.maxIterations =
      500; /* requires a larger number of iterations to converge */
  param.verbosity = NonlinearOptimizerParams::SILENT;

  NonlinearConjugateGradientOptimizer optimizer(graph, initialEstimate, param);
  Values result = optimizer.optimize();
  //  cout << "cg final error = " << graph.error(result) << endl;

  EXPECT_DOUBLES_EQUAL(0.0, graph.error(result), 1e-4);
}

namespace rosenbrock {

/// Alias for the first term in the Rosenbrock function
// using Rosenbrock1Factor = PriorFactor<double>;

using symbol_shorthand::X;
using symbol_shorthand::Y;

constexpr double sqrt_2 = 1.4142135623730951;

class Rosenbrock1Factor : public NoiseModelFactorN<double> {
 private:
  typedef Rosenbrock1Factor This;
  typedef NoiseModelFactorN<double> Base;

  double a_;

 public:
  /** Constructor: key is x */
  Rosenbrock1Factor(Key key, double a, const SharedNoiseModel& model = nullptr)
      : Base(model, key), a_(a) {}

  /// evaluate error
  Vector evaluateError(const double& x, OptionalMatrixType H) const override {
    double d = x - a_;
    // Because linearized gradient is -A'b, it will multiply by d
    if (H) (*H) = Vector1(2 / sqrt_2).transpose();
    return Vector1(sqrt_2 * d);
  }
};

/**
 * @brief Factor for the second term of the Rosenbrock function.
 * f2 = (y - x*x)
 *
 * We use the noise model to premultiply with `b`.
 */
class Rosenbrock2Factor : public NoiseModelFactorN<double, double> {
 private:
  typedef Rosenbrock2Factor This;
  typedef NoiseModelFactorN<double, double> Base;

 public:
  /** Constructor: key1 is x, key2 is y */
  Rosenbrock2Factor(Key key1, Key key2, const SharedNoiseModel& model = nullptr)
      : Base(model, key1, key2) {}

  /// evaluate error
  Vector evaluateError(const double& x, const double& y, OptionalMatrixType H1,
                       OptionalMatrixType H2) const override {
    double x2 = x * x, d = x2 - y;
    // Because linearized gradient is -A'b, it will multiply by d
    if (H1) (*H1) = Vector1(4 * x / sqrt_2).transpose();
    if (H2) (*H2) = Vector1(-2 / sqrt_2).transpose();
    return Vector1(sqrt_2 * d);
  }
};

/**
 * @brief Get a nonlinear factor graph representing
 * the Rosenbrock Banana function.
 *
 * More details: https://en.wikipedia.org/wiki/Rosenbrock_function
 *
 * @param a
 * @param b
 * @return NonlinearFactorGraph
 */
static NonlinearFactorGraph GetRosenbrockGraph(double a = 1.0,
                                               double b = 100.0) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<Rosenbrock1Factor>(X(0), a, noiseModel::Unit::Create(1));
  graph.emplace_shared<Rosenbrock2Factor>(
      X(0), Y(0), noiseModel::Isotropic::Precision(1, b));

  return graph;
}

/// Compute the Rosenbrock function error given the nonlinear factor graph
/// and input values.
double f(const NonlinearFactorGraph& graph, double x, double y) {
  Values initial;
  initial.insert<double>(X(0), x);
  initial.insert<double>(Y(0), y);

  return graph.error(initial);
}

/// True Rosenbrock Banana function.
double rosenbrock_func(double x, double y, double a = 1.0, double b = 100.0) {
  double m = (a - x) * (a - x);
  double n = b * (y - x * x) * (y - x * x);
  return m + n;
}
}  // namespace rosenbrock

/* ************************************************************************* */
// Test whether the 2 factors are properly implemented.
TEST(NonlinearConjugateGradientOptimizer, Rosenbrock) {
  using namespace rosenbrock;
  double a = 1.0, b = 100.0;
  Rosenbrock1Factor f1(X(0), a, noiseModel::Unit::Create(1));
  Rosenbrock2Factor f2(X(0), Y(0), noiseModel::Isotropic::Sigma(1, b));
  Values values;
  values.insert<double>(X(0), 0.0);
  values.insert<double>(Y(0), 0.0);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f1, values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f2, values, 1e-7, 1e-5);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(0.0, 100.0);
  for (size_t i = 0; i < 50; ++i) {
    double x = dist(rng);
    double y = dist(rng);

    auto graph = GetRosenbrockGraph(a, b);
    EXPECT_DOUBLES_EQUAL(rosenbrock_func(x, y, a, b), f(graph, x, y), 1e-5);
  }
}

/* ************************************************************************* */
// Optimize the Rosenbrock function to verify optimizer works
TEST(NonlinearConjugateGradientOptimizer, Optimization) {
  using namespace rosenbrock;

  double a = 12;
  auto graph = GetRosenbrockGraph(a);

  // Assert that error at global minimum is 0.
  double error = f(graph, a, a * a);
  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-12);

  NonlinearOptimizerParams param;
  param.maxIterations = 350;
  // param.verbosity = NonlinearOptimizerParams::LINEAR;
  param.verbosity = NonlinearOptimizerParams::SILENT;

  double x = 3.0, y = 5.0;
  Values initialEstimate;
  initialEstimate.insert<double>(X(0), x);
  initialEstimate.insert<double>(Y(0), y);

  GaussianFactorGraph::shared_ptr linear = graph.linearize(initialEstimate);
  // std::cout << "error: " << f(graph, x, y) << std::endl;
  // linear->print();
  // linear->gradientAtZero().print("Gradient: ");

  NonlinearConjugateGradientOptimizer optimizer(graph, initialEstimate, param);
  Values result = optimizer.optimize();
  // result.print();
  // cout << "cg final error = " << graph.error(result) << endl;

  Values expected;
  expected.insert<double>(X(0), a);
  expected.insert<double>(Y(0), a * a);
  EXPECT(assert_equal(expected, result, 1e-1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
