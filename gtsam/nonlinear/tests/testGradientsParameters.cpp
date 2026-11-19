/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */

/**
 * @file testGradientsParameters.cpp
 * @brief Parameter VJPs, fixed-noise weighting and optional contributions.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Gradients.h>
#include <gtsam/nonlinear/PriorFactor.h>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <limits>
#include <memory>
#include <stdexcept>

using namespace gtsam;

/* ************************************************************************* */
namespace parameter_fixture {

constexpr Key kState = 42;
const ParameterTag kCalibration{"calibration"};
const ParameterTag kScale{"scale"};
const Vector2 kDirection{.7, -.2}, kSecondDirection{-.3, .8};
const Matrix2 kStateJacobian{{1.3, -.2}, {.4, .9}};
const Matrix2 kParameterJacobian{{.2, .7}, {-.5, 1.1}};
const Matrix2 kCovariance{{.7, .2}, {.2, 1.4}};

// Copies keep a shared identity; equal labels never merge distinct parameters.
TEST(GradientsParameters, TagIdentity) {
  const ParameterTag original{"calibration"};
  const ParameterTag copied = original;
  EXPECT(original == copied);
  EXPECT(original.equals(copied));
  EXPECT_LONGS_EQUAL(original.hash(), copied.hash());
  EXPECT(original != ParameterTag{"calibration"});
  EXPECT(ParameterTag{} != ParameterTag{});
  ParameterTag survivor;
  {
    const ParameterTag temporary{"temporary"};
    survivor = temporary;
    EXPECT(survivor == temporary);
  }
  EXPECT(survivor.name() == "temporary");
  const auto roundTrip = [](ParameterTag tag) { return tag; };
  EXPECT(roundTrip(survivor) == survivor);
}

class CalibrationPrior : public PriorFactor<Vector> {
 public:
  CalibrationPrior(const Vector2& calibration, double scale,
                   const SharedNoiseModel& noise)
      : PriorFactor<Vector>(kState, calibration + scale * kDirection, noise) {}

  std::optional<Vector> parameterVjp(
      const Values&, const ParameterTag& parameter,
      const Vector& factorResidualCotangent) const override {
    if (parameter == kCalibration) return Vector(-factorResidualCotangent);
    if (parameter == kScale)
      return Vector1{-kDirection.dot(factorResidualCotangent)};
    return std::nullopt;
  }
};

class InactivePrior : public CalibrationPrior {
 public:
  using CalibrationPrior::CalibrationPrior;
  bool active(const Values&) const override { return false; }
  std::optional<Vector> parameterVjp(const Values&, const ParameterTag&,
                                    const Vector&) const override {
    throw std::runtime_error("inactive parameter VJP must not be called");
  }
};

NonlinearFactorGraph graph(const Vector2& calibration, double scale,
                           const SharedNoiseModel& firstNoise) {
  NonlinearFactorGraph result;
  result.emplace_shared<CalibrationPrior>(calibration, scale, firstNoise);
  result.push_back(NonlinearFactor::shared_ptr{});
  result.emplace_shared<CustomFactor>(
      noiseModel::Gaussian::Covariance(kCovariance), KeyVector{kState},
      [calibration, scale](const CustomFactor&, const Values& values,
                           OptionalJacobianVector jacobians) -> Vector {
        if (jacobians) jacobians->get()[0] = kStateJacobian;
        return kStateJacobian * values.at<Vector>(kState) -
               kParameterJacobian * calibration - scale * kSecondDirection;
      },
      [](const CustomFactor&, const Values&, const ParameterTag& parameter,
         const Vector& factorResidualCotangent) -> std::optional<Vector> {
        if (parameter == kCalibration)
          return Vector(-kParameterJacobian.transpose() *
                        factorResidualCotangent);
        if (parameter == kScale)
          return Vector1{-kSecondDirection.dot(factorResidualCotangent)};
        return std::nullopt;
      });
  result.emplace_shared<PriorFactor<Vector>>(kState, Vector(Vector2{.1, -.4}),
                                             noiseModel::Unit::Create(2));
  result.emplace_shared<InactivePrior>(calibration, scale, firstNoise);
  return result;
}

Values initial() {
  Values result;
  result.insert(kState, Vector(Vector2::Zero()));
  return result;
}

Values solve(const Vector2& calibration, double scale,
             const SharedNoiseModel& firstNoise) {
  return GaussNewtonOptimizer(graph(calibration, scale, firstNoise), initial())
      .optimize();
}

// Native overrides and custom callbacks accumulate shared parameters in raw
// coordinates.
TEST(GradientsParameters, SharedParametersAndNoiseWeighting) {
  const Vector2 calibration{.3, -.2}, stateCotangent{.6, -.7};
  const double scale = .8;
  const VectorValues stateCotangents{{kState, stateCotangent}};
  for (const SharedNoiseModel& noise :
       {SharedNoiseModel{},
        SharedNoiseModel(noiseModel::Isotropic::Sigma(2, 2.)),
        SharedNoiseModel(noiseModel::Diagonal::Sigmas(Vector2{.4, 1.3})),
        SharedNoiseModel(noiseModel::Gaussian::Covariance(kCovariance))}) {
    const auto factors = graph(calibration, scale, noise);
    const Values solution = solve(calibration, scale, noise);
    EXPECT(factors.linearize(solution)->gradientAtZero().norm() < 1e-10);
    const Matrix2 firstInformation =
        noise ? std::dynamic_pointer_cast<noiseModel::Gaussian>(noise)
                    ->information()
              : Matrix(Matrix2::Identity());
    const Matrix2 secondInformation = kCovariance.inverse();
    const Matrix2 hessian =
        firstInformation + Matrix2::Identity() +
        kStateJacobian.transpose() * secondInformation * kStateJacobian;
    const Vector2 adjoint = hessian.ldlt().solve(stateCotangent);
    const Vector2 expectedCalibration =
        (firstInformation +
         kParameterJacobian.transpose() * secondInformation * kStateJacobian) *
        adjoint;
    const double expectedScale =
        kDirection.dot(firstInformation * adjoint) +
        kSecondDirection.dot(secondInformation * kStateJacobian * adjoint);

    for (const auto method : {Gradients::CHOLESKY, Gradients::QR}) {
      const Gradients gradients(factors, solution, Ordering{kState}, method);
      for (size_t repeat = 0; repeat < 3; ++repeat) {
        const auto calibrationGradient =
            gradients.parameterGradient(stateCotangents, kCalibration);
        const auto scaleGradient =
            gradients.parameterGradient(stateCotangents, kScale);
        EXPECT(calibrationGradient.has_value());
        EXPECT(scaleGradient.has_value());
        EXPECT(assert_equal(expectedCalibration, calibrationGradient.value(),
                            1e-12));
        EXPECT_DOUBLES_EQUAL(expectedScale, scaleGradient.value()[0], 1e-12);
      }
      EXPECT(!gradients.parameterGradient(stateCotangents,
                                         ParameterTag{"calibration"}));
      const auto zero =
          gradients.parameterGradient(VectorValues{}, kCalibration);
      EXPECT(zero.has_value());
      EXPECT(assert_equal(Vector2::Zero(), zero.value(), 1e-12));
    }

    for (double step : {1e-4, 1e-5, 1e-6}) {
      for (size_t coordinate = 0; coordinate < 2; ++coordinate) {
        Vector2 direction = Vector2::Zero();
        direction[coordinate] = step;
        const Vector positive =
            solve(calibration + direction, scale, noise).at<Vector>(kState);
        const Vector negative =
            solve(calibration - direction, scale, noise).at<Vector>(kState);
        EXPECT_DOUBLES_EQUAL(
            expectedCalibration[coordinate],
            stateCotangent.dot(positive - negative) / (2 * step), 1e-8);
      }
      const Vector positive =
          solve(calibration, scale + step, noise).at<Vector>(kState);
      const Vector negative =
          solve(calibration, scale - step, noise).at<Vector>(kState);
      EXPECT_DOUBLES_EQUAL(expectedScale,
                           stateCotangent.dot(positive - negative) / (2 * step),
                           1e-8);
    }
  }
}

// Missing callbacks, nullopt results and empty nonlinear graphs have no
// contribution.
TEST(GradientsParameters, NoDependencies) {
  NonlinearFactorGraph factors;
  factors.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(2), KeyVector{kState},
      [](const CustomFactor&, const Values& values,
         OptionalJacobianVector jacobians) -> Vector {
        if (jacobians) jacobians->get()[0] = Matrix2::Identity();
        return values.at<Vector>(kState);
      });
  const Gradients gradients(factors, initial());
  EXPECT(
      !gradients.parameterGradient({{kState, Vector2{1., -.4}}}, kCalibration));
  const Gradients empty(NonlinearFactorGraph{}, Values{});
  EXPECT(!empty.parameterGradient(VectorValues{}, kCalibration));
  const Gradients gaussian(GaussianFactorGraph{});
  CHECK_EXCEPTION(gaussian.parameterGradient(VectorValues{}, kCalibration),
                  std::logic_error);
}

// Retained factors survive source containers, and callbacks see the saved
// solution.
TEST(GradientsParameters, SourceLifetimesAndFixedSolution) {
  size_t errorCalls = 0;
  Vector observedState;
  auto factor = std::make_shared<CustomFactor>(
      noiseModel::Unit::Create(2), KeyVector{kState},
      [&errorCalls](const CustomFactor&, const Values& values,
                    OptionalJacobianVector jacobians) -> Vector {
        ++errorCalls;
        if (jacobians) jacobians->get()[0] = Matrix2::Identity();
        return values.at<Vector>(kState);
      },
      [&observedState](const CustomFactor&, const Values& values,
                       const ParameterTag&,
                       const Vector& cotangent) -> std::optional<Vector> {
        observedState = values.at<Vector>(kState);
        return Vector(-cotangent);
      });
  std::weak_ptr<CustomFactor> weak = factor;
  NonlinearFactorGraph factors;
  factors.push_back(factor);
  Values solution = initial();
  {
    const Gradients gradients(factors, solution);
    const size_t callsAtConstruction = errorCalls;
    solution.update(kState, Vector(Vector2{100., 200.}));
    factors.resize(0);
    factor.reset();
    EXPECT(!weak.expired());
    const Gradients copied = gradients;
    for (size_t repeat = 0; repeat < 3; ++repeat) {
      const auto gradient =
          copied.parameterGradient({{kState, Vector2{.3, -.7}}}, kCalibration);
      EXPECT(assert_equal(Vector2{.3, -.7}, gradient.value(), 1e-12));
      EXPECT(assert_equal(Vector2::Zero(), observedState, 1e-12));
    }
    EXPECT_LONGS_EQUAL(callsAtConstruction, errorCalls);
  }
  EXPECT(weak.expired());
}

// A constant factor can recognize a parameter and return a zero gradient with
// its dimension.
TEST(GradientsParameters, ConstantFactor) {
  NonlinearFactorGraph factors;
  factors.emplace_shared<CustomFactor>(
      noiseModel::Isotropic::Sigma(1, 3.), KeyVector{},
      [](const CustomFactor&, const Values&, OptionalJacobianVector) -> Vector {
        return Vector1{2.};
      },
      [](const CustomFactor&, const Values&, const ParameterTag& parameter,
         const Vector& cotangent) -> std::optional<Vector> {
        if (parameter != kScale) return std::nullopt;
        return Vector(-cotangent);
      });
  const Gradients gradients(factors, Values{});
  const auto result = gradients.parameterGradient(VectorValues{}, kScale);
  EXPECT(result.has_value());
  EXPECT(assert_equal(Vector1::Zero(), result.value(), 1e-12));
}

// Callback errors propagate; malformed and incompatible returned vectors fail
// explicitly.
TEST(GradientsParameters, InvalidContributions) {
  const auto error = [](const CustomFactor&, const Values& values,
                        OptionalJacobianVector jacobians) -> Vector {
    if (jacobians) jacobians->get()[0] = Matrix2::Identity();
    return values.at<Vector>(kState);
  };
  const VectorValues cotangents{{kState, Vector2{1., 2.}}};
  for (const Vector& invalid :
       {Vector{}, Vector(Vector1{std::numeric_limits<double>::quiet_NaN()})}) {
    NonlinearFactorGraph factors;
    factors.emplace_shared<CustomFactor>(
        noiseModel::Unit::Create(2), KeyVector{kState}, error,
        [invalid](const CustomFactor&, const Values&, const ParameterTag&,
                  const Vector&) -> std::optional<Vector> { return invalid; });
    CHECK_EXCEPTION(
        Gradients(factors, initial()).parameterGradient(cotangents, kScale),
        std::invalid_argument);
  }
  NonlinearFactorGraph factors;
  for (size_t dimension : {1, 2}) {
    factors.emplace_shared<CustomFactor>(
        noiseModel::Unit::Create(2), KeyVector{kState}, error,
        [dimension](const CustomFactor&, const Values&, const ParameterTag&,
                    const Vector&) -> std::optional<Vector> {
          return Vector::Zero(dimension);
        });
  }
  CHECK_EXCEPTION(
      Gradients(factors, initial()).parameterGradient(cotangents, kScale),
      std::invalid_argument);
  factors.resize(0);
  factors.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(2), KeyVector{kState}, error,
      [](const CustomFactor&, const Values&, const ParameterTag&,
         const Vector&) -> std::optional<Vector> {
        throw std::runtime_error("callback failure");
      });
  CHECK_EXCEPTION(
      Gradients(factors, initial()).parameterGradient(cotangents, kScale),
      std::runtime_error);
}

}  // namespace parameter_fixture
/* ************************************************************************* */

namespace noise_fixture {

constexpr Key kPosition = 7;
const Vector2 kLearnedMeasurement{0., 0.}, kAnchorMeasurement{1., -0.5};
const Vector2 kUnequalSigmas{0.4, 1.3};
const Vector2 kTarget{0.2, -0.3};

NonlinearFactorGraph sigmaGraph(const SharedNoiseModel& learned) {
  NonlinearFactorGraph result;
  result.emplace_shared<PriorFactor<Vector>>(kPosition, kLearnedMeasurement,
                                             learned);
  result.emplace_shared<PriorFactor<Vector>>(
      kPosition, kAnchorMeasurement, noiseModel::Isotropic::Sigma(2, 0.5));
  return result;
}

Values sigmaInitial() {
  Values result;
  result.insert(kPosition, Vector(Vector2::Zero()));
  return result;
}

Vector sigmaSolve(const NonlinearFactorGraph& graph) {
  return GaussNewtonOptimizer(graph, sigmaInitial())
      .optimize()
      .at<Vector>(kPosition);
}

VectorValues sigmaCotangents(const Values& solution) {
  return {{kPosition, solution.at<Vector>(kPosition) - kTarget}};
}

/// Central-difference dL/dσ_i with L = ½|x - target|², graph built per σ.
template <class MakeGraph>
double finiteDifference(const MakeGraph& makeGraph, const Vector2& sigmas,
                        const Vector& cotangent, size_t coordinate) {
  const double step = 1e-6;
  Vector2 plus = sigmas, minus = sigmas;
  plus[coordinate] += step;
  minus[coordinate] -= step;
  const Vector dx = sigmaSolve(makeGraph(plus)) - sigmaSolve(makeGraph(minus));
  return cotangent.dot(dx) / (2 * step);
}

// Diagonal σ gradient matches finite differences and is twice the
// Jacobian-held-fixed term obtained by baking r = e ⊘ σ into the residual.
TEST(GradientsNoise, DiagonalSigma) {
  const auto learned = noiseModel::Diagonal::Sigmas(kUnequalSigmas, false);
  const NonlinearFactorGraph graph = sigmaGraph(learned);
  const Values solution = GaussNewtonOptimizer(graph, sigmaInitial()).optimize();
  EXPECT(graph.linearize(solution)->gradientAtZero().norm() < 1e-10);
  const VectorValues cotangents = sigmaCotangents(solution);
  const Vector analytic =
      Gradients(graph, solution).noiseGradient(cotangents, learned).value();
  EXPECT_LONGS_EQUAL(2, analytic.size());

  const auto makeGraph = [](const Vector2& sigmas) {
    return sigmaGraph(noiseModel::Diagonal::Sigmas(sigmas, false));
  };
  for (size_t coordinate = 0; coordinate < 2; ++coordinate)
    EXPECT_DOUBLES_EQUAL(
        analytic[coordinate],
        finiteDifference(makeGraph, kUnequalSigmas, cotangents.at(kPosition),
                         coordinate),
        1e-6);

  const ParameterTag baked{"baked sigma"};
  NonlinearFactorGraph bakedGraph;
  bakedGraph.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(2), KeyVector{kPosition},
      [](const CustomFactor&, const Values& values,
         OptionalJacobianVector jacobians) -> Vector {
        const Vector error =
            (values.at<Vector>(kPosition) - kLearnedMeasurement).array() /
            kUnequalSigmas.array();
        if (jacobians) {
          Matrix2 jacobian = Matrix2::Zero();
          jacobian.diagonal() = kUnequalSigmas.array().inverse();
          jacobians->get()[0] = jacobian;
        }
        return error;
      },
      [baked](const CustomFactor&, const Values& values,
              const ParameterTag& parameter,
              const Vector& cotangent) -> std::optional<Vector> {
        if (parameter != baked) return std::nullopt;
        const Vector residual =
            values.at<Vector>(kPosition) - kLearnedMeasurement;
        return Vector((-cotangent.array() * residual.array() /
                       kUnequalSigmas.array().square())
                          .matrix());
      });
  bakedGraph.emplace_shared<PriorFactor<Vector>>(
      kPosition, kAnchorMeasurement, noiseModel::Isotropic::Sigma(2, 0.5));
  const Values bakedSolution =
      GaussNewtonOptimizer(bakedGraph, sigmaInitial()).optimize();
  EXPECT(assert_equal(solution.at<Vector>(kPosition),
                      bakedSolution.at<Vector>(kPosition), 1e-10));
  const auto bakedGradient =
      Gradients(bakedGraph, bakedSolution).parameterGradient(cotangents, baked);
  EXPECT(assert_equal((0.5 * analytic).eval(), bakedGradient.value(), 1e-10));
}

// An Isotropic model returns one shared σ: the sum of the per-row gradients
// of an equal-sigma Diagonal built with smart = false.
TEST(GradientsNoise, IsotropicSigma) {
  const double sigma = 0.7;
  const auto isotropic = noiseModel::Isotropic::Sigma(2, sigma);
  const auto diagonal =
      noiseModel::Diagonal::Sigmas(Vector2::Constant(sigma), false);
  EXPECT(!std::dynamic_pointer_cast<noiseModel::Isotropic>(diagonal));
  const Values solution =
      GaussNewtonOptimizer(sigmaGraph(isotropic), sigmaInitial()).optimize();
  const VectorValues cotangents = sigmaCotangents(solution);
  const Vector isotropicGradient = Gradients(sigmaGraph(isotropic), solution)
                                       .noiseGradient(cotangents, isotropic)
                                       .value();
  const Vector diagonalGradient = Gradients(sigmaGraph(diagonal), solution)
                                      .noiseGradient(cotangents, diagonal)
                                      .value();
  EXPECT_LONGS_EQUAL(1, isotropicGradient.size());
  EXPECT_LONGS_EQUAL(2, diagonalGradient.size());
  EXPECT_DOUBLES_EQUAL(diagonalGradient.sum(), isotropicGradient[0], 1e-12);
}

/// Two learned factors plus an anchor; first and second may share a model.
NonlinearFactorGraph twoFactorGraph(const SharedNoiseModel& first,
                                    const SharedNoiseModel& second) {
  NonlinearFactorGraph result;
  result.emplace_shared<PriorFactor<Vector>>(kPosition, kLearnedMeasurement,
                                             first);
  result.emplace_shared<PriorFactor<Vector>>(kPosition, Vector2{0.3, 0.4},
                                             second);
  result.emplace_shared<PriorFactor<Vector>>(
      kPosition, kAnchorMeasurement, noiseModel::Isotropic::Sigma(2, 0.5));
  return result;
}

// One instance shared by two factors accumulates both contributions.
TEST(GradientsNoise, SharedInstanceAccumulates) {
  const auto makeGraph = [](const Vector2& sigmas) {
    const auto shared = noiseModel::Diagonal::Sigmas(sigmas, false);
    return twoFactorGraph(shared, shared);
  };
  const auto shared = noiseModel::Diagonal::Sigmas(kUnequalSigmas, false);
  const NonlinearFactorGraph graph = twoFactorGraph(shared, shared);
  const Values solution = GaussNewtonOptimizer(graph, sigmaInitial()).optimize();
  const VectorValues cotangents = sigmaCotangents(solution);
  const Vector analytic =
      Gradients(graph, solution).noiseGradient(cotangents, shared).value();
  for (size_t coordinate = 0; coordinate < 2; ++coordinate)
    EXPECT_DOUBLES_EQUAL(
        analytic[coordinate],
        finiteDifference(makeGraph, kUnequalSigmas, cotangents.at(kPosition),
                         coordinate),
        1e-6);
}

// Separately constructed models with equal sigmas are distinct parameters:
// each gets only its own factor, and together they sum to the shared case.
TEST(GradientsNoise, EqualModelsAreDistinct) {
  const auto first = noiseModel::Diagonal::Sigmas(kUnequalSigmas, false);
  const auto second = noiseModel::Diagonal::Sigmas(kUnequalSigmas, false);
  const NonlinearFactorGraph separate = twoFactorGraph(first, second);
  const NonlinearFactorGraph shared = twoFactorGraph(first, first);
  const Values solution =
      GaussNewtonOptimizer(separate, sigmaInitial()).optimize();
  const VectorValues cotangents = sigmaCotangents(solution);
  const Gradients gradients(separate, solution);
  const Vector firstGradient = gradients.noiseGradient(cotangents, first).value();
  const Vector secondGradient =
      gradients.noiseGradient(cotangents, second).value();
  const Vector sharedGradient =
      Gradients(shared, solution).noiseGradient(cotangents, first).value();
  EXPECT((sharedGradient - firstGradient).norm() > 1e-6);
  EXPECT(assert_equal(sharedGradient, Vector(firstGradient + secondGradient),
                      1e-12));
}

// Unsupported models and Gaussian-only construction are rejected; a model no
// active factor holds, even with equal sigmas, has no gradient.
TEST(GradientsNoise, Errors) {
  const auto diagonal = noiseModel::Diagonal::Sigmas(kUnequalSigmas, false);
  const NonlinearFactorGraph graph = sigmaGraph(diagonal);
  const Values solution = GaussNewtonOptimizer(graph, sigmaInitial()).optimize();
  const VectorValues cotangents = sigmaCotangents(solution);
  const Gradients gradients(graph, solution);

  // Equal values with smart = true collapse to Unit, which has no parameters.
  const auto unit = noiseModel::Diagonal::Sigmas(Vector2::Ones());
  EXPECT(unit->isUnit());
  CHECK_EXCEPTION(Gradients(sigmaGraph(unit), solution)
                      .noiseGradient(cotangents, unit),
                  std::invalid_argument);

  const auto robust = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.0), diagonal);
  CHECK_EXCEPTION(gradients.noiseGradient(cotangents, robust),
                  std::invalid_argument);
  CHECK_EXCEPTION(gradients.noiseGradient(cotangents, nullptr),
                  std::invalid_argument);
  EXPECT(!gradients.noiseGradient(
      cotangents, noiseModel::Diagonal::Sigmas(kUnequalSigmas, false)));
  CHECK_EXCEPTION(Gradients(*graph.linearize(solution))
                      .noiseGradient(cotangents, diagonal),
                  std::logic_error);
}

}  // namespace noise_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
