/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testPlanarGyroFactor.cpp
 * @date May 1, 2026
 * @author joel@truher.org
 * @brief tests for PlanarGyroFactor
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/FixedJacobianFactor.h>
#include <gtsam/navigation/PlanarGyroFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

namespace gtsam {
using symbol_shorthand::B;
using symbol_shorthand::P;

TEST(PlanarGyroFactor, fromRate) {
  double arwSigma = 1.0;
  double omega = 0.1;
  double dt = 0.5;
  double biasInstabilitySigma = 3e-4;

  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);

  PlanarGyroFactor x =
      PlanarGyroFactor::FromRate(P(0), P(1), B(0), p, omega, dt);

  // Check the effect of bias.
  double bias = 0.05;
  Matrix1 H;
  Rot2 corrected = x.deltaR(bias, H);
  EXPECT(assert_equal(0.025, corrected.theta(), 1e-9))
  EXPECT(assert_equal(-0.5, H(0, 0), 1e-9))

  // Numeric derivative matches.
  auto f = [&x](const double& bias) { return x.deltaR(bias, {}); };
  Matrix1 numericH = numericalDerivative11(f, bias);
  EXPECT(assert_equal(-0.5, numericH(0, 0), 1e-9))
}

TEST(PlanarGyroFactor, fromRotation) {
  double arwSigma = 1.0;
  Rot2 dr = 0.05;
  double dt = 0.5;
  double biasInstabilitySigma = 3e-4;

  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);

  PlanarGyroFactor x =
      PlanarGyroFactor::FromRotation(P(0), P(1), B(0), p, dr, dt);
  double bias = 0.05;
  Matrix1 H;
  Rot2 corrected = x.deltaR(bias, H);
  EXPECT(assert_equal(0.025, corrected.theta(), 1e-9))
  EXPECT(assert_equal(-0.5, H(0, 0), 1e-9))
}

TEST(PlanarGyroFactor, TernaryLinearization) {
  const auto params = std::make_shared<PlanarGyroParams>(1.0, 3e-4);
  const PlanarGyroFactor factor =
      PlanarGyroFactor::FromRate(P(0), P(1), B(0), params, 0.1, 0.5);
  const Values values{{P(0), genericValue(Pose2(1.0, 2.0, 0.3))},
                      {P(1), genericValue(Pose2(1.2, 2.1, 0.4))},
                      {B(0), genericValue(0.02)}};

  const auto generic = factor.NoiseModelFactor::linearize(values);
  const auto optimized = factor.linearize(values);
  const bool isTernary = static_cast<bool>(
      std::dynamic_pointer_cast<FixedJacobianFactor<3, 3, 3, 1>>(
          optimized));
  CHECK(isTernary);
  EXPECT(assert_equal(*generic, *optimized, 1e-12));
}

TEST(PlanarGyroParams, variance) {
  double arwSigma = 1.0;
  double dt = 0.5;
  double biasInstabilitySigma = 3e-4;
  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);
  // 1.0 * 0.5 = 0.5
  EXPECT(assert_equal(0.707107, p->arwSigma(dt), 1e-6))
}

TEST(PlanarGyroFactor, predict) {
  double arwSigma = 1.0;
  double omega = 0.1;
  double dt = 0.5;
  double biasInstabilitySigma = 3e-4;

  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);

  PlanarGyroFactor x =
      PlanarGyroFactor::FromRate(P(0), P(1), B(0), p, omega, dt);

  // Check prediction.
  Rot2 Ri = Rot2::fromAngle(1);
  double bias = 0.05;
  Matrix1 H1, H2;
  Rot2 predictedRj = x.predict(Ri, bias, H1, H2);

  // 1 + 0.025 = 1.025
  EXPECT(assert_equal(1.025, predictedRj.theta(), 1e-9))
  // Ri adds to prediction.
  EXPECT(assert_equal(1.0, H1(0, 0), 1e-9))
  // Bias * dt subtracts from prediction.
  EXPECT(assert_equal(-0.5, H2(0, 0), 1e-9))

  // Numeric derivative matches.
  auto f = [&x](const Rot2& r, const double& b) -> Rot2 {
    return x.predict(r, b);
  };
  Matrix1 nH1 = numericalDerivative21(f, Ri, bias);
  Matrix1 nH2 = numericalDerivative22(f, Ri, bias);
  EXPECT(assert_equal(1.0, nH1(0, 0), 1e-9))
  EXPECT(assert_equal(-0.5, nH2(0, 0), 1e-9))
}

TEST(PlanarGyroFactor, computeError) {
  double arwSigma = 1.0;
  double omega = 0.1;
  double dt = 0.5;
  double biasInstabilitySigma = 3e-4;

  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);

  PlanarGyroFactor x =
      PlanarGyroFactor::FromRate(P(0), P(1), B(0), p, omega, dt);

  // Check error.
  Rot2 Ri = Rot2::fromAngle(1);
  Rot2 Rj = Rot2::fromAngle(2);
  double bias = 0.05;
  Matrix1 H1, H2, H3;
  double err = x.computeError(Ri, Rj, bias, H1, H2, H3);

  // estimate - prediction = 2 - 1.025 = -0.975
  EXPECT(assert_equal(-0.975, err, 1e-9))
  // Ri up => error up (less negative)
  EXPECT(assert_equal(1.0, H1(0, 0), 1e-9))
  // Rj up -> error down (more negative)
  EXPECT(assert_equal(-1.0, H2(0, 0), 1e-9))
  // bias up -> error down (more negative), scaled by dt
  EXPECT(assert_equal(-0.5, H3(0, 0), 1e-9))

  // Numeric derivative matches
  auto f = [&x](const Rot2& r1, const Rot2& r2, const double& b) -> double {
    return x.computeError(r1, r2, b);
  };
  Matrix1 nH1 = numericalDerivative31(f, Ri, Rj, bias);
  Matrix1 nH2 = numericalDerivative32(f, Ri, Rj, bias);
  Matrix1 nH3 = numericalDerivative33(f, Ri, Rj, bias);
  EXPECT(assert_equal(1.0, nH1(0, 0), 1e-9))
  EXPECT(assert_equal(-1.0, nH2(0, 0), 1e-9))
  EXPECT(assert_equal(-0.5, nH3(0, 0), 1e-9))
}

TEST(PlanarGyroFactor, evaluateError) {
  using symbol_shorthand::B;
  using symbol_shorthand::P;
  double arwSigma = 0.1;
  double trueOmega = M_PI / 10.0;
  double B1 = 0.3;
  // Measurement includes bias.
  double measuredOmega = trueOmega + B1;
  double deltaT = 1.0;
  double biasInstabilitySigma = 3e-4;

  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);

  PlanarGyroFactor factor =
      PlanarGyroFactor::FromRate(P(1), P(2), B(1), p, measuredOmega, deltaT);

  double initialRotation = M_PI / 4.0;
  Pose2 P1(0.0, 0.0, initialRotation);
  double error = 0.1;
  Pose2 P2(0.0, 0.0, initialRotation + trueOmega * deltaT - error);

  EXPECT(assert_equal(Vector3(0, 0, error), factor.evaluateError(P1, P2, B1),
                      1e-6))
}

TEST(PlanarGyroFactor, optimize) {
  using noiseModel::Diagonal;

  NonlinearFactorGraph graph;

  // Starting pose is known.
  graph.add(PriorFactor<Pose2>(P(0), Pose2(),
                               Diagonal::Sigmas(Vector3(0.001, 0.001, 0.001))));

  // BetweenFactors that simulate odometry.
  Pose2 p0 = Pose2(0, 0, 0);
  Pose2 p1 = Pose2(0, 0, 0.1);
  Pose2 p2 = Pose2(0.1, 0, 0.2);
  Pose2 p3 = Pose2(0.2, 0, 0.3);
  Pose2 p4 = Pose2(0.3, 0, 0.4);
  // Add error in the "between" rotation, so the gyro factor can fix it.
  Pose2 pErr = Pose2(0, 0, 0.1);
  // When motionless, the rotation is known.
  // This is how we learn the bias.
  SharedDiagonal lowRotationNoise = Diagonal::Sigmas(Vector3(1e-3, 1e-3, 1e-3));
  graph.add(BetweenFactor<Pose2>(P(0), P(1), p0.between(p1), lowRotationNoise));

  // When moving, rotation is much less certain.
  SharedDiagonal highRotationNoise = Diagonal::Sigmas(Vector3(1e-3, 1e-3, 1));
  graph.add(BetweenFactor<Pose2>(P(1), P(2), p1.between(p2).compose(pErr),
                                 highRotationNoise));
  graph.add(BetweenFactor<Pose2>(P(2), P(3), p2.between(p3).compose(pErr),
                                 highRotationNoise));
  graph.add(BetweenFactor<Pose2>(P(3), P(4), p3.between(p4).compose(pErr),
                                 highRotationNoise));

  // Bias prior: very uncertain.
  graph.add(PriorFactor<double>(B(0), 1.0, Diagonal::Sigmas(Vector1(1))));

  // Gyro measurements affect rotation only.
  double trueOmega = 0.1;
  double bias = 1;  // large !
  double measuredOmega = trueOmega + bias;
  double dt = 1.0;
  double arwSigma = 1e-4;
  double biasInstabilitySigma = 3e-4;

  // Bias evolution.  Bias stability is an important parameter.
  auto p = std::make_shared<PlanarGyroParams>(arwSigma, biasInstabilitySigma);
  graph.add(PlanarGyroBiasFactor(B(0), B(1), p));
  graph.add(PlanarGyroBiasFactor(B(1), B(2), p));
  graph.add(PlanarGyroBiasFactor(B(2), B(3), p));
  graph.add(PlanarGyroBiasFactor(B(3), B(4), p));

  graph.add(PlanarGyroFactor::FromRate(P(0), P(1), B(0), p, measuredOmega, dt));
  graph.add(PlanarGyroFactor::FromRate(P(1), P(2), B(1), p, measuredOmega, dt));
  graph.add(PlanarGyroFactor::FromRate(P(2), P(3), B(2), p, measuredOmega, dt));
  graph.add(PlanarGyroFactor::FromRate(P(3), P(4), B(3), p, measuredOmega, dt));

  // Initial values should not matter.
  Values values;
  values.insert(B(0), 0.0);
  values.insert(B(1), 0.0);
  values.insert(B(2), 0.0);
  values.insert(B(3), 0.0);
  values.insert(B(4), 0.0);
  values.insert(P(0), Pose2());
  values.insert(P(1), Pose2());
  values.insert(P(2), Pose2());
  values.insert(P(3), Pose2());
  values.insert(P(4), Pose2());

  LevenbergMarquardtParams params;
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  // Rotation increments are what the more-certain gyro factor said, overriding
  // what the less-certain "between" factor said.
  EXPECT(assert_equal(Pose2(0.0, 0.0, 0.0), result.at<Pose2>(P(0)), 1e-5));
  EXPECT(assert_equal(Pose2(0.0, 0.0, 0.1), result.at<Pose2>(P(1)), 1e-5));
  EXPECT(assert_equal(Pose2(0.1, 0.0, 0.2), result.at<Pose2>(P(2)), 1e-5));
  EXPECT(assert_equal(Pose2(0.2, 0.0, 0.3), result.at<Pose2>(P(3)), 1e-5));
  EXPECT(assert_equal(Pose2(0.3, 0.0, 0.4), result.at<Pose2>(P(4)), 1e-5));

  // Bias is correctly learned.
  EXPECT(assert_equal(1.0, result.at<double>(B(0)), 1e-6));
  EXPECT(assert_equal(1.0, result.at<double>(B(1)), 1e-6));
  EXPECT(assert_equal(1.0, result.at<double>(B(2)), 1e-6));
  EXPECT(assert_equal(1.0, result.at<double>(B(3)), 1e-6));
  EXPECT(assert_equal(1.0, result.at<double>(B(4)), 1e-6));

  Marginals marginals(graph, result);

  // Look at std dev because it's not so tiny.
  EXPECT(assert_equal(
      Vector3(0.001000, 0.001000, 0.001000),
      Vector3(marginals.marginalCovariance(P(0)).diagonal().cwiseSqrt()), 1e-6))
  EXPECT(assert_equal(
      Vector3(0.001414, 0.001414, 0.001414),
      Vector3(marginals.marginalCovariance(P(1)).diagonal().cwiseSqrt()), 1e-6))
  EXPECT(assert_equal(
      Vector3(0.001732, 0.001738, 0.002261),
      Vector3(marginals.marginalCovariance(P(2)).diagonal().cwiseSqrt()), 1e-6))
  EXPECT(assert_equal(
      Vector3(0.002003, 0.002030, 0.003242),
      Vector3(marginals.marginalCovariance(P(3)).diagonal().cwiseSqrt()), 1e-6))
  EXPECT(assert_equal(
      Vector3(0.002252, 0.002322, 0.004287),
      Vector3(marginals.marginalCovariance(P(4)).diagonal().cwiseSqrt()), 1e-6))

  // Bias variance is roughly constant.
  EXPECT(assert_equal(0.001005, sqrt(marginals.marginalCovariance(B(0))(0, 0)),
                      1e-6))
  EXPECT(assert_equal(0.001049, sqrt(marginals.marginalCovariance(B(1))(0, 0)),
                      1e-6))
  EXPECT(assert_equal(0.001091, sqrt(marginals.marginalCovariance(B(2))(0, 0)),
                      1e-6))
  EXPECT(assert_equal(0.001131, sqrt(marginals.marginalCovariance(B(3))(0, 0)),
                      1e-6))
  EXPECT(assert_equal(0.001170, sqrt(marginals.marginalCovariance(B(4))(0, 0)),
                      1e-6))
}
}  // namespace gtsam

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
