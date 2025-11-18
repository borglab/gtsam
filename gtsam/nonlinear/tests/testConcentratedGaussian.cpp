/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testConcentratedGaussian.cpp
 * @date September 30, 2025
 * @author Frank Dellaert
 * @brief Unit tests for ConcentratedGaussian
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ConcentratedGaussian.h>

using namespace gtsam;
using sharedGaussianNoiseModel = noiseModel::Gaussian::shared_ptr;

//******************************************************************************
TEST(ConcentratedGaussian, Pose2) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  sharedGaussianNoiseModel model =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3));
  ConcentratedGaussian<Pose2> factor(key, origin, model);

  // Test error
  Pose2 x = origin;
  Values values;
  values.insert(key, x);
  EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-9);

  // Test logProbability and evaluate
  double expected_log_prob =
      -0.5 * 3 * log(2 * M_PI) + log(1.0 / (0.1 * 0.2 * 0.3));
  EXPECT_DOUBLES_EQUAL(expected_log_prob, factor.logProbability(x), 1e-9);
  EXPECT_DOUBLES_EQUAL(expected_log_prob, factor.logProbability(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(exp(expected_log_prob), factor.evaluate(x), 1e-9);
  EXPECT_DOUBLES_EQUAL(exp(expected_log_prob), factor.evaluate(values), 1e-9);
}

//******************************************************************************
TEST(ConcentratedGaussian, Pose2WithMean) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  sharedGaussianNoiseModel model =
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3));

  Pose2 x = origin;
  Values values;
  values.insert(key, x);

  // Non-zero mean in tangent space (dx, dy, dtheta)
  Vector3 mean(0.05, -0.02, 0.1);
  ConcentratedGaussian<Pose2> factor(key, origin, mean, model);

  // Compute expected error = 0.5 * ||-mean||^2_{Sigma^{-1}}
  Vector sigmas(3);
  sigmas << 0.1, 0.2, 0.3;
  Vector invSigmas = sigmas.cwiseInverse();
  double mahalanobis =
      (-mean.cwiseProduct(invSigmas)).squaredNorm();  // (r .* invSigmas)^2 sum
  double expected_error = 0.5 * mahalanobis;
  EXPECT_DOUBLES_EQUAL(expected_error, factor.error(values), 1e-9);

  // Normalization constant (same as zero-mean case)
  double logDetSigma =
      2 * log(0.1) + 2 * log(0.2) + 2 * log(0.3);  // log(σ^2) per component sum
  double expected_log_k = -0.5 * 3 * log(2 * M_PI) - 0.5 * logDetSigma;
  double expected_log_prob = expected_log_k - expected_error;
  EXPECT_DOUBLES_EQUAL(expected_log_prob, factor.logProbability(x), 1e-9);
  EXPECT_DOUBLES_EQUAL(expected_log_prob, factor.logProbability(values), 1e-9);
  EXPECT_DOUBLES_EQUAL(exp(expected_log_prob), factor.evaluate(x), 1e-9);
  EXPECT_DOUBLES_EQUAL(exp(expected_log_prob), factor.evaluate(values), 1e-9);

  // Compare value vs values interface consistency
  EXPECT_DOUBLES_EQUAL(factor.logProbability(x), factor.logProbability(values),
                       1e-9);
  EXPECT_DOUBLES_EQUAL(factor.evaluate(x), factor.evaluate(values), 1e-9);
}

//******************************************************************************
TEST(ConcentratedGaussian, TransportToSameOrigin) {
  Key key(1);
  Pose2 origin(1.0, 2.0, 0.3);
  Matrix3 Sigma;
  Sigma << 0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.16;

  // Nonzero mean to verify it's preserved when mapping to same reference
  Vector3 mean;
  mean << 0.01, -0.02, 0.05;
  ConcentratedGaussian<Pose2> d(key, origin, mean, Sigma);

  ConcentratedGaussian<Pose2> mapped = d.transportTo(origin);

  // Origin preserved
  EXPECT(assert_equal(origin, mapped.origin()));

  // Mean and covariance preserved
  auto g = mapped.gaussianModel("TransportToSameOrigin");
  CHECK(g);
  EXPECT(assert_equal(Sigma, g->covariance(), 1e-9));
  CHECK(mapped.mean().has_value());
  EXPECT(assert_equal(mean, *mapped.mean(), 1e-9));
}

//******************************************************************************
TEST(ConcentratedGaussian, ResetMatchesTransport) {
  Key key(1);
  Pose2 origin(1.0, 2.0, 0.3);
  Matrix3 Sigma;
  Sigma << 0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.16;

  Vector3 mean;
  mean << 0.05, -0.02, 0.1;
  ConcentratedGaussian<Pose2> d(key, origin, mean, Sigma);

  // Compute expected new origin xplus = Retract(origin, mean)
  Pose2 xplus = traits<Pose2>::Retract(origin, mean);

  // Our reset method should move origin to xplus and set zero mean
  ConcentratedGaussian<Pose2> r = d.reset();
  EXPECT(assert_equal(xplus, r.origin()));
  CHECK(!r.mean().has_value());

  // Transport-to(xplus) should yield the same covariance, and near-zero mean
  ConcentratedGaussian<Pose2> mapped = d.transportTo(xplus);
  auto g_r = r.gaussianModel("ResetMatchesTransport");
  auto g_m = mapped.gaussianModel("ResetMatchesTransport");
  CHECK(g_r && g_m);
  EXPECT(assert_equal(g_r->covariance(), g_m->covariance(), 1e-9));

  // Mapped mean should be (approximately) zero in the chart at xplus
  Vector3 zero = Vector3::Zero();
  Vector3 mapped_mean = mapped.mean().value_or(Vector3::Zero());
  EXPECT(assert_equal(zero, mapped_mean, 1e-7));
}

//******************************************************************************
TEST(ConcentratedGaussian, FusionPose2Identical) {
  Key key(1);
  Pose2 origin(1.0, 2.0, 0.3);
  Matrix3 Sigma;
  Sigma << 0.04, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0, 0.16;
  sharedGaussianNoiseModel model = noiseModel::Gaussian::Covariance(Sigma);

  ConcentratedGaussian<Pose2> a(key, origin, model);
  ConcentratedGaussian<Pose2> b(key, origin, model);

  ConcentratedGaussian<Pose2> fused = a * b;

  // Expect origin unchanged for identical inputs
  EXPECT(assert_equal(origin, fused.origin()));

  // Covariance should be halved: (Σ^{-1}+Σ^{-1})^{-1} = Σ/2
  auto gf = fused.gaussianModel("FusionPose2Identical");
  CHECK(gf);
  Matrix3 Sigma_f = gf->covariance();
  EXPECT(assert_equal<Matrix3>(0.5 * Sigma, Sigma_f, 1e-9));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
