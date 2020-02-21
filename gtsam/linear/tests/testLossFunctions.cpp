/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testLossFunctions.cpp
 * @date Feb 21, 2020
 * @author Yetong Zhang
 * @author Fan Jiang
 * @author Frank Dellaert
 */

#include <gtsam/linear/LossFunctions.h>

#include <CppUnitLite/TestHarness.h>

/*
 * These tests are responsible for testing the weight functions for the
 * m-estimators in GTSAM. The weight function is related to the analytic
 * derivative of the residual function. See
 *  https://members.loria.fr/MOBerger/Enseignement/Master2/Documents/ZhangIVC-97-01.pdf
 * for details. This weight function is required when optimizing cost functions
 * with robust penalties using iteratively re-weighted least squares.
 */

TEST(NoiseModel, robustFunctionFair) {
  using gtsam::noiseModel::mEstimator::Fair;
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const Fair::shared_ptr fair = Fair::Create(k);
  DOUBLES_EQUAL(0.8333333333333333, fair->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.3333333333333333, fair->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.3333333333333333, fair->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.8333333333333333, fair->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.441961080151135, fair->residual(error1), 1e-8);
  DOUBLES_EQUAL(22.534692783297260, fair->residual(error2), 1e-8);
  DOUBLES_EQUAL(22.534692783297260, fair->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.441961080151135, fair->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionHuber) {
  using gtsam::noiseModel::mEstimator::Huber;
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const Huber::shared_ptr huber = Huber::Create(k);
  DOUBLES_EQUAL(1.0, huber->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.5, huber->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.5, huber->weight(error3), 1e-8);
  DOUBLES_EQUAL(1.0, huber->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.5000, huber->residual(error1), 1e-8);
  DOUBLES_EQUAL(37.5000, huber->residual(error2), 1e-8);
  DOUBLES_EQUAL(37.5000, huber->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.5000, huber->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionCauchy) {
  using gtsam::noiseModel::mEstimator::Cauchy;
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const Cauchy::shared_ptr cauchy = Cauchy::Create(k);
  DOUBLES_EQUAL(0.961538461538461, cauchy->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.2000, cauchy->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.2000, cauchy->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.961538461538461, cauchy->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.490258914416017, cauchy->residual(error1), 1e-8);
  DOUBLES_EQUAL(20.117973905426254, cauchy->residual(error2), 1e-8);
  DOUBLES_EQUAL(20.117973905426254, cauchy->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.490258914416017, cauchy->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionGemanMcClure) {
  using gtsam::noiseModel::mEstimator::GemanMcClure;
  const double k = 1.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const GemanMcClure::shared_ptr gmc = GemanMcClure::Create(k);
  DOUBLES_EQUAL(0.25, gmc->weight(error1), 1e-8);
  DOUBLES_EQUAL(9.80296e-5, gmc->weight(error2), 1e-8);
  DOUBLES_EQUAL(9.80296e-5, gmc->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.25, gmc->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.2500, gmc->residual(error1), 1e-8);
  DOUBLES_EQUAL(0.495049504950495, gmc->residual(error2), 1e-8);
  DOUBLES_EQUAL(0.495049504950495, gmc->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.2500, gmc->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionWelsch) {
  using gtsam::noiseModel::mEstimator::Welsch;
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const Welsch::shared_ptr welsch = Welsch::Create(k);
  DOUBLES_EQUAL(0.960789439152323, welsch->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.018315638888734, welsch->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.018315638888734, welsch->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.960789439152323, welsch->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.490132010595960, welsch->residual(error1), 1e-8);
  DOUBLES_EQUAL(12.271054513890823, welsch->residual(error2), 1e-8);
  DOUBLES_EQUAL(12.271054513890823, welsch->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.490132010595960, welsch->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionTukey) {
  using gtsam::noiseModel::mEstimator::Tukey;
  const double k = 5.0, error1 = 1.0, error2 = 10.0, error3 = -10.0,
               error4 = -1.0;
  const Tukey::shared_ptr tukey = Tukey::Create(k);
  DOUBLES_EQUAL(0.9216, tukey->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.0, tukey->weight(error2), 1e-8);
  // Test negative value to ensure we take absolute value of error.
  DOUBLES_EQUAL(0.0, tukey->weight(error3), 1e-8);
  DOUBLES_EQUAL(0.9216, tukey->weight(error4), 1e-8);

  DOUBLES_EQUAL(0.480266666666667, tukey->residual(error1), 1e-8);
  DOUBLES_EQUAL(4.166666666666667, tukey->residual(error2), 1e-8);
  DOUBLES_EQUAL(4.166666666666667, tukey->residual(error3), 1e-8);
  DOUBLES_EQUAL(0.480266666666667, tukey->residual(error4), 1e-8);
}

TEST(NoiseModel, robustFunctionDCS) {
  using gtsam::noiseModel::mEstimator::DCS;
  const double k = 1.0, error1 = 1.0, error2 = 10.0;
  const DCS::shared_ptr dcs = DCS::Create(k);

  DOUBLES_EQUAL(1.0, dcs->weight(error1), 1e-8);
  DOUBLES_EQUAL(0.00039211, dcs->weight(error2), 1e-8);

  DOUBLES_EQUAL(0.5, dcs->residual(error1), 1e-8);
  DOUBLES_EQUAL(0.9900990099, dcs->residual(error2), 1e-8);
}

TEST(NoiseModel, robustFunctionL2WithDeadZone) {
  using gtsam::noiseModel::mEstimator::L2WithDeadZone;
  const double k = 1.0, e0 = -10.0, e1 = -1.01, e2 = -0.99, e3 = 0.99,
               e4 = 1.01, e5 = 10.0;
  const L2WithDeadZone::shared_ptr lsdz = L2WithDeadZone::Create(k);

  DOUBLES_EQUAL(0.9, lsdz->weight(e0), 1e-8);
  DOUBLES_EQUAL(0.00990099009, lsdz->weight(e1), 1e-8);
  DOUBLES_EQUAL(0.0, lsdz->weight(e2), 1e-8);
  DOUBLES_EQUAL(0.0, lsdz->weight(e3), 1e-8);
  DOUBLES_EQUAL(0.00990099009, lsdz->weight(e4), 1e-8);
  DOUBLES_EQUAL(0.9, lsdz->weight(e5), 1e-8);

  DOUBLES_EQUAL(40.5, lsdz->residual(e0), 1e-8);
  DOUBLES_EQUAL(0.00005, lsdz->residual(e1), 1e-8);
  DOUBLES_EQUAL(0.0, lsdz->residual(e2), 1e-8);
  DOUBLES_EQUAL(0.0, lsdz->residual(e3), 1e-8);
  DOUBLES_EQUAL(0.00005, lsdz->residual(e4), 1e-8);
  DOUBLES_EQUAL(40.5, lsdz->residual(e5), 1e-8);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
