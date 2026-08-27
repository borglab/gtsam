/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testWahbaFactor.cpp
 * @brief Tests for the chordal Wahba factor.
 * @author Avinash Subramanian
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/WahbaFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace wahba_fixture {

const Key kKey = 1;
const Rot3 aRb = Rot3::Expmap(Vector3(0.2, -0.1, 0.3));
const Unit3 bDirection(Point3(1.2, -0.7, 0.5));
const Unit3 measured_aDirection(Point3(0.3, 0.8, -0.4));

// Verifies the chordal residual, exact correspondence, and analytic Jacobian.
TEST(WahbaFactor, ErrorAndJacobian) {
  const auto model = noiseModel::Unit::Create(3);
  const WahbaFactor factor(kKey, bDirection, measured_aDirection, model);

  Matrix H;
  const Vector expected =
      aRb.rotate(bDirection.unitVector()) - measured_aDirection.unitVector();
  EXPECT(expected.norm() > 0.0);
  EXPECT(assert_equal(expected, factor.evaluateError(aRb, H)));
  EXPECT_LONGS_EQUAL(3, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  const WahbaFactor exactFactor(kKey, bDirection, aRb.rotate(bDirection),
                                model);
  EXPECT(assert_equal(Vector3::Zero(), exactFactor.evaluateError(aRb)));

  Values aRbs;
  aRbs.insert(kKey, aRb);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, aRbs, 1e-7, 1e-5);
}

// Verifies that a full information matrix determines the chordal factor cost.
TEST(WahbaFactor, FullInformationMatrix) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  const auto model = noiseModel::Gaussian::Information(information);
  const WahbaFactor factor(kKey, bDirection, measured_aDirection, model);

  Values aRbs;
  aRbs.insert(kKey, aRb);
  const Vector error = factor.evaluateError(aRb);
  const double expected = 0.5 * error.dot(information * error);
  EXPECT_DOUBLES_EQUAL(expected, factor.error(aRbs), 1e-9);
}

// Verifies exact nonlinear and QCQP costs at a nonzero Rot3 assignment.
TEST(WahbaFactor, QcqpError) {
  Matrix3 information;
  information << 4.0, 1.0, 0.5, 1.0, 3.0, 0.25, 0.5, 0.25, 2.0;
  NonlinearFactorGraph graph;
  graph.emplace_shared<WahbaFactor>(
      kKey, bDirection, measured_aDirection,
      noiseModel::Gaussian::Information(information));

  Values aRbs;
  aRbs.insert(kKey, aRb);
  const QcqpProblem qcqp(graph, 1);
  Values qcqpValues;
  InsertQcqpValue<Rot3, 1>(kKey, aRb, &qcqpValues);

  const double nonlinearError = graph.error(aRbs);
  EXPECT(nonlinearError > 0.0);
  EXPECT_DOUBLES_EQUAL(nonlinearError, qcqp.costs().error(qcqpValues), 1e-9);
}

// Verifies clear failures for unsupported QCQP configurations and noise models.
TEST(WahbaFactor, QcqpValidation) {
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  const WahbaFactor factor(kKey, bDirection, measured_aDirection,
                           noiseModel::Unit::Create(3));
  CHECK_EXCEPTION(factor.qcqpFactors(&costs, &constraints, 2),
                  std::invalid_argument);
  CHECK_EXCEPTION(factor.qcqpFactors(nullptr, &constraints, 1),
                  std::invalid_argument);

  const WahbaFactor nullModelFactor(kKey, bDirection, measured_aDirection,
                                    nullptr);
  CHECK_EXCEPTION(nullModelFactor.qcqpFactors(&costs, &constraints, 1),
                  std::runtime_error);

  const auto robustModel = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.0), noiseModel::Unit::Create(3));
  const WahbaFactor robustFactor(kKey, bDirection, measured_aDirection,
                                 robustModel);
  CHECK_EXCEPTION(robustFactor.qcqpFactors(&costs, &constraints, 1),
                  std::runtime_error);

  const WahbaFactor constrainedFactor(kKey, bDirection, measured_aDirection,
                                      noiseModel::Constrained::All(3));
  CHECK_EXCEPTION(constrainedFactor.qcqpFactors(&costs, &constraints, 1),
                  std::runtime_error);
}

// Verifies polymorphic cloning and measurement-aware equality.
TEST(WahbaFactor, CloneAndEquals) {
  const auto model = noiseModel::Unit::Create(3);
  const WahbaFactor factor(kKey, bDirection, measured_aDirection, model);
  const auto clone = factor.clone();
  EXPECT(factor.equals(*clone));
  EXPECT(!factor.equals(WahbaFactor(
      kKey, Unit3(Point3(0.9, -0.4, 0.2)), measured_aDirection, model)));
  EXPECT(!factor.equals(WahbaFactor(
      kKey, bDirection, Unit3(Point3(-0.2, 0.5, 0.9)), model)));

  NonlinearFactorGraph graph;
  graph.push_back(clone);
  const NonlinearFactorGraph graphClone = graph.clone();
  EXPECT(std::dynamic_pointer_cast<WahbaFactor>(graphClone.at(0)));
}

}  // namespace wahba_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
