/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 *
 * -------------------------------------------------------------------------- */

/**
 * @file testKnownLandmarkSquaredRangeFactor.cpp
 * @brief Tests for KnownLandmarkSquaredRangeFactor and AugmentedPoint QCQP
 *        traits.
 * @author Oluwatodimu Afolayan
 * @author Frederike Dümbgen
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/KnownLandmarkSquaredRangeFactor.h>

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <tuple>

using namespace gtsam;

/* ************************************************************************* */
namespace point2_fixture {

using Point = Eigen::Matrix<double, 2, 1>;
using Factor = KnownLandmarkSquaredRangeFactor<Point>;

const Key kKey = 3;

// Verifies the 2D residual, analytic Jacobian, and zero-residual case.
TEST(KnownLandmarkSquaredRangeFactor, Point2) {
  const Point wX(1.0, 2.0);
  const Point wL(4.0, 6.0);
  const double measuredRange = 4.0;
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, measuredRange, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wX, H);
  const Vector expected = Vector1(9.0);

  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(1, H.rows());
  EXPECT_LONGS_EQUAL(2, H.cols());

  Matrix expectedH(1, 2);
  expectedH << -6.0, -8.0;
  EXPECT(assert_equal(expectedH, H));

  const Factor exactFactor(kKey, wL, 5.0, model);
  EXPECT(assert_equal(Vector1::Zero(), exactFactor.evaluateError(wX)));

  Values values;
  values.insert(kKey, wX);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies that dynamically sized 2D landmarks are accepted after validation.
TEST(KnownLandmarkSquaredRangeFactor, AcceptsDynamicLandmark2D) {
  Vector wL(2);
  wL << 4.0, 6.0;
  const Point wX(1.0, 2.0);
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, 4.0, model);
  EXPECT(assert_equal(Vector1(9.0), factor.evaluateError(wX)));
}

// Verifies that short and long 2D landmark vectors throw before Eigen
// conversion.
TEST(KnownLandmarkSquaredRangeFactor, RejectsWrongLandmarkDimension2D) {
  const auto model = noiseModel::Unit::Create(1);
  const Vector tooShort = Vector::Zero(1);
  const Vector tooLong = Vector::Zero(3);

  CHECK_EXCEPTION(Factor(kKey, tooShort, 4.0, model), std::invalid_argument);
  CHECK_EXCEPTION(Factor(kKey, tooLong, 4.0, model), std::invalid_argument);
}

}  // namespace point2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace point3_fixture {

using Point = Eigen::Matrix<double, 3, 1>;
using Factor = KnownLandmarkSquaredRangeFactor<Point>;

const Key kKey = 4;

// Verifies the 3D residual, analytic Jacobian, and zero-residual case.
TEST(KnownLandmarkSquaredRangeFactor, Point3) {
  const Point wX(1.0, 2.0, 3.0);
  const Point wL(3.0, 5.0, 9.0);
  const double measuredRange = 5.0;
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, measuredRange, model);

  Matrix H;
  const Vector actual = factor.evaluateError(wX, H);
  const Vector expected = Vector1(24.0);

  EXPECT(assert_equal(expected, actual));
  EXPECT_LONGS_EQUAL(1, H.rows());
  EXPECT_LONGS_EQUAL(3, H.cols());

  Matrix expectedH(1, 3);
  expectedH << -4.0, -6.0, -12.0;
  EXPECT(assert_equal(expectedH, H));

  const Factor exactFactor(kKey, wL, 7.0, model);
  EXPECT(assert_equal(Vector1::Zero(), exactFactor.evaluateError(wX)));

  Values values;
  values.insert(kKey, wX);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Verifies cloning preserves factor data and equality detects changed data.
TEST(KnownLandmarkSquaredRangeFactor, CloneAndEquals) {
  const Point wL(3.0, 5.0, 9.0);
  const auto model = noiseModel::Unit::Create(1);

  const Factor factor(kKey, wL, 5.0, model);
  const auto clone = factor.clone();

  EXPECT(factor.equals(*clone));

  Point shiftedLandmark = wL;
  shiftedLandmark(0) += 0.1;
  EXPECT(!factor.equals(Factor(kKey, shiftedLandmark, 5.0, model)));

  EXPECT(!factor.equals(Factor(kKey, wL, 5.1, model)));

  NonlinearFactorGraph graph;
  graph.push_back(clone);
  const NonlinearFactorGraph graphClone = graph.clone();
  EXPECT(std::dynamic_pointer_cast<Factor>(graphClone.at(0)) != nullptr);
}

}  // namespace point3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace augmented_point3_fixture {

using Point = AugmentedPoint<3>;
using Traits = traits<Point>;

// Dynamic wrapper inputs must be validated before conversion to fixed storage.
TEST(AugmentedPoint, DynamicCoordinates) {
  const Vector coordinates = (Vector(3) << 1.0, -2.0, 3.0).finished();
  EXPECT(Point(coordinates).vector().isApprox(coordinates));
  CHECK_EXCEPTION(Point(Vector::Zero(2)), std::invalid_argument);
  CHECK_EXCEPTION(Point(Vector::Zero(4)), std::invalid_argument);
  CHECK_EXCEPTION(Point(Matrix::Zero(1, 3)), std::invalid_argument);
}

// Verifies the additive group operations and vector-space traits.
TEST(AugmentedPoint, VectorSpace) {
  const Point origin = Point::Identity();
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Point other(Eigen::Vector3d(4.0, 5.0, 6.0));

  // Compare Eigen vectors directly: avoids ambiguity between GTSAM's
  // assert_equal(Vector, Vector) and assert_equal(Matrix, Matrix) overloads.
  EXPECT(origin.vector().isZero(1e-12));
  EXPECT((point + other).vector().isApprox(
      point.vector() + other.vector(), 1e-12));
  EXPECT((point - other).vector().isApprox(
      point.vector() - other.vector(), 1e-12));
  EXPECT((-point).vector().isApprox(-point.vector(), 1e-12));
  EXPECT((2.0 * point).vector().isApprox(2.0 * point.vector(), 1e-12));
  EXPECT((point * 2.0).vector().isApprox(2.0 * point.vector(), 1e-12));

  const Vector tangent = Vector::Ones(3);
  const Point shifted = point + tangent;
  EXPECT(shifted.vector().isApprox(point.vector() + tangent, 1e-12));
  EXPECT(Traits::Local(point, shifted).isApprox(tangent, 1e-12));
  EXPECT(Traits::Retract(point, tangent).equals(shifted));

  CHECK_EXCEPTION(point + Vector::Zero(2), std::invalid_argument);
}

// Verifies the lifted coordinates and both quadratic equalities.
TEST(AugmentedPoint, QcqpLiftAndConstraints3D) {
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Matrix X = Traits::QcqpValue<1>(point);

  EXPECT_LONGS_EQUAL(5, X.rows());
  EXPECT_LONGS_EQUAL(1, X.cols());
  const Vector expected = (Vector(5) << 1.0, 1.0, -2.0, 3.0, 14.0).finished();
  EXPECT(expected.isApprox(X.col(0), 1e-12));

  const auto constraints = Traits::QcqpConstraints<1>();
  EXPECT_LONGS_EQUAL(2, constraints.size());
  if (constraints.size() != 2) return;

  // Each tuple represents X^T A X + 2 a^T X = b.
  for (const auto& constraint : constraints) {
    const Matrix& A = std::get<0>(constraint);
    const Vector& a = std::get<1>(constraint);
    const double b = std::get<2>(constraint);

    EXPECT_LONGS_EQUAL(5, A.rows());
    EXPECT_LONGS_EQUAL(5, A.cols());
    EXPECT_LONGS_EQUAL(0, a.size());
    EXPECT(A.isApprox(A.transpose(), 1e-12));
    EXPECT_DOUBLES_EQUAL(b, (X.transpose() * A * X)(0, 0), 1e-12);
  }

  const Matrix& A = std::get<0>(constraints.at(0));
  const Matrix& H = std::get<0>(constraints.at(1));
  EXPECT_DOUBLES_EQUAL(0.0, std::get<2>(constraints.at(0)), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, std::get<2>(constraints.at(1)), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(0, 4), 1e-12);
  EXPECT_DOUBLES_EQUAL(-0.5, A(4, 0), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, H(0, 0), 1e-12);

  // Changing u violates x^T x - h*u = 0.
  Matrix inconsistent = X;
  inconsistent(4, 0) += 1.0;
  EXPECT(std::abs((inconsistent.transpose() * A * inconsistent)(0, 0)) > 1e-9);
}

// Verifies round-trip recovery and global sign invariance.
TEST(AugmentedPoint, QcqpRoundTripAndSign) {
  const Point point(Eigen::Vector3d(1.0, -2.0, 3.0));
  const Matrix X = qcqpValue<3>(point);
  EXPECT(point.equals(fromQcqpValue<3>(X)));

  // h^2 = 1 permits h = -1; recovery must cancel the global sign.
  EXPECT(point.equals(fromQcqpValue<3>(-X)));

  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(4, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(5, 2)),
                  std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(Matrix::Zero(5, 1)),
                  std::invalid_argument);

  Matrix invalidH = X;
  invalidH(0, 0) = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(Traits::FromQcqpValue<1>(invalidH), std::invalid_argument);
}

// Verifies unsupported multi-column QCQP conversions are rejected.
TEST(AugmentedPoint, RejectsUnsupportedQcqpColumns) {
  const Point point(Eigen::Vector3d(1.0, 2.0, 3.0));

  CHECK_EXCEPTION(Traits::QcqpValue<2>(point), std::invalid_argument);
  CHECK_EXCEPTION(Traits::QcqpConstraints<2>(), std::invalid_argument);
  CHECK_EXCEPTION(Traits::FromQcqpValue<2>(Matrix::Zero(5, 1)),
                  std::invalid_argument);
}

}  // namespace augmented_point3_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace augmented_factor1_fixture {

using VectorType = Eigen::Matrix<double, 1, 1>;
using Point = AugmentedPoint<1>;
using Factor = KnownLandmarkSquaredRangeFactor<Point>;

const Key kKey = 11;

// C++ dimensions remain unrestricted by the 2D/3D wrapper specializations.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedPoint1D) {
  VectorType coordinates;
  coordinates << 2.0;
  VectorType landmark;
  landmark << 5.0;
  const Point point(coordinates);
  const auto model = noiseModel::Unit::Create(1);
  const Factor factor(kKey, landmark, 2.0, model);

  Matrix H;
  EXPECT(assert_equal(Vector1(5.0), factor.evaluateError(point, H)));
  EXPECT_LONGS_EQUAL(1, H.rows());
  EXPECT_LONGS_EQUAL(1, H.cols());
  EXPECT_DOUBLES_EQUAL(-6.0, H(0, 0), 1e-12);

  NonlinearFactorGraph graph;
  graph.push_back(std::make_shared<Factor>(factor));
  Values values;
  values.insert(kKey, point);

  const QcqpProblem qcqp(graph);
  Values lifted;
  insertQcqpValue<1>(kKey, point, lifted);
  EXPECT_LONGS_EQUAL(1, qcqp.costs().size());
  EXPECT_LONGS_EQUAL(2, qcqp.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(graph.error(values), qcqp.costs().error(lifted),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, qcqp.eConstraints().violationNorm(lifted),
                       1e-12);
}

}  // namespace augmented_factor1_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace augmented_factor2_fixture {

using VectorType = Eigen::Vector2d;
using Point = AugmentedPoint<2>;
using Factor = KnownLandmarkSquaredRangeFactor<Point>;
using EigenFactor = KnownLandmarkSquaredRangeFactor<VectorType>;

const Key kKey = 12;

// Verifies that both state types give the same residual and tangent Jacobian.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedPoint2DResidualAndJacobian) {
  const VectorType coordinates(1.0, 2.0);
  const VectorType landmark(4.0, 6.0);
  const Point point(coordinates);
  const auto model = noiseModel::Unit::Create(1);

  const Factor augmented(kKey, landmark, 4.0, model);
  const EigenFactor eigen(kKey, landmark, 4.0, model);

  Matrix augmentedH, eigenH;
  const Vector augmentedError = augmented.evaluateError(point, augmentedH);
  const Vector eigenError = eigen.evaluateError(coordinates, eigenH);

  EXPECT(assert_equal(Vector1(9.0), augmentedError));
  EXPECT(assert_equal(eigenError, augmentedError));
  EXPECT(assert_equal(eigenH, augmentedH));
  EXPECT_LONGS_EQUAL(1, augmentedH.rows());
  EXPECT_LONGS_EQUAL(2, augmentedH.cols());

  Values values;
  values.insert(kKey, point);
  EXPECT_CORRECT_FACTOR_JACOBIANS(augmented, values, 1e-7, 1e-5);
}

// An AugmentedPoint supplied as the fixed landmark must produce the same
// factor as one constructed with ordinary Eigen coordinates.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedLandmarkConstructor2D) {
  const VectorType landmark(4.0, 6.0);
  const Point point(VectorType(1.0, 2.0));
  const auto model = noiseModel::Unit::Create(1);
  const Factor fromVector(kKey, landmark, 4.0, model);
  const Factor fromAugmented(kKey, Point(landmark), 4.0, model);

  EXPECT(fromVector.equals(fromAugmented));
  EXPECT(assert_equal(fromVector.evaluateError(point),
                      fromAugmented.evaluateError(point)));
}

// The whitened lifted quadratic cost must match the nonlinear information-
// weighted least-squares cost, including its homogeneous sign ambiguity.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedPoint2DWeightedQcqpCost) {
  const Point point(VectorType(1.0, 2.0));
  const VectorType landmark(4.0, 6.0);
  Matrix information(1, 1);
  information << 4.0;
  const auto model = noiseModel::Gaussian::Information(information);

  NonlinearFactorGraph graph;
  graph.emplace_shared<Factor>(kKey, landmark, 4.0, model);

  Values nonlinearValues;
  nonlinearValues.insert(kKey, point);
  const QcqpProblem qcqp(graph);
  Values liftedValues;
  insertQcqpValue<2>(kKey, point, liftedValues);

  EXPECT_LONGS_EQUAL(1, qcqp.costs().size());
  EXPECT_LONGS_EQUAL(2, qcqp.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(162.0, graph.error(nonlinearValues), 1e-12);
  EXPECT_DOUBLES_EQUAL(graph.error(nonlinearValues),
                       qcqp.costs().error(liftedValues), 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0,
                       qcqp.eConstraints().violationNorm(liftedValues),
                       1e-12);

  // X and -X represent the same original point and quadratic cost.
  const Matrix negativeLift = -traits<Point>::QcqpValue<1>(point);
  Values negativeValues;
  negativeValues.insert(kKey, negativeLift);
  EXPECT_DOUBLES_EQUAL(qcqp.costs().error(liftedValues),
                       qcqp.costs().error(negativeValues), 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0,
                       qcqp.eConstraints().violationNorm(negativeValues),
                       1e-12);

  const Values recovered = extractQcqpValues<2>(negativeValues);
  EXPECT(recovered.at<Point>(kKey).equals(point));
}

// The Eigen-state factor remains usable for nonlinear optimization, while
// conversion to QCQP requires the opt-in AugmentedPoint state.
TEST(KnownLandmarkSquaredRangeFactor, RejectsEigenStateQcqpConversion) {
  const VectorType landmark(4.0, 6.0);
  const auto model = noiseModel::Unit::Create(1);
  const EigenFactor factor(kKey, landmark, 4.0, model);
  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  CHECK_EXCEPTION(factor.qcqpFactors(&costs, &constraints),
                  std::runtime_error);
}

}  // namespace augmented_factor2_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace augmented_factor3_fixture {

using VectorType = Eigen::Vector3d;
using Point = AugmentedPoint<3>;
using Factor = KnownLandmarkSquaredRangeFactor<Point>;

const Key kKey = 13;

// Checks the 3D tangent Jacobian and the factor's nonlinear graph interface.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedPoint3DResidualAndJacobian) {
  const Point point(VectorType(1.0, 2.0, 3.0));
  const VectorType landmark(3.0, 5.0, 9.0);
  const auto model = noiseModel::Unit::Create(1);
  const Factor factor(kKey, landmark, 5.0, model);

  Matrix H;
  EXPECT(assert_equal(Vector1(24.0), factor.evaluateError(point, H)));
  Matrix expectedH(1, 3);
  expectedH << -4.0, -6.0, -12.0;
  EXPECT(assert_equal(expectedH, H));

  Values values;
  values.insert(kKey, point);
  EXPECT_DOUBLES_EQUAL(0.5 * 24.0 * 24.0, factor.error(values), 1e-12);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

// Multiple measurements on the same state must contribute separate costs but
// must not duplicate the two shared point-lift constraints.
TEST(KnownLandmarkSquaredRangeFactor, MultipleAugmentedQcqpFactors3D) {
  const Point point(VectorType(1.0, 2.0, 3.0));
  const auto model = noiseModel::Unit::Create(1);

  NonlinearFactorGraph graph;
  graph.emplace_shared<Factor>(kKey, VectorType(3.0, 5.0, 9.0), 5.0,
                               model);
  graph.emplace_shared<Factor>(kKey, VectorType(-1.0, 2.0, 4.0), 3.0,
                               model);

  Values nonlinearValues;
  nonlinearValues.insert(kKey, point);
  const QcqpProblem qcqp(graph);
  Values qcqpValues;
  InsertQcqpValue<Point, 1>(kKey, point, &qcqpValues);

  EXPECT_LONGS_EQUAL(2, qcqp.costs().size());
  EXPECT_LONGS_EQUAL(2, qcqp.eConstraints().size());
  EXPECT_DOUBLES_EQUAL(graph.error(nonlinearValues),
                       qcqp.costs().error(qcqpValues), 1e-10);
  EXPECT_DOUBLES_EQUAL(0.0,
                       qcqp.eConstraints().violationNorm(qcqpValues),
                       1e-12);
}

// Validate qcqpFactors error handling without an SDP solve.
TEST(KnownLandmarkSquaredRangeFactor, AugmentedQcqpRejectsInvalidInputs) {
  const VectorType landmark(3.0, 5.0, 9.0);
  const auto model = noiseModel::Unit::Create(1);
  const Factor factor(kKey, landmark, 5.0, model);

  NonlinearFactorGraph costs;
  NonlinearEqualityConstraints constraints;
  CHECK_EXCEPTION(factor.qcqpFactors(nullptr, &constraints),
                  std::invalid_argument);
  CHECK_EXCEPTION(factor.qcqpFactors(&costs, nullptr),
                  std::invalid_argument);
  CHECK_EXCEPTION(factor.qcqpFactors(&costs, &constraints, 2),
                  std::invalid_argument);

  const Factor missingNoise(kKey, landmark, 5.0, SharedNoiseModel());
  CHECK_EXCEPTION(missingNoise.qcqpFactors(&costs, &constraints),
                  std::runtime_error);
  CHECK_EXCEPTION(Factor(kKey, landmark, -1.0, model),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      Factor(kKey, landmark, std::numeric_limits<double>::infinity(), model),
                  std::invalid_argument);
}

}  // namespace augmented_factor3_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
