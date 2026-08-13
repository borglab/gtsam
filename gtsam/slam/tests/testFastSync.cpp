/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/** @file testFastSync.cpp */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SL4.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FastSync.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <Eigen/QR>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

using namespace gtsam;

/* ************************************************************************* */
namespace noiseless_fixture {

constexpr Key k0 = 10, k1 = 42, k2 = 77;

template <class T>
NonlinearFactorGraph triangleGraph(const T& value0, const T& value1,
                                   const T& value2) {
  const auto model = noiseModel::Isotropic::Sigma(T::dimension, 0.1);
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<T>>(k0, k1, value0.between(value1), model);
  graph.emplace_shared<BetweenFactor<T>>(k1, k2, value1.between(value2), model);
  graph.emplace_shared<BetweenFactor<T>>(k2, k0, value2.between(value0), model);
  graph.emplace_shared<PriorFactor<T>>(k0, value0, model);
  return graph;
}

template <class T>
bool checkTriangle(const T& value0, const T& value1, const T& value2,
                   const double tolerance = 1e-7) {
  const Values values = fastSync<T>(triangleGraph(value0, value1, value2));
  return traits<T>::Equals(value0, values.at<T>(k0), tolerance) &&
         traits<T>::Equals(value1, values.at<T>(k1), tolerance) &&
         traits<T>::Equals(value2, values.at<T>(k2), tolerance);
}

// Verifies exact synchronization and prior alignment for Rot2.
TEST(FastSync, Rot2Noiseless) {
  EXPECT(checkTriangle(Rot2::fromAngle(0.2), Rot2::fromAngle(0.7),
                       Rot2::fromAngle(-0.4)));
}

// Verifies exact synchronization and prior alignment for Rot3.
TEST(FastSync, Rot3Noiseless) {
  EXPECT(checkTriangle(Rot3::Expmap(Vector3{0.1, -0.2, 0.05}),
                       Rot3::Expmap(Vector3{-0.3, 0.1, 0.2}),
                       Rot3::Expmap(Vector3{0.2, 0.25, -0.1})));
}

// Verifies Frobenius between/prior factors produce the same aligned Rot3
// initialization as their tangent-residual counterparts.
TEST(FastSync, FrobeniusRot3Factors) {
  const Rot3 value0 = Rot3::Expmap(Vector3{0.1, -0.2, 0.05});
  const Rot3 value1 = Rot3::Expmap(Vector3{-0.3, 0.1, 0.2});
  const Rot3 value2 = Rot3::Expmap(Vector3{0.2, 0.25, -0.1});
  const auto model = noiseModel::Isotropic::Sigma(Rot3::dimension, 0.1);

  NonlinearFactorGraph graph;
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
      k0, k1, value0.between(value1), model);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
      k1, k2, value1.between(value2), model);
  graph.emplace_shared<FrobeniusBetweenFactor<Rot3>>(
      k2, k0, value2.between(value0), model);
  graph.emplace_shared<FrobeniusPrior<Rot3>>(k0, value0.matrix(), model);

  const Values actual = fastSync<Rot3>(graph);
  EXPECT(assert_equal(value0, actual.at<Rot3>(k0), 1e-7));
  EXPECT(assert_equal(value1, actual.at<Rot3>(k1), 1e-7));
  EXPECT(assert_equal(value2, actual.at<Rot3>(k2), 1e-7));
}

// Verifies exact synchronization and prior alignment for Pose2.
TEST(FastSync, Pose2Noiseless) {
  EXPECT(checkTriangle(Pose2(1.0, -2.0, 0.2), Pose2(2.0, 0.5, 0.7),
                       Pose2(-1.0, 1.5, -0.4)));
}

// Verifies exact synchronization and prior alignment for Pose3.
TEST(FastSync, Pose3Noiseless) {
  EXPECT(checkTriangle(
      Pose3(Rot3::Expmap(Vector3{0.1, -0.2, 0.05}), Point3(1.0, -2.0, 0.5)),
      Pose3(Rot3::Expmap(Vector3{-0.3, 0.1, 0.2}), Point3(2.0, 0.5, -1.0)),
      Pose3(Rot3::Expmap(Vector3{0.2, 0.25, -0.1}), Point3(-1.0, 1.5, 2.0))));
}

// Verifies exact synchronization and prior alignment for Similarity2.
TEST(FastSync, Similarity2Noiseless) {
  EXPECT(checkTriangle(
      Similarity2(Rot2::fromAngle(0.2), Point2(1.0, -2.0), 1.1),
      Similarity2(Rot2::fromAngle(0.7), Point2(2.0, 0.5), 0.9),
      Similarity2(Rot2::fromAngle(-0.4), Point2(-1.0, 1.5), 1.3), 1e-6));
}

// Verifies exact synchronization and prior alignment for Similarity3.
TEST(FastSync, Similarity3Noiseless) {
  EXPECT(checkTriangle(Similarity3(Rot3::Expmap(Vector3{0.1, -0.2, 0.05}),
                                   Point3(1.0, -2.0, 0.5), 1.1),
                       Similarity3(Rot3::Expmap(Vector3{-0.3, 0.1, 0.2}),
                                   Point3(2.0, 0.5, -1.0), 0.9),
                       Similarity3(Rot3::Expmap(Vector3{0.2, 0.25, -0.1}),
                                   Point3(-1.0, 1.5, 2.0), 1.3),
                       1e-6));
}

// Verifies exact synchronization and prior alignment for SL4.
TEST(FastSync, SL4Noiseless) {
  EXPECT(checkTriangle(SL4::Expmap(Vector::LinSpaced(15, 0.001, 0.015)),
                       SL4::Expmap(Vector::LinSpaced(15, -0.01, 0.02)),
                       SL4::Expmap(Vector::LinSpaced(15, 0.02, -0.01)), 1e-6));
}

// Verifies the class and convenience APIs accept generated and custom
// orderings.
TEST(FastSync, OrderingSelection) {
  const Rot2 value0 = Rot2::fromAngle(0.2);
  const Rot2 value1 = Rot2::fromAngle(0.7);
  const Rot2 value2 = Rot2::fromAngle(-0.4);
  const NonlinearFactorGraph graph = triangleGraph(value0, value1, value2);

  const FastSync<Rot2> solver(graph);
  const Values relaxed = solver.solve(Ordering::COLAMD);
  const Values classResult = solver.projectAndAlign(relaxed);
  const Values functionResult = fastSync<Rot2>(graph, Ordering::COLAMD);
  const Ordering customOrdering{k2, k0, k1};
  const Values customResult = fastSync<Rot2>(graph, customOrdering);

  for (const auto& [key, expected] :
       std::map<Key, Rot2>{{k0, value0}, {k1, value1}, {k2, value2}}) {
    EXPECT(assert_equal(expected, classResult.at<Rot2>(key), 1e-7));
    EXPECT(assert_equal(expected, functionResult.at<Rot2>(key), 1e-7));
    EXPECT(assert_equal(expected, customResult.at<Rot2>(key), 1e-7));
  }
}

}  // namespace noiseless_fixture

/* ************************************************************************* */
namespace linear_fixture {

// Verifies structured back-substitution against an independent dense QR solve.
TEST(FastSync, DenseQrAgreement) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot2>>(
      2, 7, Rot2::fromAngle(0.4), noiseModel::Isotropic::Sigma(1, 0.7));
  graph.emplace_shared<BetweenFactor<Rot2>>(
      7, 9, Rot2::fromAngle(-0.2), noiseModel::Isotropic::Sigma(1, 1.2));
  graph.emplace_shared<BetweenFactor<Rot2>>(
      9, 2, Rot2::fromAngle(-0.1), noiseModel::Isotropic::Sigma(1, 0.9));
  const FastSync<Rot2> solver(graph);
  const auto actual = solver.solve();
  // Local measurement record for the independent dense QR reference.
  struct M {
    Key key1;
    Key key2;
    Matrix2 measured;
    double sigma;
  };
  const std::vector<M> measurements{
      {2, 7, Rot2::fromAngle(0.4).matrix(), 0.7},
      {7, 9, Rot2::fromAngle(-0.2).matrix(), 1.2},
      {9, 2, Rot2::fromAngle(-0.1).matrix(), 0.9}};

  Key gaugeKey = 0;
  double smallestIdentityError = std::numeric_limits<double>::infinity();
  for (const Key key : actual.keys()) {
    const Matrix2 matrix = actual.at<Matrix2>(key);
    const double error = (matrix - Matrix2::Identity()).norm();
    if (error < smallestIdentityError) {
      smallestIdentityError = error;
      gaugeKey = key;
    }
  }
  EXPECT_DOUBLES_EQUAL(0.0, smallestIdentityError, 1e-12);

  const std::vector<Key> keys{2, 7, 9};
  std::vector<Key> freeKeys;
  for (const Key key : keys) {
    if (key != gaugeKey) freeKeys.push_back(key);
  }
  const std::map<Key, size_t> positions{{freeKeys[0], 0}, {freeKeys[1], 1}};
  Matrix A = Matrix::Zero(6, 4);
  Matrix B = Matrix::Zero(6, 2);
  size_t row = 0;
  for (const auto& measurement : measurements) {
    const Matrix2 firstBlock =
        -measurement.measured.transpose() / measurement.sigma;
    const Matrix2 secondBlock = Matrix2::Identity() / measurement.sigma;
    if (measurement.key1 == gaugeKey) {
      B.block<2, 2>(row, 0) -= firstBlock;
    } else {
      A.block<2, 2>(row, 2 * positions.at(measurement.key1)) = firstBlock;
    }
    if (measurement.key2 == gaugeKey) {
      B.block<2, 2>(row, 0) -= secondBlock;
    } else {
      A.block<2, 2>(row, 2 * positions.at(measurement.key2)) = secondBlock;
    }
    row += 2;
  }
  const Matrix dense = A.colPivHouseholderQr().solve(B);

  for (const Key key : keys) {
    const Matrix2 expected =
        key == gaugeKey
            ? Matrix2::Identity()
            : Matrix2(dense.block<2, 2>(2 * positions.at(key), 0).transpose());
    EXPECT_DOUBLES_EQUAL(0.0, (expected - actual.at<Matrix2>(key)).norm(),
                         1e-9);
  }
}

// Matches a selected rounded SO(2) result from the Python I_Fast_SCG reference.
TEST(FastSync, PythonReferenceGoldenRot2) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot2>>(
      0, 1, Rot2::fromAngle(0.2), noiseModel::Isotropic::Sigma(1, 1.0));
  graph.emplace_shared<BetweenFactor<Rot2>>(
      1, 2, Rot2::fromAngle(-0.35), noiseModel::Isotropic::Sigma(1, 0.5));
  graph.emplace_shared<BetweenFactor<Rot2>>(
      2, 0, Rot2::fromAngle(0.1), noiseModel::Isotropic::Sigma(1, 2.0));
  const auto actual = FastSync<Rot2>(graph).solve();

  const Rot2 rounded0 = Rot2::ClosestTo(actual.at<Matrix2>(0));
  const Rot2 rounded1 = Rot2::ClosestTo(actual.at<Matrix2>(1));
  EXPECT_DOUBLES_EQUAL(0.138097218014, rounded0.theta(), 1e-10);
  EXPECT_DOUBLES_EQUAL(0.347619902410, rounded1.theta(), 1e-10);
  EXPECT_DOUBLES_EQUAL(
      0.0, (Matrix2::Identity() - actual.at<Matrix2>(2)).norm(), 1e-12);
}

// Verifies that a high-confidence edge dominates a conflicting weak edge.
TEST(FastSync, WeightedMeasurements) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Rot2>>(
      0, 1, Rot2::fromAngle(0.2), noiseModel::Isotropic::Sigma(1, 0.01));
  graph.emplace_shared<BetweenFactor<Rot2>>(
      0, 1, Rot2::fromAngle(1.0), noiseModel::Isotropic::Sigma(1, 10.0));
  graph.emplace_shared<PriorFactor<Rot2>>(0, Rot2(),
                                          noiseModel::Isotropic::Sigma(1, 0.1));
  const Values result = fastSync<Rot2>(graph);
  EXPECT_DOUBLES_EQUAL(0.2, result.at<Rot2>(1).theta(), 1e-3);
}

}  // namespace linear_fixture

/* ************************************************************************* */
namespace projection_fixture {

// Verifies all built-in projectors produce valid group elements.
TEST(FastSync, BuiltInProjections) {
  Matrix2 rawRotation2{{1.0, -0.2}, {0.3, 0.9}};
  EXPECT_DOUBLES_EQUAL(
      1.0,
      FastSyncProjection<Rot2>::project(rawRotation2).matrix().determinant(),
      1e-9);

  Matrix3 rawRotation3 = Matrix3::Identity();
  rawRotation3(0, 1) = 0.2;
  EXPECT_DOUBLES_EQUAL(
      1.0,
      FastSyncProjection<Rot3>::project(rawRotation3).matrix().determinant(),
      1e-9);

  Matrix3 rawSimilarity2 = Matrix3::Identity();
  rawSimilarity2(2, 2) = -0.5;
  EXPECT(FastSyncProjection<Similarity2>::project(rawSimilarity2).scale() >
         0.0);

  Matrix4 rawSimilarity3 = Matrix4::Identity();
  rawSimilarity3(3, 3) = -0.25;
  EXPECT(FastSyncProjection<Similarity3>::project(rawSimilarity3).scale() >
         0.0);

  const SL4 singular = FastSyncProjection<SL4>::project(Matrix4::Zero());
  EXPECT(singular.equals(SL4::Identity(), 1e-12));
}

}  // namespace projection_fixture

/* ************************************************************************* */
namespace validation_fixture {

// Verifies custom orderings must cover every graph key exactly once.
TEST(FastSync, InvalidCustomOrdering) {
  NonlinearFactorGraph graph;
  const auto model = noiseModel::Unit::Create(1);
  graph.emplace_shared<BetweenFactor<Rot2>>(0, 1, Rot2(), model);
  const FastSync<Rot2> solver(graph);

  CHECK_EXCEPTION(solver.solve(Ordering{0}), std::invalid_argument);
  CHECK_EXCEPTION(solver.solve(Ordering{0, 0}), std::invalid_argument);
  CHECK_EXCEPTION(solver.solve(Ordering{0, 2}), std::invalid_argument);
}

// Verifies empty graphs fail early and disconnected graphs fail in elimination.
TEST(FastSync, InvalidGraphStructure) {
  CHECK_EXCEPTION(fastSync<Rot2>(NonlinearFactorGraph()),
                  std::invalid_argument);

  NonlinearFactorGraph disconnected;
  const auto model = noiseModel::Unit::Create(1);
  disconnected.emplace_shared<BetweenFactor<Rot2>>(0, 1, Rot2(), model);
  disconnected.emplace_shared<BetweenFactor<Rot2>>(2, 3, Rot2(), model);
  const FastSync<Rot2> solver(disconnected);
  try {
    solver.solve();
    CHECK(false);
  } catch (const IndeterminateSystemException&) {
    // The nearby variable is ordering-dependent, but the failure occurs in
    // the linear solve after the constructor accepts both components.
  } catch (...) {
    CHECK(false);
  }
}

// Verifies anisotropic, constrained, and robust models are rejected.
TEST(FastSync, InvalidNoiseModels) {
  NonlinearFactorGraph anisotropic;
  anisotropic.emplace_shared<BetweenFactor<Rot3>>(
      0, 1, Rot3(), noiseModel::Diagonal::Sigmas(Vector3{1.0, 2.0, 1.0}));
  CHECK_EXCEPTION(fastSync<Rot3>(anisotropic), std::invalid_argument);

  NonlinearFactorGraph constrained;
  constrained.emplace_shared<BetweenFactor<Rot2>>(
      0, 1, Rot2(), noiseModel::Constrained::All(1));
  CHECK_EXCEPTION(fastSync<Rot2>(constrained), std::invalid_argument);

  NonlinearFactorGraph robust;
  robust.emplace_shared<BetweenFactor<Rot2>>(
      0, 1, Rot2(),
      noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0),
                                 noiseModel::Unit::Create(1)));
  CHECK_EXCEPTION(fastSync<Rot2>(robust), std::invalid_argument);
}

// Verifies that more than one matching prior is rejected.
TEST(FastSync, MultiplePriors) {
  NonlinearFactorGraph graph;
  const auto model = noiseModel::Unit::Create(1);
  graph.emplace_shared<BetweenFactor<Rot2>>(0, 1, Rot2(), model);
  graph.emplace_shared<PriorFactor<Rot2>>(0, Rot2(), model);
  graph.emplace_shared<PriorFactor<Rot2>>(1, Rot2(), model);
  CHECK_EXCEPTION(fastSync<Rot2>(graph), std::invalid_argument);
}

}  // namespace validation_fixture
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
