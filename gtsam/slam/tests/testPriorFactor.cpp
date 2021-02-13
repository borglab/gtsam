/**
 * @file   testPriorFactor.cpp
 * @brief  Test PriorFactor
 * @author Frank Dellaert
 * @date   Nov 4, 2014
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

// Constructor scalar
TEST(PriorFactor, ConstructorScalar) {
  SharedNoiseModel model;
  PriorFactor<double> factor(1, 1.0, model);
}

// Constructor vector3
TEST(PriorFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  PriorFactor<Vector3> factor(1, Vector3(1,2,3), model);
}

// Constructor dynamic sized vector
TEST(PriorFactor, ConstructorDynamicSizeVector) {
  Vector v(5); v << 1, 2, 3, 4, 5;
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  PriorFactor<Vector> factor(1, v, model);
}

// Test negative sigmas
TEST(PriorFactor, NegativeSigmas) {
  auto noise1 = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
  auto noise2 = gtsam::noiseModel::Isotropic::Sigma(2, -2.0);

  const Values initValues = {{1, genericValue(Point2(0, 0))}};

  // First, test using positive sigmas:
  {
    NonlinearFactorGraph fg;
    fg.emplace_shared<PriorFactor<Point2>>(1, Point2(0, 0), noise1);
    fg.emplace_shared<PriorFactor<Point2>>(1, Point2(1, 1), noise1);
    const Point2 result =
        GaussNewtonOptimizer(fg, initValues).optimize().at<Point2>(1);
    EXPECT_DOUBLES_EQUAL(0.5, result.x(), 1e-3);
    EXPECT_DOUBLES_EQUAL(0.5, result.y(), 1e-3);
  }
  // Second, test using negative sigmas:
  {
    NonlinearFactorGraph fg;
    fg.emplace_shared<PriorFactor<Point2>>(1, Point2(0, 0), noise1);
    fg.emplace_shared<PriorFactor<Point2>>(1, Point2(1, 1), noise2);
    const Point2 result =
        GaussNewtonOptimizer(fg, initValues).optimize().at<Point2>(1);
    EXPECT_DOUBLES_EQUAL(0.5, result.x(), 1e-3);
    EXPECT_DOUBLES_EQUAL(0.5, result.y(), 1e-3);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
