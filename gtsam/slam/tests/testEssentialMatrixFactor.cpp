/*
 * @file testEssentialMatrixFactor.cpp
 * @brief Test EssentialMatrixFactor class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/slam/EssentialMatrixFactor.h>
#include <gtsam/base/LieScalar.h>

namespace gtsam {

/**
 * Binary factor that optimizes for E and inverse depth: assumes measurement
 * in image 1 is perfect, and returns re-projection error in image 2
 */
class EssentialMatrixFactor2: public NoiseModelFactor2<EssentialMatrix,
    LieScalar> {

  Point2 pA_, pB_; ///< Measurements in image A and B

  typedef NoiseModelFactor2<EssentialMatrix, LieScalar> Base;

public:

  /// Constructor
  EssentialMatrixFactor2(Key key1, Key key2, const Point2& pA, const Point2& pB,
      const SharedNoiseModel& model) :
      Base(model, key1, key2), pA_(pA), pB_(pB) {
  }

  /// print
  virtual void print(const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor2 with measurements\n  ("
        << pA_.vector().transpose() << ")' and (" << pB_.vector().transpose()
        << ")'" << std::endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(const EssentialMatrix& E, const LieScalar& d,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    if (H1)
      *H1 = zeros(2, 5);
    if (H2)
      *H2 = zeros(2, 1);
    return (Vector(2) << 0,0);
  }

};

} // gtsam

#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

const string filename = findExampleDataFile("5pointExample1.txt");
SfM_data data;
bool readOK = readBAL(filename, data);
Rot3 aRb = data.cameras[1].pose().rotation();
Point3 aTb = data.cameras[1].pose().translation();

Point2 pA(size_t i) {
  return data.tracks[i].measurements[0].second;
}
Point2 pB(size_t i) {
  return data.tracks[i].measurements[1].second;
}
Vector vA(size_t i) {
  return EssentialMatrix::Homogeneous(pA(i));
}
Vector vB(size_t i) {
  return EssentialMatrix::Homogeneous(pB(i));
}

//*************************************************************************
TEST (EssentialMatrixFactor, testData) {
  CHECK(readOK);

  // Check E matrix
  Matrix expected(3, 3);
  expected << 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0;
  Matrix aEb_matrix = skewSymmetric(aTb.x(), aTb.y(), aTb.z()) * aRb.matrix();
  EXPECT(assert_equal(expected, aEb_matrix,1e-8));

  // Check some projections
  EXPECT(assert_equal(Point2(0,0),pA(0),1e-8));
  EXPECT(assert_equal(Point2(0,0.1),pB(0),1e-8));
  EXPECT(assert_equal(Point2(0,-1),pA(4),1e-8));
  EXPECT(assert_equal(Point2(-1,0.2),pB(4),1e-8));

  // Check homogeneous version
  EXPECT(assert_equal((Vector(3) << -1,0.2,1),vB(4),1e-8));

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA(i).transpose() * aEb_matrix * vB(i), 1e-8);

  // Check epipolar constraint
  EssentialMatrix trueE(aRb, aTb);
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA(i),vB(i)), 1e-7);
}

//*************************************************************************
TEST (EssentialMatrixFactor, factor) {
  EssentialMatrix trueE(aRb, aTb);
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(1);

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor factor(1, pA(i), pB(i), model);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix Hactual;
    Vector actual = factor.evaluateError(trueE, Hactual);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected;
    Hexpected = numericalDerivative11<EssentialMatrix>(
        boost::bind(&EssentialMatrixFactor::evaluateError, &factor, _1,
            boost::none), trueE);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected, Hactual, 1e-8));
  }
}

//*************************************************************************
TEST (EssentialMatrixFactor, fromConstraints) {
  // Here we want to optimize directly on essential matrix constraints
  // Yi Ma's algorithm (Ma01ijcv) is a bit cumbersome to implement,
  // but GTSAM does the equivalent anyway, provided we give the right
  // factors. In this case, the factors are the constraints.

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,
      0.01);
  for (size_t i = 0; i < 5; i++)
    graph.add(EssentialMatrixFactor(1, pA(i), pB(i), model));

  // Check error at ground truth
  Values truth;
  EssentialMatrix trueE(aRb, aTb);
  truth.insert(1, trueE);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE = trueE.retract(
      (Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1));
  initial.insert(1, initialE);
  EXPECT_DOUBLES_EQUAL(640, graph.error(initial), 1e-2);

  // Optimize
  LevenbergMarquardtParams parameters;
  LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
  Values result = optimizer.optimize();

  // Check result
  EssentialMatrix actual = result.at<EssentialMatrix>(1);
  EXPECT(assert_equal(trueE, actual,1e-1));

  // Check error at result
  EXPECT_DOUBLES_EQUAL(0, graph.error(result), 1e-4);

  // Check errors individually
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, actual.error(vA(i),vB(i)), 1e-6);

}

//*************************************************************************
TEST (EssentialMatrixFactor2, factor) {
  EssentialMatrix trueE(aRb, aTb);
  LieScalar d(5);
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(1);

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor2 factor(100, i, pA(i), pB(i), model);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix Hactual1, Hactual2;
    Vector actual = factor.evaluateError(trueE, d, Hactual1, Hactual2);
    EXPECT(assert_equal(expected, actual, 1e-7));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix Hexpected1, Hexpected2;
    boost::function<Vector(const EssentialMatrix&, const LieScalar&)> f =
        boost::bind(&EssentialMatrixFactor2::evaluateError, &factor, _1, _2,
            boost::none, boost::none);
    Hexpected1 = numericalDerivative21<EssentialMatrix>(f, trueE, d);
    Hexpected2 = numericalDerivative22<EssentialMatrix>(f, trueE, d);

    // Verify the Jacobian is correct
    EXPECT(assert_equal(Hexpected1, Hactual1, 1e-8));
    EXPECT(assert_equal(Hexpected2, Hactual2, 1e-8));
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

