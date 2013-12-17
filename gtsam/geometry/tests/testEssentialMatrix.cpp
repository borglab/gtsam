/*
 * @file testEssentialMatrix.cpp
 * @brief Test EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

//#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>
#include <vector>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

/**
 * An essential matrix is like a Pose3, except with translation up to scale
 * It is named after the 3*3 matrix aEb = [aTb]x aRb from computer vision,
 * but here we choose instead to parameterize it as a (Rot3,Sphere2) pair.
 * We can then non-linearly optimize immediately on this 5-dimensional manifold.
 */
class EssentialMatrix: public DerivedValue<EssentialMatrix> {

private:

  Rot3 aRb_; ///< Rotation between a and b
  Sphere2 aTb_; ///< translation direction from a to b
  Matrix E_; ///< Essential matrix

  /// Static function to convert Point2 to 3D
  static Point3 Upgrade(const Point2& p) {
    return Point3(p.x(), p.y(), 1);
  }

public:

  /// Static function to convert Point2 to homogeneous coordinates
  static Vector Homogeneous(const Point2& p) {
    return Vector(3) << p.x(), p.y(), 1;
  }

  /// @name Constructors and named constructors
  /// @{

  /// Construct from rotation and translation
  EssentialMatrix(const Rot3& aRb, const Sphere2& aTb) :
      aRb_(aRb), aTb_(aTb), E_(aTb_.skew() * aRb_.matrix()) {
  }

  /// @}

  /// @name Value
  /// @{

  /// Return the dimensionality of the tangent space
  virtual size_t dim() const {
    return 5;
  }

  /// Retract delta to manifold
  virtual EssentialMatrix retract(const Vector& xi) const {
    assert(xi.size()==5);
    Vector3 omega(sub(xi, 0, 3));
    Vector2 z(sub(xi, 3, 5));
    Rot3 R = aRb_.retract(omega);
    Sphere2 t = aTb_.retract(z);
    return EssentialMatrix(R, t);
  }

  /// Compute the coordinates in the tangent space
  virtual Vector localCoordinates(const EssentialMatrix& value) const {
    return Vector(5) << 0, 0, 0, 0, 0;
  }

  /// @}

  /// @name Testable
  /// @{

  /// print with optional string
  void print(const string& s) const {
    cout << s;
    aRb_.print("R:\n");
    aTb_.print("d: ");
  }

  /// assert equality up to a tolerance
  bool equals(const EssentialMatrix& other, double tol) const {
    return aRb_.equals(other.aRb_, tol) && aTb_.equals(other.aTb_, tol);
  }

  /// @}

  /// @name Essential matrix methods
  /// @{

  /// Rotation
  const Rot3& rotation() const {
    return aRb_;
  }

  /// Direction
  const Sphere2& direction() const {
    return aTb_;
  }

  /// Return 3*3 matrix representation
  const Matrix& matrix() const {
    return E_;
  }

  /// epipolar error, algebraic
  double error(const Vector& vA, const Vector& vB, //
      boost::optional<Matrix&> H = boost::none) const {
    if (H) {
      H->resize(1, 5);
      Matrix HR = vA.transpose() * E_ * skewSymmetric(-vB);
      Matrix HD = vA.transpose() * skewSymmetric(-aRb_.matrix() * vB)
          * aTb_.getBasis();
      *H << HR, HD;
    }
    return dot(vA, E_ * vB);
  }

  /// @}

};

/**
 * Factor that evaluates epipolar error p'Ep for given essential matrix
 */
class EssentialMatrixFactor: public NoiseModelFactor1<EssentialMatrix> {

  Point2 pA_, pB_; ///< Measurements in image A and B
  Vector vA_, vB_; ///< Homogeneous versions

  typedef NoiseModelFactor1<EssentialMatrix> Base;

public:

  /// Constructor
  EssentialMatrixFactor(Key key, const Point2& pA, const Point2& pB,
      const SharedNoiseModel& model) :
      Base(model, key), pA_(pA), pB_(pB), //
      vA_(EssentialMatrix::Homogeneous(pA)), //
      vB_(EssentialMatrix::Homogeneous(pB)) {
  }

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    Base::print(s);
    std::cout << "  EssentialMatrixFactor with measurements\n  ("
        << pA_.vector().transpose() << ")' and (" << pB_.vector().transpose()
        << ")'" << endl;
  }

  /// vector of errors returns 1D vector
  Vector evaluateError(const EssentialMatrix& E, boost::optional<Matrix&> H =
      boost::none) const {
    return (Vector(1) << E.error(vA_, vB_, H));
  }

};

//*************************************************************************
// Create two cameras and corresponding essential matrix E
Rot3 aRb = Rot3::yaw(M_PI_2);
Point3 aTb(0.1, 0, 0);
Pose3 identity, aPb(aRb, aTb);
typedef CalibratedCamera Cam;
Cam cameraA(identity), cameraB(aPb);
Matrix aEb_matrix = skewSymmetric(aTb.x(), aTb.y(), aTb.z()) * aRb.matrix();

// Create test data, we need at least 5 points
Point3 P[5] = { Point3(0, 0, 1), Point3(-0.1, 0, 1), Point3(0.1, 0, 1), //
Point3(0, 0.5, 0.5), Point3(0, -0.5, 0.5) };

// Project points in both cameras
vector<Point2> pA(5), pB(5);
vector<Point2>::iterator //
it1 = std::transform(P, P + 5, pA.begin(),
    boost::bind(&Cam::project, &cameraA, _1, boost::none, boost::none)), //
it2 = std::transform(P, P + 5, pB.begin(),
    boost::bind(&Cam::project, &cameraB, _1, boost::none, boost::none));

// Converto to homogenous coordinates
vector<Vector> vA(5), vB(5);
vector<Vector>::iterator //
it3 = std::transform(pA.begin(), pA.end(), vA.begin(),
    &EssentialMatrix::Homogeneous), //
it4 = std::transform(pB.begin(), pB.end(), vB.begin(),
    &EssentialMatrix::Homogeneous);

//*************************************************************************
TEST (EssentialMatrix, testData) {
  // Check E matrix
  Matrix expected(3, 3);
  expected << 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0;
  EXPECT(assert_equal(expected, aEb_matrix));

  // Check some projections
  EXPECT(assert_equal(Point2(0,0),pA[0]));
  EXPECT(assert_equal(Point2(0,0.1),pB[0]));
  EXPECT(assert_equal(Point2(0,-1),pA[4]));
  EXPECT(assert_equal(Point2(-1,0.2),pB[4]));

  // Check homogeneous version
  EXPECT(assert_equal((Vector(3) << -1,0.2,1),vB[4]));

  // Check epipolar constraint
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, vA[i].transpose() * aEb_matrix * vB[i], 1e-8);

  // Check epipolar constraint
  EssentialMatrix trueE(aRb, aTb);
  for (size_t i = 0; i < 5; i++)
    EXPECT_DOUBLES_EQUAL(0, trueE.error(vA[i],vB[i]), 1e-8);
}

//*************************************************************************
TEST (EssentialMatrix, equality) {
//  EssentialMatrix actual, expected;
//  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract1) {
  EssentialMatrix expected(aRb.retract((Vector(3) << 0.1, 0, 0)), aTb);
  EssentialMatrix trueE(aRb, aTb);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0.1, 0, 0, 0, 0));
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract2) {
  EssentialMatrix expected(aRb, Sphere2(aTb).retract((Vector(2) << 0.1, 0)));
  EssentialMatrix trueE(aRb, aTb);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0, 0, 0, 0.1, 0));
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, factor) {
  EssentialMatrix trueE(aRb, aTb);
  noiseModel::Unit::shared_ptr model = noiseModel::Unit::Create(1);

  for (size_t i = 0; i < 5; i++) {
    EssentialMatrixFactor factor(1, pA[i], pB[i], model);

    // Check evaluation
    Vector expected(1);
    expected << 0;
    Matrix HActual;
    Vector actual = factor.evaluateError(trueE, HActual);
    EXPECT(assert_equal(expected, actual, 1e-8));

    // Use numerical derivatives to calculate the expected Jacobian
    Matrix HExpected;
    HExpected = numericalDerivative11<EssentialMatrix>(
        boost::bind(&EssentialMatrixFactor::evaluateError, &factor, _1,
            boost::none), trueE);

    // Verify the Jacobian is correct
    CHECK(assert_equal(HExpected, HActual, 1e-9));
  }
}

//*************************************************************************
TEST (EssentialMatrix, fromConstraints) {
  // Here we want to optimize directly on essential matrix constraints
  // Yi Ma's algorithm (Ma01ijcv) is a bit cumbersome to implement,
  // but GTSAM does the equivalent anyway, provided we give the right
  // factors. In this case, the factors are the constraints.

  // We start with a factor graph and add constraints to it
  // Noise sigma is 1cm, assuming metric measurements
  NonlinearFactorGraph graph;
  noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Sigma(1,0.01);
  for (size_t i = 0; i < 5; i++)
    graph.add(EssentialMatrixFactor(1, pA[i], pB[i], model));

  // Check error at ground truth
  Values truth;
  EssentialMatrix trueE(aRb, aTb);
  truth.insert(1, trueE);
  EXPECT_DOUBLES_EQUAL(0, graph.error(truth), 1e-8);

  // Check error at initial estimate
  Values initial;
  EssentialMatrix initialE = trueE.retract((Vector(5) << 0.1, -0.1, 0.1, 0.1, -0.1));
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
    EXPECT_DOUBLES_EQUAL(0, actual.error(vA[i],vB[i]), 1e-6);

}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

