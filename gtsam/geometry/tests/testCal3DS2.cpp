/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testCal3DS2.cpp
 * @brief Unit tests for transform derivatives
 */


#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Cal3DS2)
GTSAM_CONCEPT_MANIFOLD_INST(Cal3DS2)

static Cal3DS2 K(500, 100, 0.1, 320, 240, 1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3);
static Point2 p(2,3);

/* ************************************************************************* */
TEST( Cal3DS2, uncalibrate)
{
  Vector k = K.k() ;
  double r = p.x()*p.x() + p.y()*p.y() ;
  double g = 1+k[0]*r+k[1]*r*r ;
  double tx = 2*k[2]*p.x()*p.y()       +   k[3]*(r+2*p.x()*p.x()) ;
  double ty =   k[2]*(r+2*p.y()*p.y()) + 2*k[3]*p.x()*p.y() ;
  Vector v_hat = (Vector(3) << g*p.x() + tx, g*p.y() + ty, 1.0).finished();
  Vector v_i = K.K() * v_hat ;
  Point2 p_i(v_i(0)/v_i(2), v_i(1)/v_i(2)) ;
  Point2 q = K.uncalibrate(p);
  CHECK(assert_equal(q,p_i));
}

TEST( Cal3DS2, calibrate )
{
  Point2 pn(0.5, 0.5);
  Point2 pi = K.uncalibrate(pn);
  Point2 pn_hat = K.calibrate(pi);
  CHECK( pn.equals(pn_hat, 1e-5));
}

Point2 uncalibrate_(const Cal3DS2& k, const Point2& pt) { return k.uncalibrate(pt); }

/* ************************************************************************* */
TEST( Cal3DS2, Duncalibrate1)
{
  Matrix computed;
  K.uncalibrate(p, computed, boost::none);
  Matrix numerical = numericalDerivative21(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
  Matrix separate = K.D2d_calibration(p);
  CHECK(assert_equal(numerical,separate,1e-5));
}

/* ************************************************************************* */
TEST( Cal3DS2, Duncalibrate2)
{
  Matrix computed; K.uncalibrate(p, boost::none, computed);
  Matrix numerical = numericalDerivative22(uncalibrate_, K, p, 1e-7);
  CHECK(assert_equal(numerical,computed,1e-5));
  Matrix separate = K.D2d_intrinsic(p);
  CHECK(assert_equal(numerical,separate,1e-5));
}

/* ************************************************************************* */
TEST( Cal3DS2, assert_equal)
{
  CHECK(assert_equal(K,K,1e-5));
}

/* ************************************************************************* */
TEST( Cal3DS2, retract)
{
  Cal3DS2 expected(500 + 1, 100 + 2, 0.1 + 3, 320 + 4, 240 + 5, 1e-3 + 6,
      2.0 * 1e-3 + 7, 3.0 * 1e-3 + 8, 4.0 * 1e-3 + 9);
  Vector d(9);
  d << 1,2,3,4,5,6,7,8,9;
  Cal3DS2 actual = K.retract(d);
  CHECK(assert_equal(expected,actual,1e-7));
  CHECK(assert_equal(d,K.localCoordinates(actual),1e-7));
}

/* ************************************************************************* */
// Test Eigen::Ref
//TEST( Cal3DS2, Ref) {
//  // In this case we don't want a copy
//  Matrix25 fixedDcal;
//  Eigen::Ref<Matrix25> test1(fixedDcal);
//  cout << test1 << endl;
//  test1.resize(2, 5);
//  test1 = Matrix25::Zero();
//  cout << test1 << endl;
//
//  // In this case we want dynamic1 to be correctly allocate and filled
//  Matrix dynamic1(2,5);
//  Eigen::Ref<Matrix25> test2(dynamic1);
//  cout << test2.rows() << "," << test2.cols() << endl;
//  test2.resize(2, 5);
//  test2 = Matrix25::Zero();
//  cout << test2 << endl;
//}

//void test(Eigen::Ref<Matrix25> H) {
//  cout << "test" << endl;
//  cout << H.size() << endl;
//  cout << H.rows() << "," << H.cols() << endl;
//  if (H.size()) {
//    cout << "H before:\n" << H << endl;
//    H.resize(2, 5);
//    H = Matrix25::Zero();
//    cout << "H after:\n" << H << endl;
//  }
//}
//
//TEST( Cal3DS2, Ref) {
//
//  // In this case we don't want anything to happen
//  cout << "\nempty" << endl;
//  Matrix dynamic0;
//  test(dynamic0);
//  cout << "after" << dynamic0 << endl;
//
//  // In this case we don't want a copy, TODO: does it copy???
//  cout << "\nfixed" << endl;
//  Matrix25 fixedDcal;
//  fixedDcal.setOnes();
//
//  cout << fixedDcal << endl;
//  test(fixedDcal);
//  cout << "after" << fixedDcal << endl;
//
//  // In this case we want dynamic1 to be correctly allocate and filled
//  cout << "\ndynamic wrong size" << endl;
//  Matrix dynamic1(8,5);
//  dynamic1.setOnes();
//
//  cout << dynamic1 << endl;
//  test(dynamic1);
//  cout << "after" << dynamic1 << endl;
//
//  // In this case we want dynamic1 to be correctly allocate and filled
//  cout << "\ndynamic right size" << endl;
//  Matrix dynamic2(2,5);
//  dynamic2.setOnes();
//
//  cout << dynamic2 << endl;
//  test(dynamic2);
//  cout << "after" << dynamic2 << endl;
//}
//
//void test2(Eigen::Ref<Matrix> H) {
//  cout << "test2" << endl;
//  cout << H.size() << endl;
//  cout << H.rows() << "," << H.cols() << endl;
//  if (H.size()) {
//    cout << "H before:\n" << H << endl;
//    H.resize(2, 5);
//    H = Matrix25::Zero();
//    cout << "H after:\n" << H << endl;
//  }
//}
//
//TEST( Cal3DS2, Ref2) {
//
//  // In this case we don't want anything to happen
//  cout << "\nempty" << endl;
//  Matrix dynamic0;
//  test2(dynamic0);
//  cout << "after" << dynamic0 << endl;
//
//  // In this case we don't want a copy, TODO: does it copy???
//  cout << "\nfixed" << endl;
//  Matrix25 fixedDcal;
//  fixedDcal.setOnes();
//
//  cout << fixedDcal << endl;
//  test2(fixedDcal);
//  cout << "after" << fixedDcal << endl;
//
//  // In this case we want dynamic1 to be correctly allocate and filled
//  cout << "\ndynamic wrong size" << endl;
//  Matrix dynamic1(8,5);
//  dynamic1.setOnes();
//
//  cout << dynamic1 << endl;
//  test2(dynamic1);
//  cout << "after" << dynamic1 << endl;
//
//  // In this case we want dynamic1 to be correctly allocate and filled
//  cout << "\ndynamic right size" << endl;
//  Matrix dynamic2(2,5);
//  dynamic2.setOnes();
//
//  cout << dynamic2 << endl;
//  test2(dynamic2);
//  cout << "after" << dynamic2 << endl;
//}

/* ************************************************************************* */

template<int Rows, int Cols>
struct OptionalJacobian {
  bool empty_;
  typedef Eigen::Matrix<double, Rows, Cols> Fixed;
  Eigen::Map<Fixed> map_;
  OptionalJacobian() :
      empty_(true), map_(NULL) {
  }
  OptionalJacobian(Fixed& fixed) :
      empty_(false), map_(NULL) {
    // Trick on http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
    // to make map_ usurp the memory of the fixed size matrix
    new (&map_) Eigen::Map<Fixed>(fixed.data());
  }
  OptionalJacobian(Matrix& dynamic) :
      empty_(false), map_(NULL) {
    dynamic.resize(Rows, Cols);
    // Trick on http://eigen.tuxfamily.org/dox/group__TutorialMapClass.html
    // to make map_ usurp the memory of the fixed size matrix
    new (&map_) Eigen::Map<Fixed>(dynamic.data());
  }
  /// Return true is allocated, false if default constructor was used
  operator bool() const {
    return !empty_;
  }
  /// Assign a fixed matrix to this
  void operator=(const Fixed& fixed) {
    map_ << fixed;
  }
  /// Print
  void print(const string& s) {
    if (!empty_)
      cout << s << map_ << endl;
  }
};

void test3(OptionalJacobian<2,3> H = OptionalJacobian<2,3>()) {
  if (H)
    H = Matrix23::Zero();
}

TEST( Cal3DS2, Ref2) {

  Matrix expected;
  expected = Matrix23::Zero();

  // Default argument does nothing
  test3();

  // Fixed size, no copy
  Matrix23 fixedDcal;
  fixedDcal.setOnes();
  test3(fixedDcal);
  EXPECT(assert_equal(expected,fixedDcal));

  // Empty is no longer a sign we don't want a matrix, we want it resized
  Matrix dynamic0;
  test3(dynamic0);
  EXPECT(assert_equal(expected,dynamic0));

  // Dynamic wrong size
  Matrix dynamic1(3,5);
  dynamic1.setOnes();
  test3(dynamic1);
  EXPECT(assert_equal(expected,dynamic0));

  // Dynamic right size
  Matrix dynamic2(2,5);
  dynamic2.setOnes();
  test3(dynamic2);
  EXPECT(assert_equal(dynamic2,dynamic0));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
