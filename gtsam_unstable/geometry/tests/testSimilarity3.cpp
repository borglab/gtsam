/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSimilarity3.cpp
 * @brief  Unit tests for Similarity3 class
 */

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

/**
 * 3D similarity transform
 */
class Similarity3: private Matrix4 {

  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s) {
    *this << s * R, t, 0, 0, 0, 1;
  }

  /// Return the translation
  const Eigen::Block<const Matrix4, 3, 1> t() const {
    return this->topRightCorner<3, 1>();
  }

  /// Return the rotation matrix
  const Eigen::Block<const Matrix4, 3, 3> R() const {
    return this->topLeftCorner<3, 3>();
  }

public:

  Similarity3() {
    setIdentity();
  }

  /// Construct pure scaling
  Similarity3(double s) {
    setIdentity();
    this->topLeftCorner<3, 3>() = s * Matrix3::Identity();
  }

  /// Construct from GTSAM types
  Similarity3(const Rot3& R, const Point3& t, double s) {
    *this << R.matrix(), t.vector(), 0, 0, 0, 1.0 / s;
  }

  bool operator==(const Similarity3& other) const {
    return Matrix4::operator==(other);
  }

  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 7 DOF - used to autodetect sizes
  inline static size_t Dim() {
    return 7;
  }

  /// Dimensionality of tangent space = 3 DOF
  inline size_t dim() const {
    return 7;
  }

  /// Return the rotation matrix
  Rot3 rotation() const {
    return R().eval();
  }

  /// Return the translation
  Point3 translation() const {
    Vector3 t = this->t();
    return Point3::Expmap(t);
  }

  /// Return the scale
  double scale() const {
    return 1.0 / (*this)(3, 3);
  }

  /// Update Similarity transform via 7-dim vector in tangent space
  Similarity3 retract(const Vector7& v) const {

    // Will retracting or localCoordinating R work if R is not a unit rotation?
    // Also, how do we actually get s out?  Seems like we need to store it somewhere.
    return Similarity3( //
        rotation().retract(v.head<3>()), // retract rotation using v[0,1,2]
        translation().retract(v.segment<3>(3)), scale() + v[6]); // finally, update scale using v[6]
  }

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  Vector7 localCoordinates(const Similarity3& other) const {

    Vector7 v;
    v.head<3>() = rotation().localCoordinates(other.rotation());
    v.segment<3>(3) = translation().localCoordinates(other.translation());
    v[6] = other.scale() - scale();
    return v;
  }

  /// @}

  /// @name Lie Group
  /// @{

  // compose T1*T2
  // between T2*inverse(T1)
  // identity I4
  // inverse inverse(T)

  /// @}

};
}

#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

//******************************************************************************
TEST(Similarity3, Constructors) {
  Similarity3 test;
}

//******************************************************************************
TEST(Similarity3, Getters) {
  Similarity3 test;
  EXPECT(assert_equal(Rot3(), test.rotation()));
  EXPECT(assert_equal(Point3(), test.translation()));
  EXPECT_DOUBLES_EQUAL(1.0, test.scale(), 1e-9);
}

//******************************************************************************
TEST(Similarity3, Getters2) {
  Similarity3 test(Rot3::ypr(1, 2, 3), Point3(4, 5, 6), 7);
  EXPECT(assert_equal(Rot3::ypr(1, 2, 3), test.rotation()));
  EXPECT(assert_equal(Point3(4, 5, 6), test.translation()));
  EXPECT_DOUBLES_EQUAL(7.0, test.scale(), 1e-9);
}

//******************************************************************************
TEST(Similarity3, Manifold) {
  EXPECT_LONGS_EQUAL(7, Similarity3::Dim());
  Vector z = Vector7::Zero();
  Similarity3 sim;
  EXPECT(sim.retract(z) == sim);

  Vector7 v = Vector7::Zero();
  v(6) = 2;
  Similarity3 sim2;
  EXPECT(sim2.retract(z) == sim2);

  EXPECT(assert_equal(z, sim2.localCoordinates(sim)));

  Similarity3 sim3 = Similarity3(Rot3(), Point3(1, 1, 1), 1);
  Vector v3(7);
  v3 << 0, 0, 0, 1, 1, 1, 0;
  EXPECT(assert_equal(v3, sim2.localCoordinates(sim3)));

  Similarity3 other = Similarity3(Rot3::ypr(1, 2, 3), Point3(4, 5, 6), 7);
//  Similarity3 other = Similarity3(Rot3(),Point3(4,5,6),1);

  EXPECT(sim.retract(sim.localCoordinates(other)) == other);

  // TODO add unit tests for retract and localCoordinates
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

