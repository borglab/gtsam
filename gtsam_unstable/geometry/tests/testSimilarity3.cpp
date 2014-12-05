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

#include <gtsam/base/Manifold.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam {

/**
 * 3D similarity transform
 */
class Similarity3: private Matrix4 {

  /// Construct from Eigen types
  Similarity3(const Matrix3& R, const Vector3& t, double s) {
    *this << s * R, t, 0, 0, 0, 1;
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
    *this << s * R.matrix(), t.vector(), 0, 0, 0, 1;
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

  /// Update Similarity transform via 7-dim vector in tangent space
  Similarity3 retract(const Vector& v) const {

    // Get rotation - translation - scale from 4*4
    Rot3 R(this->topLeftCorner<3, 3>());
    Vector3 t(this->topRightCorner<3, 1>());
    double s((*this)(3, 3));

    return Similarity3( //
        R.retract(v.head<3>()), // retract rotation using v[0,1,2]
        Point3(t + v.segment < 3 > (3)), // retract translation via v[3,4,5]
        s + v[6]); // finally, update scale using v[6]
  }

  /// 7-dimensional vector v in tangent space that makes other = this->retract(v)
  Vector localCoordinates(const Similarity3& other) const {
    return Vector7::Zero();
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

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

//******************************************************************************
TEST(Similarity3, constructors) {
  Similarity3 test;
}

//******************************************************************************
TEST(Similarity3, manifold) {
  EXPECT_LONGS_EQUAL(7, Similarity3::Dim());
  Vector7 z = Vector7::Zero();
  Similarity3 sim;
  EXPECT(sim.retract(z)==sim);

  Vector7 v = Vector7::Zero();
  v(6) = 2;
  Similarity3 sim2;
  EXPECT(sim2.retract(z)==sim2);

  // TODO add unit tests for retract and localCoordinates
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

