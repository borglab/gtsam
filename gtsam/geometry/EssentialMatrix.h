/*
 * @file EssentialMatrix.h
 * @brief EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Sphere2.h>
#include <gtsam/geometry/Point2.h>
#include <iostream>

namespace gtsam {

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

public:

  /// Static function to convert Point2 to homogeneous coordinates
  static Vector Homogeneous(const Point2& p) {
    return Vector(3) << p.x(), p.y(), 1;
  }

  /// @name Constructors and named constructors
  /// @{

  /// Default constructor
  EssentialMatrix() :
      aTb_(1, 0, 0), E_(aTb_.skew()) {
  }

  /// Construct from rotation and translation
  EssentialMatrix(const Rot3& aRb, const Sphere2& aTb) :
      aRb_(aRb), aTb_(aTb), E_(aTb_.skew() * aRb_.matrix()) {
  }

  /// @}

  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const {
    std::cout << s;
    aRb_.print("R:\n");
    aTb_.print("d: ");
  }

  /// assert equality up to a tolerance
  bool equals(const EssentialMatrix& other, double tol = 1e-8) const {
    return aRb_.equals(other.aRb_, tol) && aTb_.equals(other.aTb_, tol);
  }

  /// @}

  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 5 DOF
  inline static size_t Dim() {
    return 5;
  }

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

  /**
   * @brief takes point in world coordinates and transforms it to pose with |t|==1
   * @param p point in world coordinates
   * @param DE optional 3*5 Jacobian wrpt to E
   * @param Dpoint optional 3*3 Jacobian wrpt point
   * @return point in pose coordinates
   */
  Point3 transform_to(const Point3& p,
      boost::optional<Matrix&> DE = boost::none,
      boost::optional<Matrix&> Dpoint = boost::none) const {
    Pose3 pose(aRb_, aTb_.point3());
    Point3 q = pose.transform_to(p, DE, Dpoint);
    if (DE) {
      // DE returned by pose.transform_to is 3*6, but we need it to be 3*5
      // The last 3 columns are derivative with respect to change in translation
      // The derivative of translation with respect to a 2D sphere delta is 3*2 aTb_.basis()
      // Duy made an educated guess that this needs to be rotated to the local frame
      Matrix H(3, 5);
      H << DE->block < 3, 3 > (0, 0), -aRb_.transpose() * aTb_.basis();
      *DE = H;
    }
    return q;
  }

  /// epipolar error, algebraic
  double error(const Vector& vA, const Vector& vB, //
      boost::optional<Matrix&> H = boost::none) const {
    if (H) {
      H->resize(1, 5);
      // See math.lyx
      Matrix HR = vA.transpose() * E_ * skewSymmetric(-vB);
      Matrix HD = vA.transpose() * skewSymmetric(-aRb_.matrix() * vB)
          * aTb_.basis();
      *H << HR, HD;
    }
    return dot(vA, E_ * vB);
  }

  /// @}

};

} // gtsam

