/*
 * @file EssentialMatrix.h
 * @brief EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/geometry/Rot3.h>
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
  void print(const std::string& s) const {
    std::cout << s;
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

} // gtsam

