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
  Matrix3 E_; ///< Essential matrix

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

  /// Named constructor converting a Pose3 with scale to EssentialMatrix (no scale)
  static EssentialMatrix FromPose3(const Pose3& _1P2_,
      boost::optional<Matrix&> H = boost::none);

  /// Random, using Rot3::Random and Sphere2::Random
  template<typename Engine>
  static EssentialMatrix Random(Engine & rng) {
    return EssentialMatrix(Rot3::Random(rng), Sphere2::Random(rng));
  }

  /// @}

  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const;

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
  virtual EssentialMatrix retract(const Vector& xi) const;

  /// Compute the coordinates in the tangent space
  virtual Vector localCoordinates(const EssentialMatrix& other) const;

  /// @}

  /// @name Essential matrix methods
  /// @{

  /// Rotation
  inline const Rot3& rotation() const {
    return aRb_;
  }

  /// Direction
  inline const Sphere2& direction() const {
    return aTb_;
  }

  /// Return 3*3 matrix representation
  inline const Matrix3& matrix() const {
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
      boost::optional<Matrix&> Dpoint = boost::none) const;

  /**
   * Given essential matrix E in camera frame B, convert to body frame C
   * @param cRb rotation from body frame to camera frame
   * @param E essential matrix E in camera frame C
   */
  EssentialMatrix rotate(const Rot3& cRb, boost::optional<Matrix&> HE =
      boost::none, boost::optional<Matrix&> HR = boost::none) const;

  /**
   * Given essential matrix E in camera frame B, convert to body frame C
   * @param cRb rotation from body frame to camera frame
   * @param E essential matrix E in camera frame C
   */
  friend EssentialMatrix operator*(const Rot3& cRb, const EssentialMatrix& E) {
    return E.rotate(cRb);
  }

  /// epipolar error, algebraic
  double error(const Vector& vA, const Vector& vB, //
      boost::optional<Matrix&> H = boost::none) const;

  /// @}

  /// @name Streaming operators
  /// @{

  // stream to stream
  friend std::ostream& operator <<(std::ostream& os, const EssentialMatrix& E) {
    Rot3 R = E.rotation();
    Sphere2 d = E.direction();
    os.precision(10);
    os << R.xyz().transpose() << " " << d.point3().vector().transpose() << " ";
    return os;
  }

  /// stream from stream
  friend std::istream& operator >>(std::istream& is, EssentialMatrix& E) {
    double rx, ry, rz, dx, dy, dz;
    is >> rx >> ry >> rz; // Read the rotation rxyz
    is >> dx >> dy >> dz; // Read the translation dxyz

    // Create EssentialMatrix from rotation and translation
    Rot3 rot = Rot3::RzRyRx(rx, ry, rz);
    Sphere2 dt = Sphere2(dx, dy, dz);
    E = EssentialMatrix(rot, dt);

    return is;
  }

  /// @}

};

} // gtsam

