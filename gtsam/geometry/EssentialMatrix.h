/*
 * @file EssentialMatrix.h
 * @brief EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Manifold.h>

#include <iosfwd>
#include <string>

namespace gtsam {

/**
 * An essential matrix is like a Pose3, except with translation up to scale
 * It is named after the 3*3 matrix aEb = [aTb]x aRb from computer vision,
 * but here we choose instead to parameterize it as a (Rot3,Unit3) pair.
 * We can then non-linearly optimize immediately on this 5-dimensional manifold.
 */
class EssentialMatrix {
 private:
  Rot3 R_;     ///< Rotation
  Unit3 t_;    ///< Translation
  Matrix3 E_;  ///< Essential matrix

 public:
  /// Static function to convert Point2 to homogeneous coordinates
  static Vector3 Homogeneous(const Point2& p) {
    return Vector3(p.x(), p.y(), 1);
  }

  /// @name Constructors and named constructors
  /// @{

  /// Default constructor
  EssentialMatrix() :E_(t_.skew()) {
  }

  /// Construct from rotation and translation
  EssentialMatrix(const Rot3& aRb, const Unit3& aTb) :
      R_(aRb), t_(aTb), E_(t_.skew() * R_.matrix()) {
  }

  /// Named constructor with derivatives
  GTSAM_EXPORT static EssentialMatrix FromRotationAndDirection(const Rot3& aRb, const Unit3& aTb,
                                                  OptionalJacobian<5, 3> H1 = boost::none,
                                                  OptionalJacobian<5, 2> H2 = boost::none);

  /// Named constructor converting a Pose3 with scale to EssentialMatrix (no scale)
  GTSAM_EXPORT static EssentialMatrix FromPose3(const Pose3& _1P2_,
      OptionalJacobian<5, 6> H = boost::none);

  /// Random, using Rot3::Random and Unit3::Random
  template<typename Engine>
  static EssentialMatrix Random(Engine & rng) {
    return EssentialMatrix(Rot3::Random(rng), Unit3::Random(rng));
  }

  virtual ~EssentialMatrix() {}

  /// @}

  /// @name Testable
  /// @{

  /// print with optional string
  GTSAM_EXPORT void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const EssentialMatrix& other, double tol = 1e-8) const {
    return R_.equals(other.R_, tol)
        && t_.equals(other.t_, tol);
  }

  /// @}

  /// @name Manifold
  /// @{
  enum { dimension = 5 };
  inline static size_t Dim() { return dimension;}
  inline size_t dim() const { return dimension;}

  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  /// Retract delta to manifold
  EssentialMatrix retract(const Vector5& xi) const {
    return EssentialMatrix(R_.retract(xi.head<3>()), t_.retract(xi.tail<2>()));
  }

  /// Compute the coordinates in the tangent space
  Vector5 localCoordinates(const EssentialMatrix& other) const {
    auto v1 = R_.localCoordinates(other.R_);
    auto v2 = t_.localCoordinates(other.t_);
    Vector5 v;
    v << v1, v2;
    return v;
  }
  /// @}

  /// @name Essential matrix methods
  /// @{

  /// Rotation
  inline const Rot3& rotation() const {
    return R_;
  }

  /// Direction
  inline const Unit3& direction() const {
    return t_;
  }

  /// Return 3*3 matrix representation
  inline const Matrix3& matrix() const {
    return E_;
  }

  /// Return epipole in image_a , as Unit3 to allow for infinity
  inline const Unit3& epipole_a() const {
    return t_;
  }

  /// Return epipole in image_b, as Unit3 to allow for infinity
  inline Unit3 epipole_b() const {
    return R_.unrotate(t_);
  }

  /**
   * @brief takes point in world coordinates and transforms it to pose with |t|==1
   * @param p point in world coordinates
   * @param DE optional 3*5 Jacobian wrpt to E
   * @param Dpoint optional 3*3 Jacobian wrpt point
   * @return point in pose coordinates
   */
  GTSAM_EXPORT Point3 transformTo(const Point3& p,
      OptionalJacobian<3, 5> DE = boost::none,
      OptionalJacobian<3, 3> Dpoint = boost::none) const;

  /**
   * Given essential matrix E in camera frame B, convert to body frame C
   * @param cRb rotation from body frame to camera frame
   * @param E essential matrix E in camera frame C
   */
  GTSAM_EXPORT EssentialMatrix rotate(const Rot3& cRb, OptionalJacobian<5, 5> HE =
      boost::none, OptionalJacobian<5, 3> HR = boost::none) const;

  /**
   * Given essential matrix E in camera frame B, convert to body frame C
   * @param cRb rotation from body frame to camera frame
   * @param E essential matrix E in camera frame C
   */
  friend EssentialMatrix operator*(const Rot3& cRb, const EssentialMatrix& E) {
    return E.rotate(cRb);
  }

  /// epipolar error, algebraic
  GTSAM_EXPORT double error(const Vector3& vA, const Vector3& vB,
      OptionalJacobian<1, 5> H = boost::none) const;

  /// @}

  /// @name Streaming operators
  /// @{

  /// stream to stream
  GTSAM_EXPORT friend std::ostream& operator <<(std::ostream& os, const EssentialMatrix& E);

  /// stream from stream
  GTSAM_EXPORT friend std::istream& operator >>(std::istream& is, EssentialMatrix& E);

  /// @}

 private:
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(R_);
      ar & BOOST_SERIALIZATION_NVP(t_);

      ar & boost::serialization::make_nvp("E11", E_(0, 0));
      ar & boost::serialization::make_nvp("E12", E_(0, 1));
      ar & boost::serialization::make_nvp("E13", E_(0, 2));
      ar & boost::serialization::make_nvp("E21", E_(1, 0));
      ar & boost::serialization::make_nvp("E22", E_(1, 1));
      ar & boost::serialization::make_nvp("E23", E_(1, 2));
      ar & boost::serialization::make_nvp("E31", E_(2, 0));
      ar & boost::serialization::make_nvp("E32", E_(2, 1));
      ar & boost::serialization::make_nvp("E33", E_(2, 2));
    }

  /// @}

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

template<>
struct traits<EssentialMatrix> : public internal::Manifold<EssentialMatrix> {};

template<>
struct traits<const EssentialMatrix> : public internal::Manifold<EssentialMatrix> {};

}  // namespace gtsam

