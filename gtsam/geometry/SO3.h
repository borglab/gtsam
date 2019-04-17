/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SO3.h
 * @brief   3*3 matrix representation of SO(3)
 * @author  Frank Dellaert
 * @author  Luca Carlone
 * @author  Duy Nguyen Ta
 * @date    December 2014
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Lie.h>

#include <cmath>
#include <iosfwd>

namespace gtsam {

/**
 *  True SO(3), i.e., 3*3 matrix subgroup
 *  We guarantee (all but first) constructors only generate from sub-manifold.
 *  However, round-off errors in repeated composition could move off it...
 */
class SO3: public Matrix3, public LieGroup<SO3, 3> {

protected:

public:
  enum { N = 3 };
  enum { dimension = 3 };

  /// @name Constructors
  /// @{

  /// Default constructor creates identity
  SO3() :
      Matrix3(I_3x3) {
  }

  /// Constructor from Eigen Matrix
  template<typename Derived>
  SO3(const MatrixBase<Derived>& R) :
      Matrix3(R.eval()) {
  }

  /// Constructor from AngleAxisd
  SO3(const Eigen::AngleAxisd& angleAxis) :
      Matrix3(angleAxis) {
  }

  /// Static, named constructor. TODO(dellaert): think about relation with above
  static SO3 AxisAngle(const Vector3& axis, double theta);

  /// Static, named constructor that finds SO(3) matrix closest to M in Frobenius norm.
  static SO3 ClosestTo(const Matrix3& M);

  /// Static, named constructor that finds chordal mean = argmin_R \sum sqr(|R-R_i|_F).
  static SO3 ChordalMean(const std::vector<SO3>& rotations);

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s) const;

  bool equals(const SO3 & R, double tol) const {
    return equal_with_abs_tol(*this, R, tol);
  }

  /// @}
  /// @name Group
  /// @{

  /// identity rotation for group operation
  static SO3 identity() {
    return I_3x3;
  }

  /// inverse of a rotation = transpose
  SO3 inverse() const {
    return this->transpose();
  }

  /// @}
  /// @name Lie Group
  /// @{

  static Matrix3 Hat(const Vector3 &xi); ///< make skew symmetric matrix
  static Vector3 Vee(const Matrix3 &X);  ///< inverse of Hat

  /**
   * Exponential map at identity - create a rotation from canonical coordinates
   * \f$ [R_x,R_y,R_z] \f$ using Rodrigues' formula
   */
  static SO3 Expmap(const Vector3& omega, ChartJacobian H = boost::none);

  /// Derivative of Expmap
  static Matrix3 ExpmapDerivative(const Vector3& omega);

  /**
   * Log map at identity - returns the canonical coordinates
   * \f$ [R_x,R_y,R_z] \f$ of this rotation
   */
  static Vector3 Logmap(const SO3& R, ChartJacobian H = boost::none);

  /// Derivative of Logmap
  static Matrix3 LogmapDerivative(const Vector3& omega);

  Matrix3 AdjointMap() const {
    return *this;
  }

  // Chart at origin
  struct ChartAtOrigin {
    static SO3 Retract(const Vector3& omega, ChartJacobian H = boost::none) {
      return Expmap(omega, H);
    }
    static Vector3 Local(const SO3& R, ChartJacobian H = boost::none) {
      return Logmap(R, H);
    }
  };

  using LieGroup<SO3, 3>::inverse;

  /// @}
  /// @name Other methods
  /// @{

  /// Vectorize
  Vector9 vec(OptionalJacobian<9, 3> H = boost::none) const;

  /// Return matrix (for wrapper)
  const Matrix3& matrix() const { return *this;}

  /// @

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/)
    {
       ar & boost::serialization::make_nvp("R11", (*this)(0,0));
       ar & boost::serialization::make_nvp("R12", (*this)(0,1));
       ar & boost::serialization::make_nvp("R13", (*this)(0,2));
       ar & boost::serialization::make_nvp("R21", (*this)(1,0));
       ar & boost::serialization::make_nvp("R22", (*this)(1,1));
       ar & boost::serialization::make_nvp("R23", (*this)(1,2));
       ar & boost::serialization::make_nvp("R31", (*this)(2,0));
       ar & boost::serialization::make_nvp("R32", (*this)(2,1));
       ar & boost::serialization::make_nvp("R33", (*this)(2,2));
    }

};

namespace so3 {

/** 
 * Compose general matrix with an SO(3) element. 
 * We only provide the 9*9 derivative in the first argument M.
 */
Matrix3 compose(const Matrix3& M, const SO3& R,
                OptionalJacobian<9, 9> H = boost::none);

/// (constant) Jacobian of compose wrpt M
Matrix99 Dcompose(const SO3& R);

// Below are two functors that allow for saving computation when exponential map
// and its derivatives are needed at the same location in so<3>. The second
// functor also implements dedicated methods to apply dexp and/or inv(dexp).

/// Functor implementing Exponential map
class GTSAM_EXPORT ExpmapFunctor {
 protected:
  const double theta2;
  Matrix3 W, K, KK;
  bool nearZero;
  double theta, sin_theta, one_minus_cos;  // only defined if !nearZero

  void init(bool nearZeroApprox = false);

 public:
  /// Constructor with element of Lie algebra so(3)
  ExpmapFunctor(const Vector3& omega, bool nearZeroApprox = false);

  /// Constructor with axis-angle
  ExpmapFunctor(const Vector3& axis, double angle, bool nearZeroApprox = false);

  /// Rodrigues formula
  SO3 expmap() const;
};

/// Functor that implements Exponential map *and* its derivatives
class GTSAM_EXPORT DexpFunctor : public ExpmapFunctor {
  const Vector3 omega;
  double a, b;
  Matrix3 dexp_;

 public:
  /// Constructor with element of Lie algebra so(3)
  DexpFunctor(const Vector3& omega, bool nearZeroApprox = false);

  // NOTE(luca): Right Jacobian for Exponential map in SO(3) - equation
  // (10.86) and following equations in G.S. Chirikjian, "Stochastic Models,
  // Information Theory, and Lie Groups", Volume 2, 2008.
  //   expmap(omega + v) \approx expmap(omega) * expmap(dexp * v)
  // This maps a perturbation v in the tangent space to
  // a perturbation on the manifold Expmap(dexp * v) */
  const Matrix3& dexp() const { return dexp_; }

  /// Multiplies with dexp(), with optional derivatives
  Vector3 applyDexp(const Vector3& v, OptionalJacobian<3, 3> H1 = boost::none,
                    OptionalJacobian<3, 3> H2 = boost::none) const;

  /// Multiplies with dexp().inverse(), with optional derivatives
  Vector3 applyInvDexp(const Vector3& v,
                       OptionalJacobian<3, 3> H1 = boost::none,
                       OptionalJacobian<3, 3> H2 = boost::none) const;
};
}  //  namespace so3

template<>
struct traits<SO3> : public internal::LieGroup<SO3> {
};

template<>
struct traits<const SO3> : public internal::LieGroup<SO3> {
};
} // end namespace gtsam

