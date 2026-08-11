/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Rot2.h
 * @brief 2D rotation
 * @date Dec 9, 2009
 * @author Frank Dellaert
 * @author John Lambert
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/geometry/Point2.h>

#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {

  /**
   * Rotation matrix
   * NOTE: the angle theta is in radians unless explicitly stated
   * @ingroup geometry
   * \nosubgrouping
   */
  class GTSAM_EXPORT Rot2 : public MatrixLieGroup<Rot2, 1, 2> {
    /** we store cos(theta) and sin(theta) */
    double c_, s_;

    /** private constructor from cos/sin */
    inline Rot2(double c, double s) : c_(c), s_(s) {}

  public:

    /// @name Constructors and named constructors
    /// @{

    /** default constructor, zero rotation */
    Rot2() : c_(1.0), s_(0.0) {}
    
    /** copy constructor */
    Rot2(const Rot2& r) = default;

    Rot2& operator=(const Rot2& other) = default;

    /// Constructor from angle in radians == exponential map at identity
    Rot2(double theta) : c_(cos(theta)), s_(sin(theta)) {}

    // Rot2& operator=(const gtsam::Rot2& other) = default;

    /// Named constructor from angle in radians
    static Rot2 fromAngle(double theta) {
      return Rot2(theta);
    }

    /// Named constructor from angle in degrees
    static Rot2 fromDegrees(double theta) {
      static const double degree = M_PI / 180;
      return fromAngle(theta * degree);
    }

    /// Named constructor from cos(theta),sin(theta) pair
    static Rot2 fromCosSin(double c, double s);

    /**
     * Named constructor with derivative
     * Calculate relative bearing to a landmark in local coordinate frame
     * @param d 2D location of landmark
     * @param H optional reference for Jacobian
     * @return 2D rotation \f$ \in SO(2) \f$
     */
    static Rot2 relativeBearing(const Point2& d, OptionalJacobian<1,2> H =
        {});

    /** Named constructor that behaves as atan2, i.e., y,x order (!) and normalizes */
    static Rot2 atan2(double y, double x);

    /**
     * Random, generates random angle \f$\in\f$ [-pi,pi]
     * Example:
     *   std::mt19937 engine(42);
     *   Unit3 unit = Unit3::Random(engine);
     */
    static Rot2 Random(std::mt19937 & rng);

    /// @}
    /// @name Testable
    /// @{

    /** print */
    void print(const std::string& s = "theta") const;

    /** equals with an tolerance */
    bool equals(const Rot2& R, double tol = 1e-9) const;

    /// @}
    /// @name Group
    /// @{

    /** Identity */
    inline static Rot2 Identity() {  return Rot2(); }

    /** The inverse rotation - negative angle */
    Rot2 inverse() const { return Rot2(c_, -s_);}

    /** Compose - make a new rotation by adding angles */
    Rot2 operator*(const Rot2& R) const {
      return fromCosSin(c_ * R.c_ - s_ * R.s_, s_ * R.c_ + c_ * R.s_);
    }

    /// @}
    /// @name Lie Group
    /// @{

    using LieAlgebra = Matrix2;

    /// Exponential map at identity - create a rotation from canonical coordinates
    static Rot2 Expmap(const Vector1& v, ChartJacobian H = {});

    /// Log map at identity - return the canonical coordinates of this rotation
    static Vector1 Logmap(const Rot2& r, ChartJacobian H = {});

    /** Calculate Adjoint map */
    Matrix1 AdjointMap() const { return I_1x1; }

    /// Lie-algebra adjoint (zero for abelian SO(2)).
    static Matrix1 adjointMap(const Vector1&);

    /// Apply Lie-algebra adjoint (always zero).
    static Vector1 adjoint(const Vector1&, const Vector1&,
                           OptionalJacobian<1, 1> Hxi = {},
                           OptionalJacobian<1, 1> Hy = {});

    /// Left-trivialized derivative of the exponential map
    static Matrix ExpmapDerivative(const Vector& /*v*/) {
      return I_1x1;
    }

    /// Left-trivialized derivative inverse of the exponential map
    static Matrix LogmapDerivative(const Vector& /*v*/) {
      return I_1x1;
    }

    // Chart at origin simply uses exponential map and its inverse
    struct ChartAtOrigin {
      static Rot2 Retract(const Vector1& v, ChartJacobian H = {}) {
        return Expmap(v, H);
      }
      static Vector1 Local(const Rot2& r, ChartJacobian H = {}) {
        return Logmap(r, H);
      }
    };

    using LieGroup<Rot2, 1>::inverse; // version with derivative

    /// Hat maps from tangent vector to Lie algebra
    static Matrix2 Hat(const Vector1& xi);

    /// Vee maps from Lie algebra to tangent vector
    static Vector1 Vee(const Matrix2& X);

    /// @}
    /// @name Group Action on Point2
    /// @{

    /**
     * rotate point from rotated coordinate frame to world \f$ p^w = R_c^w p^c \f$
     */
    Point2 rotate(const Point2& p, OptionalJacobian<2, 1> H1 = {},
        OptionalJacobian<2, 2> H2 = {}) const;

    /** syntactic sugar for rotate */
    inline Point2 operator*(const Point2& p) const {
      return rotate(p);
    }

    /**
     * rotate point from world to rotated frame \f$ p^c = (R_c^w)^T p^w \f$
     */
    Point2 unrotate(const Point2& p, OptionalJacobian<2, 1> H1 = {},
        OptionalJacobian<2, 2> H2 = {}) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// Creates a unit vector as a Point2
    inline Point2 unit() const {
      return Point2(c_, s_);
    }

    /** return angle (RADIANS) */
    double theta() const {
      return ::atan2(s_, c_);
    }

    /** return angle (DEGREES) */
    double degrees() const {
      const double degree = M_PI / 180;
      return theta() / degree;
    }

    /** return cos */
    inline double c() const {
      return c_;
    }

    /** return sin */
    inline double s() const {
      return s_;
    }

    /** return 2*2 rotation matrix */
    Matrix2 matrix() const;

    /** return 2*2 transpose (inverse) rotation matrix */
    Matrix2 transpose() const;

    /** Find closest valid rotation matrix, given a 2x2 matrix */
    static Rot2 ClosestTo(const Matrix2& M);

    /** Vectorize the rotation matrix into a 4D vector */
    Vector4 vec(OptionalJacobian<4, 1> H = {}) const;
    /// @}

    private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(s_);
    }
#endif

  };

template <>
struct traits<Rot2> : public internal::MatrixLieGroup<Rot2, 2> {
  /// Dimension of the D=1 homogenized QCQP vector.
  inline constexpr static int QcqpVectorDim = 5;

  /**
   * Return a matrix-valued QCQP variable for Rot2.
   *
   * D=1 prepends a fixed homogenization coordinate to the existing
   * column-major SO(2) vectorization, yielding a 5-by-1 matrix.
   * D>=2 returns [R', 0] as a 2-by-D row-orthonormal matrix. These matrix
   * variables form a Stiefel relaxation with a common right-O(D) gauge.
   */
  template <int D = 1>
  static Matrix QcqpValue(const Rot2& value) {
    if constexpr (D == 1) {
      const Matrix2 R = value.matrix();
      Vector5 X;
      X(0, 0) = 1.0;  // Homogenization entry.
      X.bottomRows(4) = Eigen::Map<const Matrix>(R.data(), 4, 1);
      return X;
    } else if constexpr (D >= 2) {
      Matrix X = Matrix::Zero(2, D);
      X.leftCols<2>() = value.matrix().transpose();
      return X;
    } else {
      throw std::invalid_argument(
          "traits<Rot2>::QcqpValue requires D>=1.");
    }
  }

  /**
   * Return row-space QCQP equality constraints A, b such that
   * trace(x_i' A x_i) = b[j]. For D=1 these are the lifted SO(2) constraints in
   * column-major coordinates. For D>=2 the same 2-by-2 constraints enforce
   * row orthonormality. The matrix constraints enforce XX'=I, not determinant
   * +1; square D=2 variables therefore admit both components of O(2).
   */
  template <int D = 1>
  static std::vector<std::pair<Matrix, double>> QcqpConstraints() {
    if constexpr (D == 1) {
      // The homogenized Rot2 lifted vector is
      // x = [1, r00, r10, r01, r11].
      std::vector<std::pair<Matrix, double>> constraints;
      constraints.reserve(5);

      Matrix A = Matrix::Zero(5, 5);

      // The quadratic lift fixes x(0)^2 = 1; a hard prior pins its sign.
      A(0, 0) = 1.0;
      constraints.emplace_back(A, 1.0);

      // det(R) = r00*r11 - r10*r01 = 1.
      A.setZero();
      A(1, 4) = 0.5;
      A(4, 1) = 0.5;
      A(2, 3) = -0.5;
      A(3, 2) = -0.5;
      constraints.emplace_back(A, 1.0);

      // RR^T = I supplies the default, non-redundant row-orthonormality
      // Note the reuse of the A variable for multiple constraints.
      A.setZero();
      A(1, 1) = 1.0;
      A(3, 3) = 1.0;
      constraints.emplace_back(A, 1.0);

      A.setZero();
      A(1, 2) = 0.5;
      A(2, 1) = 0.5;
      A(3, 4) = 0.5;
      A(4, 3) = 0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(2, 2) = 1.0;
      A(4, 4) = 1.0;
      constraints.emplace_back(A, 1.0);

      return constraints;
    } else if constexpr (D >= 2) {
      std::vector<std::pair<Matrix, double>> constraints;
      constraints.reserve(3);

      Matrix A = Matrix::Zero(2, 2);
      A(0, 0) = 1.0;
      constraints.emplace_back(A, 1.0);

      A.setZero();
      A(1, 1) = 1.0;
      constraints.emplace_back(A, 1.0);

      A.setZero();
      A(0, 1) = 0.5;
      A(1, 0) = 0.5;
      constraints.emplace_back(A, 0.0);

      return constraints;
    } else {
      throw std::invalid_argument(
          "traits<Rot2>::QcqpConstraints only supports D=1 and D>=2.");
    }
  }

  /**
   * Project a D=1 vector or canonical 2-by-D lift back to Rot2.
   *
   * Matrix-form QCQP solutions have a right-O(D) gauge, making this
   * leading-block projection gauge-dependent unless the caller has first
   * chosen a gauge. Matrix-form X must be exactly 2-by-D.
   */
  template <int D>
  static Rot2 FromQcqpValue(const Matrix& X) {
    if constexpr (D == 1) {
      if (X.rows() != QcqpVectorDim || X.cols() != 1 ||
          std::abs(X(0, 0)) < 1e-9) {
        throw std::invalid_argument(
            "traits<Rot2>::FromQcqpValue requires a 5-by-1 vector with a "
            "nonzero homogenization entry.");
      }
      const Vector x = X.col(0) / X(0, 0);
      Matrix2 R;
      R.col(0) = x.segment<2>(1);
      R.col(1) = x.segment<2>(3);
      return Rot2::ClosestTo(R);
    } else {
      static_assert(D >= 2,
                    "traits<Rot2>::FromQcqpValue requires D >= 2.");
      if (X.rows() != 2 || X.cols() != D) {
        throw std::invalid_argument(
            "traits<Rot2>::FromQcqpValue requires a 2-by-D matrix.");
      }
      return Rot2::ClosestTo(X.template leftCols<2>().transpose());
    }
  }
};

template <>
struct traits<const Rot2> : public traits<Rot2> {};

}  // namespace gtsam
