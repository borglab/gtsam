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

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Lie.h>

#include <random>

namespace gtsam {

  /**
   * Rotation matrix
   * NOTE: the angle theta is in radians unless explicitly stated
   * @ingroup geometry
   * \nosubgrouping
   */
  class GTSAM_EXPORT Rot2 : public LieGroup<Rot2, 1> {

    /** we store cos(theta) and sin(theta) */
    double c_, s_;

    /** normalize to make sure cos and sin form unit vector */
    Rot2& normalize();

    /** private constructor from cos/sin */
    inline Rot2(double c, double s) : c_(c), s_(s) {}

  public:

    /// @name Constructors and named constructors
    /// @{

    /** default constructor, zero rotation */
    Rot2() : c_(1.0), s_(0.0) {}
    
    /** copy constructor */
    Rot2(const Rot2& r) : Rot2(r.c_, r.s_) {}

    /// Constructor from angle in radians == exponential map at identity
    Rot2(double theta) : c_(cos(theta)), s_(sin(theta)) {}

    /// Named constructor from angle in radians
    static Rot2 fromAngle(double theta) {
      return Rot2(theta);
    }

    /// Named constructor from angle in degrees
    static Rot2 fromDegrees(double theta) {
      static const double degree = M_PI / 180;
      return fromAngle(theta * degree);
    }

    /// Named constructor from cos(theta),sin(theta) pair, will *not* normalize!
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

    /// Exponential map at identity - create a rotation from canonical coordinates
    static Rot2 Expmap(const Vector1& v, ChartJacobian H = {});

    /// Log map at identity - return the canonical coordinates of this rotation
    static Vector1 Logmap(const Rot2& r, ChartJacobian H = {});

    /** Calculate Adjoint map */
    Matrix1 AdjointMap() const { return I_1x1; }

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

    /** return 2*2 transpose (inverse) rotation matrix   */
    Matrix2 transpose() const;

    /** Find closest valid rotation matrix, given a 2x2 matrix */
    static Rot2 ClosestTo(const Matrix2& M);

  private:
    /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(c_);
      ar & BOOST_SERIALIZATION_NVP(s_);
    }
#endif

  };

  template<>
  struct traits<Rot2> : public internal::LieGroup<Rot2> {};

  template<>
  struct traits<const Rot2> : public internal::LieGroup<Rot2> {};

} // gtsam
