/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2Stereo.h
 * @brief  The most common 5DOF 3D->2D calibration + Stereo baseline
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <iosfwd>

namespace gtsam {

  /**
   * @brief The most common 5DOF 3D->2D calibration, stereo version
   * @addtogroup geometry
   * \nosubgrouping
   */
  class GTSAM_EXPORT Cal3_S2Stereo {
  private:

    Cal3_S2 K_;
    double b_;

  public:

    enum { dimension = 6 };
    typedef boost::shared_ptr<Cal3_S2Stereo> shared_ptr;  ///< shared pointer to stereo calibration object

    /// @name Standard Constructors
    /// @

    /// default calibration leaves coordinates unchanged
    Cal3_S2Stereo() :
      K_(1, 1, 0, 0, 0), b_(1.0) {
    }

    /// constructor from doubles
    Cal3_S2Stereo(double fx, double fy, double s, double u0, double v0, double b) :
      K_(fx, fy, s, u0, v0), b_(b) {
    }

    /// constructor from vector
    Cal3_S2Stereo(const Vector &d): K_(d(0), d(1), d(2), d(3), d(4)), b_(d(5)){}

    /// easy constructor; field-of-view in degrees, assumes zero skew
    Cal3_S2Stereo(double fov, int w, int h, double b) :
      K_(fov, w, h), b_(b) {
    }

    /// @}
    /// @name Testable
    /// @{

    void print(const std::string& s = "") const;

    /// Check if equal up to specified tolerance
    bool equals(const Cal3_S2Stereo& other, double tol = 10e-9) const;

   /// @}
    /// @name Standard Interface
    /// @{

    /// return calibration, same for left and right
    const Cal3_S2& calibration() const { return K_;}

    /// return calibration matrix K, same for left and right
    Matrix matrix() const { return K_.matrix();}

    /// focal length x
    inline double fx() const { return K_.fx();}

    /// focal length x
    inline double fy() const { return K_.fy();}

    /// skew
    inline double skew() const { return K_.skew();}

    /// image center in x
    inline double px() const { return K_.px();}

    /// image center in y
    inline double py() const { return K_.py();}

    /// return the principal point
    Point2 principalPoint() const { return K_.principalPoint();}

    /// return baseline
    inline double baseline() const { return b_; }

    /// vectorized form (column-wise)
    Vector6 vector() const {
      Vector6 v;
      v << K_.vector(), b_;
      return v;
    }

    /// @}
    /// @name Manifold
    /// @{

    /// return DOF, dimensionality of tangent space
    inline size_t dim() const { return dimension; }

    /// return DOF, dimensionality of tangent space
    static size_t Dim() { return dimension; }

    /// Given 6-dim tangent vector, create new calibration
    inline Cal3_S2Stereo retract(const Vector& d) const {
      return Cal3_S2Stereo(K_.fx() + d(0), K_.fy() + d(1), K_.skew() + d(2), K_.px() + d(3), K_.py() + d(4), b_ + d(5));
    }

    /// Unretraction for the calibration
    Vector6 localCoordinates(const Cal3_S2Stereo& T2) const {
      return T2.vector() - vector();
    }


    /// @}
    /// @name Advanced Interface
    /// @{

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(b_);
    }
    /// @}

  };

  // Define GTSAM traits
  template<>
  struct traits<Cal3_S2Stereo> : public internal::Manifold<Cal3_S2Stereo> {
  };

  template<>
  struct traits<const Cal3_S2Stereo> : public internal::Manifold<Cal3_S2Stereo> {
  };

} // \ namespace gtsam
