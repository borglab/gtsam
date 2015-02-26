/* ----------------------------------------------------------------------------
 
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 
 * See LICENSE for the license information
 
 * -------------------------------------------------------------------------- */

/**
 * @file   PinholeSet.h
 * @brief  A CameraSet of either CalibratedCamera, PinholePose, or PinholeCamera
 * @author Frank Dellaert
 * @author Luca Carlone
 * @author Zsolt Kira
 */

#pragma once

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/triangulation.h>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

namespace gtsam {

/**
 * PinholeSet: triangulates point and keeps an estimate of it around.
 */
template<class CAMERA>
class PinholeSet: public CameraSet<CAMERA> {

private:
  typedef CameraSet<CAMERA> Base;
  typedef PinholeSet<CAMERA> This;

protected:

  /// @name Triangulation parameters
  /// @{

  const double rankTolerance_; ///< threshold to decide whether triangulation is degenerate_
  const bool enableEPI_; ///< if set to true, will refine triangulation using LM

  /// @}

  mutable Point3 point_; ///< Current estimate of the 3D point

  mutable bool degenerate_;
  mutable bool cheiralityException_;

  double landmarkDistanceThreshold_; // if the landmark is triangulated at a
  // distance larger than that the factor is considered degenerate

  double dynamicOutlierRejectionThreshold_; // if this is nonnegative the factor will check if the
  // average reprojection error is smaller than this threshold after triangulation,
  // and the factor is disregarded if the error is large

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   */
  PinholeSet(const double rankTol = 1.0, const bool enableEPI = false,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
      rankTolerance_(rankTol), enableEPI_(enableEPI), degenerate_(false), cheiralityException_(
          false), landmarkDistanceThreshold_(landmarkDistanceThreshold), dynamicOutlierRejectionThreshold_(
          dynamicOutlierRejectionThreshold) {
  }

  /** Virtual destructor */
  virtual ~PinholeSet() {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "") const {
    std::cout << s << "PinholeSet, z = \n";
    std::cout << "rankTolerance_ = " << rankTolerance_ << std::endl;
    std::cout << "degenerate_ = " << degenerate_ << std::endl;
    std::cout << "cheiralityException_ = " << cheiralityException_ << std::endl;
    Base::print("");
  }

  /// equals
  bool equals(const PinholeSet& p, double tol = 1e-9) const {
    return Base::equals(p, tol); // TODO all flags
  }

  /// triangulateSafe
  size_t triangulateSafe() const {

    size_t m = this->size();
    if (m < 2) { // if we have a single pose the corresponding factor is uninformative
      degenerate_ = true;
      return m;
    }

    // We triangulate the 3D position of the landmark
    try {
      // std::cout << "triangulatePoint3 i \n" << rankTolerance << std::endl;
      point_ = triangulatePoint3<CAMERA>(*this, this->measured_, rankTolerance_,
          enableEPI_);
      degenerate_ = false;
      cheiralityException_ = false;

      // Check landmark distance and reprojection errors to avoid outliers
      double totalReprojError = 0.0;
      size_t i = 0;
      BOOST_FOREACH(const CAMERA& camera, *this) {
        Point3 cameraTranslation = camera.pose().translation();
        // we discard smart factors corresponding to points that are far away
        if (cameraTranslation.distance(point_) > landmarkDistanceThreshold_) {
          degenerate_ = true;
          break;
        }
        const Point2& zi = this->measured_.at(i);
        try {
          Point2 reprojectionError(camera.project(point_) - zi);
          totalReprojError += reprojectionError.vector().norm();
        } catch (CheiralityException) {
          cheiralityException_ = true;
        }
        i += 1;
      }
      // we discard smart factors that have large reprojection error
      if (dynamicOutlierRejectionThreshold_ > 0
          && totalReprojError / m > dynamicOutlierRejectionThreshold_)
        degenerate_ = true;

    } catch (TriangulationUnderconstrainedException&) {
      // if TriangulationUnderconstrainedException can be
      // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
      // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel *this (or motion towards the landmark)
      // in the second case we want to use a rotation-only smart factor
      degenerate_ = true;
      cheiralityException_ = false;
    } catch (TriangulationCheiralityException&) {
      // point is behind one of the *this: can be the case of close-to-parallel *this or may depend on outliers
      // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
      cheiralityException_ = true;
    }
    return m;
  }

  /// triangulate
  bool triangulateForLinearize() const {

    bool isDebug = false;
    size_t nrCameras = this->triangulateSafe(*this);

    if (nrCameras < 2 || (this->cheiralityException_ || this->degenerate_)) {
      if (isDebug) {
        std::cout
            << "createRegularImplicitSchurFactor: degenerate configuration"
            << std::endl;
      }
      return false;
    } else {

      // instead, if we want to manage the exception..
      if (this->cheiralityException_ || this->degenerate_) { // if we want to manage the exceptions with rotation-only factors
        this->degenerate_ = true;
      }
      return true;
    }
  }

  /** return the landmark */
  boost::optional<Point3> point() const {
    return point_;
  }

  /** COMPUTE the landmark */
  boost::optional<Point3> point(const Values& values) const {
    triangulateSafe(values);
    return point_;
  }

  /** return the degenerate state */
  inline bool isDegenerate() const {
    return (cheiralityException_ || degenerate_);
  }

  /** return the cheirality status flag */
  inline bool isPointBehindCamera() const {
    return cheiralityException_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

template<class CAMERA>
struct traits<PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

template<class CAMERA>
struct traits<const PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

} // \ namespace gtsam
