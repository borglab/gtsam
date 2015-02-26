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

  const double rankTolerance_; ///< threshold to decide whether triangulation is result.degenerate
  const bool enableEPI_; ///< if set to true, will refine triangulation using LM

  /**
   * if the landmark is triangulated at distance larger than this,
   * result is flagged as degenerate.
   */
  const double landmarkDistanceThreshold_; //

  /**
   * If this is nonnegative the we will check if the average reprojection error
   * is smaller than this threshold after triangulation, otherwise result is
   * flagged as degenerate.
   */
  const double dynamicOutlierRejectionThreshold_;
  /// @}

public:

  /// @name Triangulation result
  /// @{
  struct Result {
    Point3 point;
    bool degenerate;
    bool cheiralityException;
  };
  /// @}

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   */
  PinholeSet(const double rankTol = 1.0, const bool enableEPI = false,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
      rankTolerance_(rankTol), enableEPI_(enableEPI), landmarkDistanceThreshold_(
          landmarkDistanceThreshold), dynamicOutlierRejectionThreshold_(
          dynamicOutlierRejectionThreshold) {
  }

  /** Virtual destructor */
  virtual ~PinholeSet() {
  }

  /// @name Testable
  /// @{

  /// print
  virtual void print(const std::string& s = "") const {
    Base::print(s);
    std::cout << s << "PinholeSet\n";
    std::cout << "rankTolerance = " << rankTolerance_ << std::endl;
  }

  /// equals
  bool equals(const PinholeSet& p, double tol = 1e-9) const {
    return Base::equals(p, tol); // TODO all flags
  }

  /// @}

  /// triangulateSafe
  Result triangulateSafe(const std::vector<typename Base::Z>& measured) const {

    Result result;

    size_t m = this->size();

    // if we have a single pose the corresponding factor is uninformative
    if (m < 2) {
      result.degenerate = true;
      return result;
    }

    // We triangulate the 3D position of the landmark
    try {
      // std::cout << "triangulatePoint3 i \n" << rankTolerance << std::endl;
      result.point = triangulatePoint3<CAMERA>(*this, measured, rankTolerance_,
          enableEPI_);
      result.degenerate = false;
      result.cheiralityException = false;

      // Check landmark distance and reprojection errors to avoid outliers
      double totalReprojError = 0.0;
      size_t i = 0;
      BOOST_FOREACH(const CAMERA& camera, *this) {
        Point3 cameraTranslation = camera.pose().translation();
        // we discard smart factors corresponding to points that are far away
        if (cameraTranslation.distance(result.point)
            > landmarkDistanceThreshold_) {
          result.degenerate = true;
          break;
        }
        const Point2& zi = measured.at(i);
        try {
          Point2 reprojectionError(camera.project(result.point) - zi);
          totalReprojError += reprojectionError.vector().norm();
        } catch (CheiralityException) {
          result.cheiralityException = true;
        }
        i += 1;
      }
      // we discard smart factors that have large reprojection error
      if (dynamicOutlierRejectionThreshold_ > 0
          && totalReprojError / m > dynamicOutlierRejectionThreshold_)
        result.degenerate = true;

    } catch (TriangulationUnderconstrainedException&) {
      // if TriangulationUnderconstrainedException can be
      // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
      // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel *this (or motion towards the landmark)
      // in the second case we want to use a rotation-only smart factor
      result.degenerate = true;
      result.cheiralityException = false;
    } catch (TriangulationCheiralityException&) {
      // point is behind one of the *this: can be the case of close-to-parallel *this or may depend on outliers
      // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
      result.cheiralityException = true;
    }
    return result;
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
