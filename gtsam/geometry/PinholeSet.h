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

  // Some triangulation parameters
  const double rankTolerance_; ///< threshold to decide whether triangulation is degenerate_
  const double retriangulationThreshold_; ///< threshold to decide whether to re-triangulate
  mutable std::vector<Pose3> cameraPosesTriangulation_; ///< current triangulation poses

  const bool enableEPI_; ///< if set to true, will refine triangulation using LM

  mutable Point3 point_; ///< Current estimate of the 3D point

  mutable bool degenerate_;
  mutable bool cheiralityException_;

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  double landmarkDistanceThreshold_; // if the landmark is triangulated at a
  // distance larger than that the factor is considered degenerate

  double dynamicOutlierRejectionThreshold_; // if this is nonnegative the factor will check if the
  // average reprojection error is smaller than this threshold after triangulation,
  // and the factor is disregarded if the error is large

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// shorthand for a set of cameras
  typedef CameraSet<CAMERA> Cameras;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   */
  PinholeSet(const double rankTol = 1.0, const bool enableEPI = false,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
      rankTolerance_(rankTol), retriangulationThreshold_(1e-5), enableEPI_(
          enableEPI), degenerate_(false), cheiralityException_(false), throwCheirality_(
          false), verboseCheirality_(false), landmarkDistanceThreshold_(
          landmarkDistanceThreshold), dynamicOutlierRejectionThreshold_(
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

  /// Check if the new linearization point_ is the same as the one used for previous triangulation
  bool decideIfTriangulate(const Cameras& cameras) const {
    // several calls to linearize will be done from the same linearization point_, hence it is not needed to re-triangulate
    // Note that this is not yet "selecting linearization", that will come later, and we only check if the
    // current linearization is the "same" (up to tolerance) w.r.t. the last time we triangulated the point_

    size_t m = cameras.size();

    bool retriangulate = false;

    // if we do not have a previous linearization point_ or the new linearization point_ includes more poses
    if (cameraPosesTriangulation_.empty()
        || cameras.size() != cameraPosesTriangulation_.size())
      retriangulate = true;

    if (!retriangulate) {
      for (size_t i = 0; i < cameras.size(); i++) {
        if (!cameras[i].pose().equals(cameraPosesTriangulation_[i],
            retriangulationThreshold_)) {
          retriangulate = true; // at least two poses are different, hence we retriangulate
          break;
        }
      }
    }

    if (retriangulate) { // we store the current poses used for triangulation
      cameraPosesTriangulation_.clear();
      cameraPosesTriangulation_.reserve(m);
      for (size_t i = 0; i < m; i++)
        // cameraPosesTriangulation_[i] = cameras[i].pose();
        cameraPosesTriangulation_.push_back(cameras[i].pose());
    }

    return retriangulate; // if we arrive to this point_ all poses are the same and we don't need re-triangulation
  }

  /// triangulateSafe
  size_t triangulateSafe(const Values& values) const {
    return triangulateSafe(this->cameras(values));
  }

  /// triangulateSafe
  size_t triangulateSafe(const Cameras& cameras) const {

    size_t m = cameras.size();
    if (m < 2) { // if we have a single pose the corresponding factor is uninformative
      degenerate_ = true;
      return m;
    }
    bool retriangulate = decideIfTriangulate(cameras);

    if (retriangulate) {
      // We triangulate the 3D position of the landmark
      try {
        // std::cout << "triangulatePoint3 i \n" << rankTolerance << std::endl;
        point_ = triangulatePoint3<CAMERA>(cameras, this->measured_,
            rankTolerance_, enableEPI_);
        degenerate_ = false;
        cheiralityException_ = false;

        // Check landmark distance and reprojection errors to avoid outliers
        double totalReprojError = 0.0;
        size_t i = 0;
        BOOST_FOREACH(const CAMERA& camera, cameras) {
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
        // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
        // in the second case we want to use a rotation-only smart factor
        degenerate_ = true;
        cheiralityException_ = false;
      } catch (TriangulationCheiralityException&) {
        // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
        // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
        cheiralityException_ = true;
      }
    }
    return m;
  }

  /// triangulate
  bool triangulateForLinearize(const Cameras& cameras) const {

    bool isDebug = false;
    size_t nrCameras = this->triangulateSafe(cameras);

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

  /// Returns true if nonDegenerate
  bool computeCamerasAndTriangulate(const Values& values,
      Cameras& cameras) const {
    Values valuesFactor;

    // Select only the cameras
    BOOST_FOREACH(const Key key, this->keys_)
      valuesFactor.insert(key, values.at(key));

    cameras = this->cameras(valuesFactor);
    size_t nrCameras = this->triangulateSafe(cameras);

    if (nrCameras < 2 || (this->cheiralityException_ || this->degenerate_))
      return false;

    // instead, if we want to manage the exception..
    if (this->cheiralityException_ || this->degenerate_) // if we want to manage the exceptions with rotation-only factors
      this->degenerate_ = true;

    if (this->degenerate_) {
      std::cout << "PinholeSet: this is not ready" << std::endl;
      std::cout << "this->cheiralityException_ " << this->cheiralityException_
          << std::endl;
      std::cout << "this->degenerate_ " << this->degenerate_ << std::endl;
    }
    return true;
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Values& values) const {
    Cameras cameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, cameras);
    if (nonDegenerate)
      cameras.project2(point_, boost::none, E);
    return nonDegenerate;
  }

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionErrorAfterTriangulation(const Values& values) const {
    Cameras cameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, cameras);
    if (nonDegenerate)
      return Base::reprojectionError(cameras, point_);
    else
      return zero(cameras.size() * 2);
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  double totalReprojectionError(const Cameras& cameras,
      boost::optional<Point3> externalPoint = boost::none) const {

    size_t nrCameras;
    if (externalPoint) {
      nrCameras = this->keys_.size();
      point_ = *externalPoint;
      degenerate_ = false;
      cheiralityException_ = false;
    } else {
      nrCameras = this->triangulateSafe(cameras);
    }

    if (nrCameras < 2 || (this->cheiralityException_ || this->degenerate_)) {
      // if we don't want to manage the exceptions we discard the factor
      // std::cout << "In error evaluation: exception" << std::endl;
      return 0.0;
    }

    if (this->cheiralityException_) { // if we want to manage the exceptions with rotation-only factors
      std::cout
          << "SmartProjectionHessianFactor: cheirality exception (this should not happen if CheiralityException is disabled)!"
          << std::endl;
      this->degenerate_ = true;
    }

    if (this->degenerate_) {
      // return 0.0; // TODO: this maybe should be zero?
      std::cout
          << "SmartProjectionHessianFactor: trying to manage degeneracy (this should not happen is manageDegeneracy is disabled)!"
          << std::endl;
      // 3D parameterization of point at infinity
      const Point2& zi = this->measured_.at(0);
      this->point_ = cameras.front().backprojectPointAtInfinity(zi);
      return Base::totalReprojectionErrorAtInfinity(cameras, this->point_);
    } else {
      // Just use version in base class
      return Base::totalReprojectionError(cameras, point_);
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

  /** return cheirality verbosity */
  inline bool verboseCheirality() const {
    return verboseCheirality_;
  }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const {
    return throwCheirality_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};

template<class CAMERA>
struct traits<PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

template<class CAMERA>
struct traits<const PinholeSet<CAMERA> > : public Testable<PinholeSet<CAMERA> > {
};

} // \ namespace gtsam
