/* ----------------------------------------------------------------------------
 
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 
 * See LICENSE for the license information
 
 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionFactor.h
 * @brief  Base class to create smart factors on poses or cameras
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/SmartFactorBase.h>

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <boost/optional.hpp>
#include <boost/make_shared.hpp>
#include <vector>

namespace gtsam {

/**
 * SmartStereoProjectionFactor: triangulates point
 */
template<class CALIBRATION>
class SmartStereoProjectionFactor: public SmartFactorBase<StereoCamera> {
protected:

  // Some triangulation parameters
  const double rankTolerance_; ///< threshold to decide whether triangulation is degenerate_
  const double retriangulationThreshold_; ///< threshold to decide whether to re-triangulate
  mutable std::vector<Pose3> cameraPosesTriangulation_; ///< current triangulation poses

  const bool manageDegeneracy_; ///< if set to true will use the rotation-only version for degenerate cases

  const bool enableEPI_; ///< if set to true, will refine triangulation using LM

  const double linearizationThreshold_; ///< threshold to decide whether to re-linearize
  mutable std::vector<Pose3> cameraPosesLinearization_; ///< current linearization poses

  mutable Point3 point_; ///< Current estimate of the 3D point

  mutable bool degenerate_;
  mutable bool cheiralityException_;


  /// shorthand for base class type
  typedef SmartFactorBase<StereoCamera> Base;

  double landmarkDistanceThreshold_; // if the landmark is triangulated at a
  // distance larger than that the factor is considered degenerate

  double dynamicOutlierRejectionThreshold_; // if this is nonnegative the factor will check if the
  // average reprojection error is smaller than this threshold after triangulation,
  // and the factor is disregarded if the error is large

  /// shorthand for this class
  typedef SmartStereoProjectionFactor<CALIBRATION> This;

  enum {
    ZDim = 3
  }; ///< Dimension trait of measurement type

  /// @name Parameters governing how triangulation result is treated
  /// @{
   const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
   const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)
   /// @}

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// shorthand for a StereoCamera // TODO: Get rid of this?
  typedef StereoCamera Camera;

  /// Vector of cameras
  typedef CameraSet<Camera> Cameras;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  SmartStereoProjectionFactor(const double rankTol, const double linThreshold,
      const bool manageDegeneracy, const bool enableEPI,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1) :
      rankTolerance_(rankTol), retriangulationThreshold_(1e-5), manageDegeneracy_(
          manageDegeneracy), enableEPI_(enableEPI), linearizationThreshold_(
          linThreshold), degenerate_(false), cheiralityException_(false), throwCheirality_(
          false), verboseCheirality_(false), landmarkDistanceThreshold_(
          landmarkDistanceThreshold), dynamicOutlierRejectionThreshold_(
          dynamicOutlierRejectionThreshold) {
  }

  /** Virtual destructor */
  virtual ~SmartStereoProjectionFactor() {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartStereoProjectionFactor, z = \n";
    std::cout << "rankTolerance_ = " << rankTolerance_ << std::endl;
    std::cout << "degenerate_ = " << degenerate_ << std::endl;
    std::cout << "cheiralityException_ = " << cheiralityException_ << std::endl;
    std::cout << "linearizationThreshold_ = " << linearizationThreshold_
        << std::endl;
    Base::print("", keyFormatter);
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

        //TODO: Chris will replace this with something else for stereo
//        point_ = triangulatePoint3<CALIBRATION>(cameras, this->measured_,
//            rankTolerance_, enableEPI_);

        // // // Temporary hack to use monocular triangulation
        std::vector<Point2> mono_measurements;
        BOOST_FOREACH(const StereoPoint2& sp, this->measured_) {
          mono_measurements.push_back(sp.point2());
        }

        std::vector<PinholeCamera<Cal3_S2> > mono_cameras;
        BOOST_FOREACH(const Camera& camera, cameras) {
          const Pose3& pose = camera.pose();
          const Cal3_S2& K = camera.calibration()->calibration();
          mono_cameras.push_back(PinholeCamera<Cal3_S2>(pose, K));
        }
        point_ = triangulatePoint3<Cal3_S2>(mono_cameras, mono_measurements,
            rankTolerance_, enableEPI_);

        // // // End temporary hack

        // FIXME: temporary: triangulation using only first camera
//        const StereoPoint2& z0 = this->measured_.at(0);
//        point_ = cameras[0].backproject(z0);

        degenerate_ = false;
        cheiralityException_ = false;

        // Check landmark distance and reprojection errors to avoid outliers
        double totalReprojError = 0.0;
        size_t i = 0;
        BOOST_FOREACH(const Camera& camera, cameras) {
          Point3 cameraTranslation = camera.pose().translation();
          // we discard smart factors corresponding to points that are far away
          if (cameraTranslation.distance(point_) > landmarkDistanceThreshold_) {
            degenerate_ = true;
            break;
          }
          const StereoPoint2& zi = this->measured_.at(i);
          try {
            StereoPoint2 reprojectionError(camera.project(point_) - zi);
            totalReprojError += reprojectionError.vector().norm();
          } catch (CheiralityException) {
            cheiralityException_ = true;
          }
          i += 1;
        }
        //std::cout << "totalReprojError error: " << totalReprojError << std::endl;
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

    if (nrCameras < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_))) {
      if (isDebug) {
        std::cout << "createImplicitSchurFactor: degenerate configuration"
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

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<Base::Dim> > createHessianFactor(
      const Cameras& cameras, const double lambda = 0.0) const {

    bool isDebug = false;
    size_t numKeys = this->keys_.size();
    // Create structures for Hessian Factors
    std::vector<Key> js;
    std::vector<Matrix> Gs(numKeys * (numKeys + 1) / 2);
    std::vector<Vector> gs(numKeys);

    if (this->measured_.size() != cameras.size()) {
      std::cout
          << "SmartProjectionHessianFactor: this->measured_.size() inconsistent with input"
          << std::endl;
      exit(1);
    }

    triangulateSafe(cameras);
    if (isDebug)
      std::cout << "point_ = " << point_ << std::endl;

    if (numKeys < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_))) {
      if (isDebug)
        std::cout << "In linearize: exception" << std::endl;
      BOOST_FOREACH(Matrix& m, Gs)
        m = zeros(Base::Dim, Base::Dim);
      BOOST_FOREACH(Vector& v, gs)
        v = zero(Base::Dim);
      return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_, Gs, gs,
          0.0);
    }

    // instead, if we want to manage the exception..
    if (this->cheiralityException_ || this->degenerate_) { // if we want to manage the exceptions with rotation-only factors
      this->degenerate_ = true;
      if (isDebug)
        std::cout << "degenerate_ = true" << std::endl;
    }

    if (this->linearizationThreshold_ >= 0) // if we apply selective relinearization and we need to relinearize
      for (size_t i = 0; i < cameras.size(); i++)
        this->cameraPosesLinearization_[i] = cameras[i].pose();

    // ==================================================================
    std::vector<typename Base::MatrixZD> Fblocks;
    Matrix F, E;
    Vector b;
    computeJacobians(Fblocks, E, b, cameras);
    Base::FillDiagonalF(Fblocks, F); // expensive !!!

    // Schur complement trick
    // Frank says: should be possible to do this more efficiently?
    // And we care, as in grouped factors this is called repeatedly
    Matrix H(Base::Dim * numKeys, Base::Dim * numKeys);
    Vector gs_vector;

    Matrix3 P = Cameras::PointCov(E, lambda);
    H.noalias() = F.transpose() * (F - (E * (P * (E.transpose() * F))));
    gs_vector.noalias() = F.transpose() * (b - (E * (P * (E.transpose() * b))));

    if (isDebug)
      std::cout << "gs_vector size " << gs_vector.size() << std::endl;
    if (isDebug)
      std::cout << "H:\n" << H << std::endl;

    // Populate Gs and gs
    int GsCount2 = 0;
    for (DenseIndex i1 = 0; i1 < (DenseIndex) numKeys; i1++) { // for each camera
      DenseIndex i1D = i1 * Base::Dim;
      gs.at(i1) = gs_vector.segment<Base::Dim>(i1D);
      for (DenseIndex i2 = 0; i2 < (DenseIndex) numKeys; i2++) {
        if (i2 >= i1) {
          Gs.at(GsCount2) = H.block<Base::Dim, Base::Dim>(i1D, i2 * Base::Dim);
          GsCount2++;
        }
      }
    }
    // ==================================================================
    double f = b.squaredNorm();
    return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_, Gs, gs, f);
  }

//  // create factor
//  boost::shared_ptr<ImplicitSchurFactor<Base::Dim> > createImplicitSchurFactor(
//      const Cameras& cameras, double lambda) const {
//    if (triangulateForLinearize(cameras))
//      return Base::createImplicitSchurFactor(cameras, point_, lambda);
//    else
//      return boost::shared_ptr<ImplicitSchurFactor<Base::Dim> >();
//  }
//
//  /// create factor
//  boost::shared_ptr<JacobianFactorQ<Base::Dim> > createJacobianQFactor(
//      const Cameras& cameras, double lambda) const {
//    if (triangulateForLinearize(cameras))
//      return Base::createJacobianQFactor(cameras, point_, lambda);
//    else
//      return boost::make_shared< JacobianFactorQ<Base::Dim> >(this->keys_);
//  }
//
//  /// Create a factor, takes values
//  boost::shared_ptr<JacobianFactorQ<Base::Dim> > createJacobianQFactor(
//      const Values& values, double lambda) const {
//    Cameras cameras;
//    // TODO triangulate twice ??
//    bool nonDegenerate = computeCamerasAndTriangulate(values, cameras);
//    if (nonDegenerate)
//      return createJacobianQFactor(cameras, lambda);
//    else
//      return boost::make_shared< JacobianFactorQ<Base::Dim> >(this->keys_);
//  }
//
  /// different (faster) way to compute Jacobian factor
  boost::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianSVDFactor(cameras, point_, lambda);
    else
      return boost::make_shared<JacobianFactorSVD<Base::Dim, ZDim> >(this->keys_);
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

    if (nrCameras < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_)))
      return false;

    // instead, if we want to manage the exception..
    if (this->cheiralityException_ || this->degenerate_) // if we want to manage the exceptions with rotation-only factors
      this->degenerate_ = true;

    if (this->degenerate_) {
      std::cout << "SmartStereoProjectionFactor: this is not ready"
          << std::endl;
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

  /// Version that takes values, and creates the point
  bool computeJacobians(std::vector<typename Base::MatrixZD>& Fblocks,
      Matrix& E, Vector& b, const Values& values) const {
    Cameras cameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, cameras);
    if (nonDegenerate)
      computeJacobians(Fblocks, E, b, cameras, 0.0);
    return nonDegenerate;
  }

  /// Compute F, E only (called below in both vanilla and SVD versions)
  /// Assumes the point has been computed
  /// Note E can be 2m*3 or 2m*2, in case point is degenerate
  void computeJacobians(std::vector<typename Base::MatrixZD>& Fblocks,
      Matrix& E, Vector& b, const Cameras& cameras) const {
    if (this->degenerate_) {
      throw("FIXME: computeJacobians degenerate case commented out!");
//      std::cout << "manage degeneracy " << manageDegeneracy_ << std::endl;
//      std::cout << "point " << point_ << std::endl;
//      std::cout
//          << "SmartStereoProjectionFactor: Management of degeneracy is disabled - not ready to be used"
//          << std::endl;
//      if (D > 6) {
//        std::cout
//            << "Management of degeneracy is not yet ready when one also optimizes for the calibration "
//            << std::endl;
//      }
//
//      int numKeys = this->keys_.size();
//      E = zeros(2 * numKeys, 2);
//      b = zero(2 * numKeys);
//      double f = 0;
//      for (size_t i = 0; i < this->measured_.size(); i++) {
//        if (i == 0) { // first pose
//          this->point_ = cameras[i].backprojectPointAtInfinity(
//              this->measured_.at(i));
//          // 3D parametrization of point at infinity: [px py 1]
//        }
//        Matrix Fi, Ei;
//        Vector bi = -(cameras[i].projectPointAtInfinity(this->point_, Fi, Ei)
//            - this->measured_.at(i)).vector();
//
//        this->noise_.at(i)->WhitenSystem(Fi, Ei, bi);
//        f += bi.squaredNorm();
//        Fblocks.push_back(typename Base::MatrixZD(this->keys_[i], Fi));
//        E.block < 2, 2 > (2 * i, 0) = Ei;
//        subInsert(b, bi, 2 * i);
//      }
//      return f;
    } else {
      // nondegenerate: just return Base version
      Base::computeJacobians(Fblocks, E, b, cameras, point_);
    } // end else
  }

  /// takes values
  bool triangulateAndComputeJacobiansSVD(
      std::vector<typename Base::MatrixZD>& Fblocks, Matrix& Enull, Vector& b,
      const Values& values) const {
    typename Base::Cameras cameras;
    double good = computeCamerasAndTriangulate(values, cameras);
    if (good)
      return Base::computeJacobiansSVD(Fblocks, Enull, b, cameras, point_);
    return true;
  }

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionErrorAfterTriangulation(const Values& values) const {
    Cameras cameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, cameras);
    if (nonDegenerate)
      return Base::unwhitenedError(cameras, point_);
    else
      return zero(cameras.size() * 3);
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

    if (nrCameras < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_))) {
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
      return 0.0; // TODO: this maybe should be zero?
//      std::cout
//          << "SmartProjectionHessianFactor: trying to manage degeneracy (this should not happen is manageDegeneracy is disabled)!"
//          << std::endl;
//      size_t i = 0;
//      double overallError = 0;
//      BOOST_FOREACH(const Camera& camera, cameras) {
//        const StereoPoint2& zi = this->measured_.at(i);
//        if (i == 0) // first pose
//          this->point_ = camera.backprojectPointAtInfinity(zi); // 3D parametrization of point at infinity
//        StereoPoint2 reprojectionError(
//            camera.projectPointAtInfinity(this->point_) - zi);
//        overallError += 0.5
//            * this->noise_.at(i)->distance(reprojectionError.vector());
//        i += 1;
//      }
//      return overallError;
    } else {
      // Just use version in base class
      return Base::totalReprojectionError(cameras, point_);
    }
  }

  /// Cameras are computed in derived class
  virtual Cameras cameras(const Values& values) const = 0;

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
  /** return chirality verbosity */
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

/// traits
template<class CALIBRATION>
struct traits<SmartStereoProjectionFactor<CALIBRATION> > : public Testable<
    SmartStereoProjectionFactor<CALIBRATION> > {
};

} // \ namespace gtsam
