/* ----------------------------------------------------------------------------
 
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 
 * See LICENSE for the license information
 
 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionFactor.h
 * @brief  Base class to create smart factors on poses or cameras
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include "SmartFactorBase.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <boost/optional.hpp>
#include <boost/make_shared.hpp>
#include <vector>

namespace gtsam {

/**
 * Structure for storing some state memory, used to speed up optimization
 * @addtogroup SLAM
 */
class SmartProjectionFactorState {

protected:

public:

  SmartProjectionFactorState() {
  }
  // Hessian representation (after Schur complement)
  bool calculatedHessian;
  Matrix H;
  Vector gs_vector;
  std::vector<Matrix> Gs;
  std::vector<Vector> gs;
  double f;
};

enum LinearizationMode {
  HESSIAN, JACOBIAN_SVD, JACOBIAN_Q
};

/**
 * SmartProjectionFactor: triangulates point
 * TODO: why LANDMARK parameter?
 */
template<class POSE, class LANDMARK, class CALIBRATION, size_t D>
class SmartProjectionFactor: public SmartFactorBase<POSE, CALIBRATION, D> {
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

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  boost::shared_ptr<SmartProjectionFactorState> state_;

  /// shorthand for smart projection factor state variable
  typedef boost::shared_ptr<SmartProjectionFactorState> SmartFactorStatePtr;

  /// shorthand for base class type
  typedef SmartFactorBase<POSE, CALIBRATION, D> Base;

  double landmarkDistanceThreshold_; // if the landmark is triangulated at a
  // distance larger than that the factor is considered degenerate

  double dynamicOutlierRejectionThreshold_; // if this is nonnegative the factor will check if the
  // average reprojection error is smaller than this threshold after triangulation,
  // and the factor is disregarded if the error is large

  /// shorthand for this class
  typedef SmartProjectionFactor<POSE, LANDMARK, CALIBRATION, D> This;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// shorthand for a pinhole camera
  typedef PinholeCamera<CALIBRATION> Camera;
  typedef std::vector<Camera> Cameras;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  SmartProjectionFactor(const double rankTol, const double linThreshold,
      const bool manageDegeneracy, const bool enableEPI,
      boost::optional<POSE> body_P_sensor = boost::none,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1,
      SmartFactorStatePtr state = SmartFactorStatePtr(new SmartProjectionFactorState())) :
      Base(body_P_sensor), rankTolerance_(rankTol), retriangulationThreshold_(
          1e-5), manageDegeneracy_(manageDegeneracy), enableEPI_(enableEPI), linearizationThreshold_(
          linThreshold), degenerate_(false), cheiralityException_(false), throwCheirality_(
          false), verboseCheirality_(false), state_(state),
          landmarkDistanceThreshold_(landmarkDistanceThreshold),
          dynamicOutlierRejectionThreshold_(dynamicOutlierRejectionThreshold) {
  }

  /** Virtual destructor */
  virtual ~SmartProjectionFactor() {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const {
    std::cout << s << "SmartProjectionFactor, z = \n";
    std::cout << "rankTolerance_ = " << rankTolerance_ << std::endl;
    std::cout << "degenerate_ = " << degenerate_ << std::endl;
    std::cout << "cheiralityException_ = " << cheiralityException_ << std::endl;
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

  /// This function checks if the new linearization point_ is 'close'  to the previous one used for linearization
  bool decideIfLinearize(const Cameras& cameras) const {
    // "selective linearization"
    // The function evaluates how close are the old and the new poses, transformed in the ref frame of the first pose
    // (we only care about the "rigidity" of the poses, not about their absolute pose)

    if (this->linearizationThreshold_ < 0) //by convention if linearizationThreshold is negative we always relinearize
      return true;

    // if we do not have a previous linearization point_ or the new linearization point_ includes more poses
    if (cameraPosesLinearization_.empty()
        || (cameras.size() != cameraPosesLinearization_.size()))
      return true;

    Pose3 firstCameraPose, firstCameraPoseOld;
    for (size_t i = 0; i < cameras.size(); i++) {

      if (i == 0) { // we store the initial pose, this is useful for selective re-linearization
        firstCameraPose = cameras[i].pose();
        firstCameraPoseOld = cameraPosesLinearization_[i];
        continue;
      }

      // we compare the poses in the frame of the first pose
      Pose3 localCameraPose = firstCameraPose.between(cameras[i].pose());
      Pose3 localCameraPoseOld = firstCameraPoseOld.between(
          cameraPosesLinearization_[i]);
      if (!localCameraPose.equals(localCameraPoseOld,
          this->linearizationThreshold_))
        return true; // at least two "relative" poses are different, hence we re-linearize
    }
    return false; // if we arrive to this point_ all poses are the same and we don't need re-linearize
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
        point_ = triangulatePoint3<CALIBRATION>(cameras, this->measured_,
            rankTolerance_, enableEPI_);
        degenerate_ = false;
        cheiralityException_ = false;

        // Check landmark distance and reprojection errors to avoid outliers
        double totalReprojError = 0.0;
        size_t i=0;
        BOOST_FOREACH(const Camera& camera, cameras) {
          Point3 cameraTranslation = camera.pose().translation();
          // we discard smart factors corresponding to points that are far away
          if(cameraTranslation.distance(point_) > landmarkDistanceThreshold_){
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
        if(dynamicOutlierRejectionThreshold_ > 0 &&
            totalReprojError/m > dynamicOutlierRejectionThreshold_)
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
  boost::shared_ptr<RegularHessianFactor<D> > createHessianFactor(
      const Cameras& cameras, const double lambda = 0.0) const {

    bool isDebug = false;
    size_t numKeys = this->keys_.size();
    // Create structures for Hessian Factors
    std::vector < Key > js;
    std::vector < Matrix > Gs(numKeys * (numKeys + 1) / 2);
    std::vector < Vector > gs(numKeys);

    if (this->measured_.size() != cameras.size()) {
      std::cout
          << "SmartProjectionHessianFactor: this->measured_.size() inconsistent with input"
          << std::endl;
      exit(1);
    }

    this->triangulateSafe(cameras);

    if (numKeys < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_))) {
      // std::cout << "In linearize: exception" << std::endl;
      BOOST_FOREACH(gtsam::Matrix& m, Gs)
        m = zeros(D, D);
      BOOST_FOREACH(Vector& v, gs)
        v = zero(D);
      return boost::make_shared<RegularHessianFactor<D> >(this->keys_, Gs, gs,
          0.0);
    }

    // instead, if we want to manage the exception..
    if (this->cheiralityException_ || this->degenerate_) { // if we want to manage the exceptions with rotation-only factors
      this->degenerate_ = true;
    }

    bool doLinearize = this->decideIfLinearize(cameras);

    if (this->linearizationThreshold_ >= 0 && doLinearize) // if we apply selective relinearization and we need to relinearize
      for (size_t i = 0; i < cameras.size(); i++)
        this->cameraPosesLinearization_[i] = cameras[i].pose();

    if (!doLinearize) { // return the previous Hessian factor
      std::cout << "=============================" << std::endl;
      std::cout << "doLinearize " << doLinearize << std::endl;
      std::cout << "this->linearizationThreshold_ "
          << this->linearizationThreshold_ << std::endl;
      std::cout << "this->degenerate_ " << this->degenerate_ << std::endl;
      std::cout
          << "something wrong in SmartProjectionHessianFactor: selective relinearization should be disabled"
          << std::endl;
      exit(1);
      return boost::make_shared<RegularHessianFactor<D> >(this->keys_,
          this->state_->Gs, this->state_->gs, this->state_->f);
    }

    // ==================================================================
    Matrix F, E;
    Matrix3 PointCov;
    Vector b;
    double f = computeJacobians(F, E, PointCov, b, cameras, lambda);

    // Schur complement trick
    // Frank says: should be possible to do this more efficiently?
    // And we care, as in grouped factors this is called repeatedly
    Matrix H(D * numKeys, D * numKeys);
    Vector gs_vector;

    H.noalias() = F.transpose() * (F - (E * (PointCov * (E.transpose() * F))));
    gs_vector.noalias() = F.transpose()
        * (b - (E * (PointCov * (E.transpose() * b))));
    if (isDebug)
      std::cout << "gs_vector size " << gs_vector.size() << std::endl;

    // Populate Gs and gs
    int GsCount2 = 0;
    for (DenseIndex i1 = 0; i1 < (DenseIndex)numKeys; i1++) { // for each camera
      DenseIndex i1D = i1 * D;
      gs.at(i1) = gs_vector.segment < D > (i1D);
      for (DenseIndex i2 = 0; i2 < (DenseIndex)numKeys; i2++) {
        if (i2 >= i1) {
          Gs.at(GsCount2) = H.block < D, D > (i1D, i2 * D);
          GsCount2++;
        }
      }
    }
    // ==================================================================
    if (this->linearizationThreshold_ >= 0) { // if we do not use selective relinearization we don't need to store these variables
      this->state_->Gs = Gs;
      this->state_->gs = gs;
      this->state_->f = f;
    }
    return boost::make_shared<RegularHessianFactor<D> >(this->keys_, Gs, gs, f);
  }

  // create factor
  boost::shared_ptr<ImplicitSchurFactor<D> > createImplicitSchurFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createImplicitSchurFactor(cameras, point_, lambda);
    else
      return boost::shared_ptr<ImplicitSchurFactor<D> >();
  }

  /// create factor
  boost::shared_ptr<JacobianFactorQ<D> > createJacobianQFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianQFactor(cameras, point_, lambda);
    else
      return boost::make_shared< JacobianFactorQ<D> >(this->keys_);
  }

  /// Create a factor, takes values
  boost::shared_ptr<JacobianFactorQ<D> > createJacobianQFactor(
      const Values& values, double lambda) const {
    Cameras myCameras;
    // TODO triangulate twice ??
    bool nonDegenerate = computeCamerasAndTriangulate(values, myCameras);
    if (nonDegenerate)
      return createJacobianQFactor(myCameras, lambda);
    else
      return boost::make_shared< JacobianFactorQ<D> >(this->keys_);
  }

  /// different (faster) way to compute Jacobian factor
  boost::shared_ptr< JacobianFactor > createJacobianSVDFactor(const Cameras& cameras,
      double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianSVDFactor(cameras, point_, lambda);
    else
      return boost::make_shared< JacobianFactorSVD<D> >(this->keys_);
  }

  /// Returns true if nonDegenerate
  bool computeCamerasAndTriangulate(const Values& values,
      Cameras& myCameras) const {
    Values valuesFactor;

    // Select only the cameras
    BOOST_FOREACH(const Key key, this->keys_)
      valuesFactor.insert(key, values.at(key));

    myCameras = this->cameras(valuesFactor);
    size_t nrCameras = this->triangulateSafe(myCameras);

    if (nrCameras < 2
        || (!this->manageDegeneracy_
            && (this->cheiralityException_ || this->degenerate_)))
      return false;

    // instead, if we want to manage the exception..
    if (this->cheiralityException_ || this->degenerate_) // if we want to manage the exceptions with rotation-only factors
      this->degenerate_ = true;

    if (this->degenerate_) {
      std::cout << "SmartProjectionFactor: this is not ready" << std::endl;
      std::cout << "this->cheiralityException_ " << this->cheiralityException_
          << std::endl;
      std::cout << "this->degenerate_ " << this->degenerate_ << std::endl;
    }
    return true;
  }

  /// Takes values
  bool computeEP(Matrix& E, Matrix& PointCov, const Values& values) const {
    Cameras myCameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, myCameras);
    if (nonDegenerate)
      computeEP(E, PointCov, myCameras);
    return nonDegenerate;
  }

  /// Assumes non-degenerate !
  void computeEP(Matrix& E, Matrix& PointCov, const Cameras& cameras) const {
    return Base::computeEP(E, PointCov, cameras, point_);
  }

  /// Version that takes values, and creates the point
  bool computeJacobians(std::vector<typename Base::KeyMatrix2D>& Fblocks,
      Matrix& E, Matrix& PointCov, Vector& b, const Values& values) const {
    Cameras myCameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, myCameras);
    if (nonDegenerate)
      computeJacobians(Fblocks, E, PointCov, b, myCameras, 0.0);
    return nonDegenerate;
  }

  /// Compute F, E only (called below in both vanilla and SVD versions)
  /// Assumes the point has been computed
  /// Note E can be 2m*3 or 2m*2, in case point is degenerate
  double computeJacobians(std::vector<typename Base::KeyMatrix2D>& Fblocks,
      Matrix& E, Vector& b, const Cameras& cameras) const {

    if (this->degenerate_) {
      std::cout << "manage degeneracy " << manageDegeneracy_ << std::endl;
      std::cout << "point " << point_ << std::endl;
      std::cout
          << "SmartProjectionFactor: Management of degeneracy is disabled - not ready to be used"
          << std::endl;
      if (D > 6) {
        std::cout
            << "Management of degeneracy is not yet ready when one also optimizes for the calibration "
            << std::endl;
      }

      int numKeys = this->keys_.size();
      E = zeros(2 * numKeys, 2);
      b = zero(2 * numKeys);
      double f = 0;
      for (size_t i = 0; i < this->measured_.size(); i++) {
        if (i == 0) { // first pose
          this->point_ = cameras[i].backprojectPointAtInfinity(
              this->measured_.at(i));
          // 3D parametrization of point at infinity: [px py 1]
        }
        Matrix Fi, Ei;
        Vector bi = -(cameras[i].projectPointAtInfinity(this->point_, Fi, Ei)
            - this->measured_.at(i)).vector();

        this->noise_.at(i)->WhitenSystem(Fi, Ei, bi);
        f += bi.squaredNorm();
        Fblocks.push_back(typename Base::KeyMatrix2D(this->keys_[i], Fi));
        E.block < 2, 2 > (2 * i, 0) = Ei;
        subInsert(b, bi, 2 * i);
      }
      return f;
    } else {
      // nondegenerate: just return Base version
      return Base::computeJacobians(Fblocks, E, b, cameras, point_);
    } // end else
  }

  /// Version that computes PointCov, with optional lambda parameter
  double computeJacobians(std::vector<typename Base::KeyMatrix2D>& Fblocks,
      Matrix& E, Matrix& PointCov, Vector& b, const Cameras& cameras,
      const double lambda = 0.0) const {

    double f = computeJacobians(Fblocks, E, b, cameras);

    // Point covariance inv(E'*E)
    PointCov.noalias() = (E.transpose() * E + lambda * eye(E.cols())).inverse();

    return f;
  }

  /// takes values
  bool computeJacobiansSVD(std::vector<typename Base::KeyMatrix2D>& Fblocks,
      Matrix& Enull, Vector& b, const Values& values) const {
    typename Base::Cameras myCameras;
    double good = computeCamerasAndTriangulate(values, myCameras);
    if (good)
      computeJacobiansSVD(Fblocks, Enull, b, myCameras);
    return true;
  }

  /// SVD version
  double computeJacobiansSVD(std::vector<typename Base::KeyMatrix2D>& Fblocks,
      Matrix& Enull, Vector& b, const Cameras& cameras) const {
    return Base::computeJacobiansSVD(Fblocks, Enull, b, cameras, point_);
  }

  /// Returns Matrix, TODO: maybe should not exist -> not sparse !
  // TODO should there be a lambda?
  double computeJacobiansSVD(Matrix& F, Matrix& Enull, Vector& b,
      const Cameras& cameras) const {
    return Base::computeJacobiansSVD(F, Enull, b, cameras, point_);
  }

  /// Returns Matrix, TODO: maybe should not exist -> not sparse !
  double computeJacobians(Matrix& F, Matrix& E, Matrix3& PointCov, Vector& b,
      const Cameras& cameras, const double lambda) const {
    return Base::computeJacobians(F, E, PointCov, b, cameras, point_, lambda);
  }

  /// Calculate vector of re-projection errors, before applying noise model
  /// Assumes triangulation was done and degeneracy handled
  Vector reprojectionError(const Cameras& cameras) const {
    return Base::reprojectionError(cameras, point_);
  }

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionError(const Values& values) const {
    Cameras myCameras;
    bool nonDegenerate = computeCamerasAndTriangulate(values, myCameras);
    if (nonDegenerate)
      return reprojectionError(myCameras);
    else
      return zero(myCameras.size() * 2);
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
      // return 0.0; // TODO: this maybe should be zero?
      std::cout
          << "SmartProjectionHessianFactor: trying to manage degeneracy (this should not happen is manageDegeneracy is disabled)!"
          << std::endl;
      size_t i = 0;
      double overallError = 0;
      BOOST_FOREACH(const Camera& camera, cameras) {
        const Point2& zi = this->measured_.at(i);
        if (i == 0) // first pose
          this->point_ = camera.backprojectPointAtInfinity(zi); // 3D parametrization of point at infinity
        Point2 reprojectionError(
            camera.projectPointAtInfinity(this->point_) - zi);
        overallError += 0.5
            * this->noise_.at(i)->distance(reprojectionError.vector());
        i += 1;
      }
      return overallError;
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

} // \ namespace gtsam
