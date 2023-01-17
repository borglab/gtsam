/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionFactor.h
 * @brief  Smart stereo factor on StereoCameras (pose)
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartFactorBase.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/dllexport.h>

#include <boost/make_shared.hpp>
#include <boost/serialization/optional.hpp>
#include <optional>
#include <vector>

namespace gtsam {

/*
 *  Parameters for the smart stereo projection factors (identical to the SmartProjectionParams)
 */
typedef SmartProjectionParams SmartStereoProjectionParams;

/**
 * SmartStereoProjectionFactor: triangulates point and keeps an estimate of it around.
 * This factor operates with StereoCamera. This factor requires that values
 * contains the involved StereoCameras. Calibration is assumed to be fixed, as this
 * is also assumed in StereoCamera.
 * If you'd like to store poses in values instead of cameras, use
 * SmartStereoProjectionPoseFactor instead
*/
class SmartStereoProjectionFactor
    : public SmartFactorBase<StereoCamera> {
 private:

  typedef SmartFactorBase<StereoCamera> Base;

protected:

  /// @name Parameters
  /// @{
  const SmartStereoProjectionParams params_;
  /// @}

  /// @name Caching triangulation
  /// @{
  mutable TriangulationResult result_; ///< result from triangulateSafe
  mutable std::vector<Pose3> cameraPosesTriangulation_; ///< current triangulation poses
  /// @}

public:

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<SmartStereoProjectionFactor> shared_ptr;

  /// Vector of cameras
  typedef CameraSet<StereoCamera> Cameras;

  /// Vector of monocular cameras (stereo treated as 2 monocular)
  typedef PinholeCamera<Cal3_S2> MonoCamera;
  typedef CameraSet<MonoCamera> MonoCameras;
  typedef MonoCamera::MeasurementVector MonoMeasurements;

  /**
   * Constructor
   * @param params internal parameters of the smart factors
   */
  SmartStereoProjectionFactor(const SharedNoiseModel& sharedNoiseModel,
      const SmartStereoProjectionParams& params = SmartStereoProjectionParams(),
      const std::optional<Pose3> body_P_sensor = {}) :
      Base(sharedNoiseModel, body_P_sensor), //
      params_(params), //
      result_(TriangulationResult::Degenerate()) {
  }

  /** Virtual destructor */
  ~SmartStereoProjectionFactor() override {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "SmartStereoProjectionFactor\n";
    std::cout << "linearizationMode:\n" << params_.linearizationMode << std::endl;
    std::cout << "triangulationParameters:\n" << params_.triangulation << std::endl;
    std::cout << "result:\n" << result_ << std::endl;
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const SmartStereoProjectionFactor *e =
        dynamic_cast<const SmartStereoProjectionFactor*>(&p);
    return e && params_.linearizationMode == e->params_.linearizationMode
        && Base::equals(p, tol);
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
            params_.retriangulationThreshold)) {
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

//  /// triangulateSafe
//  size_t triangulateSafe(const Values& values) const {
//    return triangulateSafe(this->cameras(values));
//  }

  /// triangulateSafe
  TriangulationResult triangulateSafe(const Cameras& cameras) const {

    size_t m = cameras.size();
    bool retriangulate = decideIfTriangulate(cameras);

    // triangulate stereo measurements by treating each stereocamera as a pair of monocular cameras
    MonoCameras monoCameras;
    MonoMeasurements monoMeasured;
    for(size_t i = 0; i < m; i++) {
      const Pose3 leftPose = cameras[i].pose();
      const Cal3_S2 monoCal = cameras[i].calibration().calibration();
      const MonoCamera leftCamera_i(leftPose,monoCal);
      const Pose3 left_Pose_right = Pose3(Rot3(),Point3(cameras[i].baseline(),0.0,0.0));
      const Pose3 rightPose = leftPose.compose( left_Pose_right );
      const MonoCamera rightCamera_i(rightPose,monoCal);
      const StereoPoint2 zi = measured_[i];
      monoCameras.push_back(leftCamera_i);
      monoMeasured.push_back(Point2(zi.uL(),zi.v()));
      if(!std::isnan(zi.uR())){ // if right point is valid
        monoCameras.push_back(rightCamera_i);
        monoMeasured.push_back(Point2(zi.uR(),zi.v()));
      }
    }
    if (retriangulate)
      result_ = gtsam::triangulateSafe(monoCameras, monoMeasured,
          params_.triangulation);
    return result_;
  }

  /// triangulate
  bool triangulateForLinearize(const Cameras& cameras) const {
    triangulateSafe(cameras); // imperative, might reset result_
    return bool(result_);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  std::shared_ptr<RegularHessianFactor<Base::Dim> > createHessianFactor(
      const Cameras& cameras, const double lambda = 0.0,  bool diagonalDamping =
          false) const {

    size_t numKeys = this->keys_.size();
    // Create structures for Hessian Factors
    KeyVector js;
    std::vector<Matrix> Gs(numKeys * (numKeys + 1) / 2);
    std::vector<Vector> gs(numKeys);

    if (this->measured_.size() != cameras.size())
      throw std::runtime_error("SmartStereoProjectionHessianFactor: this->"
                               "measured_.size() inconsistent with input");

    triangulateSafe(cameras);

    if (params_.degeneracyMode == ZERO_ON_DEGENERACY && !result_) {
      // failed: return"empty" Hessian
      for(Matrix& m: Gs)
        m = Matrix::Zero(Base::Dim, Base::Dim);
      for(Vector& v: gs)
        v = Vector::Zero(Base::Dim);
      return std::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
          Gs, gs, 0.0);
    }

    // Jacobian could be 3D Point3 OR 2D Unit3, difference is E.cols().
    Base::FBlocks Fs;
    Matrix F, E;
    Vector b;
    computeJacobiansWithTriangulatedPoint(Fs, E, b, cameras);

    // Whiten using noise model
    Base::whitenJacobians(Fs, E, b);

    // build augmented hessian
    SymmetricBlockMatrix augmentedHessian = //
        Cameras::SchurComplement(Fs, E, b, lambda, diagonalDamping);

    return std::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
        augmentedHessian);
  }

  // create factor
//  std::shared_ptr<RegularImplicitSchurFactor<StereoCamera> > createRegularImplicitSchurFactor(
//      const Cameras& cameras, double lambda) const {
//    if (triangulateForLinearize(cameras))
//      return Base::createRegularImplicitSchurFactor(cameras, *result_, lambda);
//    else
//      // failed: return empty
//      return std::shared_ptr<RegularImplicitSchurFactor<StereoCamera> >();
//  }
//
//  /// create factor
//  std::shared_ptr<JacobianFactorQ<Base::Dim, Base::ZDim> > createJacobianQFactor(
//      const Cameras& cameras, double lambda) const {
//    if (triangulateForLinearize(cameras))
//      return Base::createJacobianQFactor(cameras, *result_, lambda);
//    else
//      // failed: return empty
//      return std::make_shared<JacobianFactorQ<Base::Dim, Base::ZDim> >(this->keys_);
//  }
//
//  /// Create a factor, takes values
//  std::shared_ptr<JacobianFactorQ<Base::Dim, Base::ZDim> > createJacobianQFactor(
//      const Values& values, double lambda) const {
//    return createJacobianQFactor(this->cameras(values), lambda);
//  }

  /// different (faster) way to compute Jacobian factor
  std::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianSVDFactor(cameras, *result_, lambda);
    else
      return std::make_shared<JacobianFactorSVD<Base::Dim, ZDim> >(this->keys_);
  }

//  /// linearize to a Hessianfactor
//  virtual std::shared_ptr<RegularHessianFactor<Base::Dim> > linearizeToHessian(
//      const Values& values, double lambda = 0.0) const {
//    return createHessianFactor(this->cameras(values), lambda);
//  }

//  /// linearize to an Implicit Schur factor
//  virtual std::shared_ptr<RegularImplicitSchurFactor<StereoCamera> > linearizeToImplicit(
//      const Values& values, double lambda = 0.0) const {
//    return createRegularImplicitSchurFactor(this->cameras(values), lambda);
//  }
//
//  /// linearize to a JacobianfactorQ
//  virtual std::shared_ptr<JacobianFactorQ<Base::Dim, Base::ZDim> > linearizeToJacobian(
//      const Values& values, double lambda = 0.0) const {
//    return createJacobianQFactor(this->cameras(values), lambda);
//  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return a Gaussian factor
   */
  std::shared_ptr<GaussianFactor> linearizeDamped(const Cameras& cameras,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    switch (params_.linearizationMode) {
    case HESSIAN:
      return createHessianFactor(cameras, lambda);
//    case IMPLICIT_SCHUR:
//      return createRegularImplicitSchurFactor(cameras, lambda);
    case JACOBIAN_SVD:
      return createJacobianSVDFactor(cameras, lambda);
//    case JACOBIAN_Q:
//      return createJacobianQFactor(cameras, lambda);
    default:
      throw std::runtime_error("SmartStereoFactorlinearize: unknown mode");
    }
  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return a Gaussian factor
   */
  std::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    Cameras cameras = this->cameras(values);
    return linearizeDamped(cameras, lambda);
  }

  /// linearize
  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    return linearizeDamped(values);
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Cameras& cameras) const {
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      cameras.project2(*result_, nullptr, &E);
    return nonDegenerate;
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Values& values) const {
    Cameras cameras = this->cameras(values);
    return triangulateAndComputeE(E, cameras);
  }


  /// Compute F, E only (called below in both vanilla and SVD versions)
  /// Assumes the point has been computed
  /// Note E can be 2m*3 or 2m*2, in case point is degenerate
  void computeJacobiansWithTriangulatedPoint(
      FBlocks& Fs,
      Matrix& E, Vector& b,
      const Cameras& cameras) const {

    if (!result_) {
      throw ("computeJacobiansWithTriangulatedPoint");
//      // Handle degeneracy
//      // TODO check flag whether we should do this
//      Unit3 backProjected; /* = cameras[0].backprojectPointAtInfinity(
//          this->measured_.at(0)); */
//
//      Base::computeJacobians(Fs, E, b, cameras, backProjected);
    } else {
      // valid result: just return Base version
      Base::computeJacobians(Fs, E, b, cameras, *result_);
    }
  }

  /// Version that takes values, and creates the point
  bool triangulateAndComputeJacobians(
      FBlocks& Fs, Matrix& E, Vector& b,
      const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      computeJacobiansWithTriangulatedPoint(Fs, E, b, cameras);
    return nonDegenerate;
  }

  /// takes values
  bool triangulateAndComputeJacobiansSVD(
      FBlocks& Fs, Matrix& Enull, Vector& b,
      const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      Base::computeJacobiansSVD(Fs, Enull, b, cameras, *result_);
    return nonDegenerate;
  }

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionErrorAfterTriangulation(const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      return Base::unwhitenedError(cameras, *result_);
    else
      return Vector::Zero(cameras.size() * Base::ZDim);
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  double totalReprojectionError(const Cameras& cameras,
      std::optional<Point3> externalPoint = {}) const {

    if (externalPoint)
      result_ = TriangulationResult(*externalPoint);
    else
      result_ = triangulateSafe(cameras);

    if (result_)
      // All good, just use version in base class
      return Base::totalReprojectionError(cameras, *result_);
    else if (params_.degeneracyMode == HANDLE_INFINITY) {
      throw(std::runtime_error("Backproject at infinity not implemented for SmartStereo."));
//      // Otherwise, manage the exceptions with rotation-only factors
//      const StereoPoint2& z0 = this->measured_.at(0);
//      Unit3 backprojected; //= cameras.front().backprojectPointAtInfinity(z0);
//
//      return Base::totalReprojectionError(cameras, backprojected);
    } else {
      // if we don't want to manage the exceptions we discard the factor
      return 0.0;
    }
  }

  /// Calculate total reprojection error
  double error(const Values& values) const override {
    if (this->active(values)) {
      return totalReprojectionError(Base::cameras(values));
    } else { // else of active flag
      return 0.0;
    }
  }

  /**
   * This corrects the Jacobians and error vector for the case in which the
   * right 2D measurement in the monocular camera is missing (nan).
   */
  void correctForMissingMeasurements(
      const Cameras& cameras, Vector& ue,
      typename Cameras::FBlocks* Fs = nullptr,
      Matrix* E = nullptr) const override {
    // when using stereo cameras, some of the measurements might be missing:
    for (size_t i = 0; i < cameras.size(); i++) {
      const StereoPoint2& z = measured_.at(i);
      if (std::isnan(z.uR()))  // if the right 2D measurement is invalid
      {
        if (Fs) {  // delete influence of right point on jacobian Fs
          MatrixZD& Fi = Fs->at(i);
          for (size_t ii = 0; ii < Dim; ii++) Fi(1, ii) = 0.0;
        }
        if (E)  // delete influence of right point on jacobian E
          E->row(ZDim * i + 1) = Matrix::Zero(1, E->cols());

        // set the corresponding entry of vector ue to zero
        ue(ZDim * i + 1) = 0.0;
      }
    }
  }

  /** return the landmark */
    TriangulationResult point() const {
      return result_;
    }

    /** COMPUTE the landmark */
    TriangulationResult point(const Values& values) const {
      Cameras cameras = this->cameras(values);
      return triangulateSafe(cameras);
    }

    /// Is result valid?
    bool isValid() const { return result_.valid(); }

    /** return the degenerate state */
    bool isDegenerate() const { return result_.degenerate(); }

    /** return the cheirality status flag */
    bool isPointBehindCamera() const { return result_.behindCamera(); }

    /** return the outlier state */
    bool isOutlier() const { return result_.outlier(); }

    /** return the farPoint state */
    bool isFarPoint() const { return result_.farPoint(); }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(params_.throwCheirality);
    ar & BOOST_SERIALIZATION_NVP(params_.verboseCheirality);
  }
};

/// traits
template<>
struct traits<SmartStereoProjectionFactor > : public Testable<
    SmartStereoProjectionFactor> {
};

} // \ namespace gtsam
