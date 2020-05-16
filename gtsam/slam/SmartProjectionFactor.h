/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionFactor.h
 * @brief  Smart factor on cameras (pose + calibration)
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/SmartFactorBase.h>
#include <gtsam/slam/SmartFactorParams.h>

#include <gtsam/geometry/triangulation.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/dataset.h>

#include <boost/optional.hpp>
#include <boost/make_shared.hpp>
#include <vector>

namespace gtsam {

/**
 * SmartProjectionFactor: triangulates point and keeps an estimate of it around.
 * This factor operates with monocular cameras, where a camera is expected to
 * behave like PinholeCamera or PinholePose. This factor is intended
 * to be used directly with PinholeCamera, which optimizes the camera pose
 * and calibration. This also requires that values contains the involved
 * cameras (instead of poses and calibrations separately).
 * If the calibration is fixed use SmartProjectionPoseFactor instead!
 */
template<class CAMERA>
class SmartProjectionFactor: public SmartFactorBase<CAMERA> {

public:

private:
  typedef SmartFactorBase<CAMERA> Base;
  typedef SmartProjectionFactor<CAMERA> This;
  typedef SmartProjectionFactor<CAMERA> SmartProjectionCameraFactor;

protected:

  /// @name Parameters
  /// @{
  SmartProjectionParams params_;
  /// @}

  /// @name Caching triangulation
  /// @{
  mutable TriangulationResult result_; ///< result from triangulateSafe
  mutable std::vector<Pose3, Eigen::aligned_allocator<Pose3> > cameraPosesTriangulation_; ///< current triangulation poses
  /// @}

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// shorthand for a set of cameras
  typedef CameraSet<CAMERA> Cameras;

  /**
   * Default constructor, only for serialization
   */
  SmartProjectionFactor() {}

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param params parameters for the smart projection factors
   */
  SmartProjectionFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel),
        params_(params),
        result_(TriangulationResult::Degenerate()) {}

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
    std::cout << s << "SmartProjectionFactor\n";
    std::cout << "linearizationMode:\n" << params_.linearizationMode
        << std::endl;
    std::cout << "triangulationParameters:\n" << params_.triangulation
        << std::endl;
    std::cout << "result:\n" << result_ << std::endl;
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && params_.linearizationMode == e->params_.linearizationMode
        && Base::equals(p, tol);
  }

  /// Check if the new linearization point is the same as the one used for previous triangulation
  bool decideIfTriangulate(const Cameras& cameras) const {
    // several calls to linearize will be done from the same linearization point, hence it is not needed to re-triangulate
    // Note that this is not yet "selecting linearization", that will come later, and we only check if the
    // current linearization is the "same" (up to tolerance) w.r.t. the last time we triangulated the point

    size_t m = cameras.size();

    bool retriangulate = false;

    // if we do not have a previous linearization point or the new linearization point includes more poses
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

    return retriangulate; // if we arrive to this point all poses are the same and we don't need re-triangulation
  }

  /// triangulateSafe
  TriangulationResult triangulateSafe(const Cameras& cameras) const {

    size_t m = cameras.size();
    if (m < 2) // if we have a single pose the corresponding factor is uninformative
      return TriangulationResult::Degenerate();

    bool retriangulate = decideIfTriangulate(cameras);
    if (retriangulate)
      result_ = gtsam::triangulateSafe(cameras, this->measured_,
          params_.triangulation);
    return result_;
  }

  /// triangulate
  bool triangulateForLinearize(const Cameras& cameras) const {
    triangulateSafe(cameras); // imperative, might reset result_
    return bool(result_);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<Base::Dim> > createHessianFactor(
      const Cameras& cameras, const double lambda = 0.0, bool diagonalDamping =
          false) const {

    size_t numKeys = this->keys_.size();
    // Create structures for Hessian Factors
    KeyVector js;
    std::vector<Matrix> Gs(numKeys * (numKeys + 1) / 2);
    std::vector<Vector> gs(numKeys);

    if (this->measured_.size() != cameras.size())
      throw std::runtime_error("SmartProjectionHessianFactor: this->measured_"
                               ".size() inconsistent with input");

    triangulateSafe(cameras);

    if (params_.degeneracyMode == ZERO_ON_DEGENERACY && !result_) {
      // failed: return"empty" Hessian
      for(Matrix& m: Gs)
        m = Matrix::Zero(Base::Dim, Base::Dim);
      for(Vector& v: gs)
        v = Vector::Zero(Base::Dim);
      return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
          Gs, gs, 0.0);
    }

    // Jacobian could be 3D Point3 OR 2D Unit3, difference is E.cols().
    std::vector<typename Base::MatrixZD, Eigen::aligned_allocator<typename Base::MatrixZD> > Fblocks;
    Matrix E;
    Vector b;
    computeJacobiansWithTriangulatedPoint(Fblocks, E, b, cameras);

    // Whiten using noise model
    Base::whitenJacobians(Fblocks, E, b);

    // build augmented hessian
    SymmetricBlockMatrix augmentedHessian = //
        Cameras::SchurComplement(Fblocks, E, b, lambda, diagonalDamping);

    return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
        augmentedHessian);
  }

  // create factor
  boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> > createRegularImplicitSchurFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createRegularImplicitSchurFactor(cameras, *result_, lambda);
    else
      // failed: return empty
      return boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> >();
  }

  /// create factor
  boost::shared_ptr<JacobianFactorQ<Base::Dim, 2> > createJacobianQFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianQFactor(cameras, *result_, lambda);
    else
      // failed: return empty
      return boost::make_shared<JacobianFactorQ<Base::Dim, 2> >(this->keys_);
  }

  /// Create a factor, takes values
  boost::shared_ptr<JacobianFactorQ<Base::Dim, 2> > createJacobianQFactor(
      const Values& values, double lambda) const {
    return createJacobianQFactor(this->cameras(values), lambda);
  }

  /// different (faster) way to compute Jacobian factor
  boost::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianSVDFactor(cameras, *result_, lambda);
    else
      // failed: return empty
      return boost::make_shared<JacobianFactorSVD<Base::Dim, 2> >(this->keys_);
  }

  /// linearize to a Hessianfactor
  virtual boost::shared_ptr<RegularHessianFactor<Base::Dim> > linearizeToHessian(
      const Values& values, double lambda = 0.0) const {
    return createHessianFactor(this->cameras(values), lambda);
  }

  /// linearize to an Implicit Schur factor
  virtual boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> > linearizeToImplicit(
      const Values& values, double lambda = 0.0) const {
    return createRegularImplicitSchurFactor(this->cameras(values), lambda);
  }

  /// linearize to a JacobianfactorQ
  virtual boost::shared_ptr<JacobianFactorQ<Base::Dim, 2> > linearizeToJacobian(
      const Values& values, double lambda = 0.0) const {
    return createJacobianQFactor(this->cameras(values), lambda);
  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return a Gaussian factor
   */
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Cameras& cameras,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    switch (params_.linearizationMode) {
    case HESSIAN:
      return createHessianFactor(cameras, lambda);
    case IMPLICIT_SCHUR:
      return createRegularImplicitSchurFactor(cameras, lambda);
    case JACOBIAN_SVD:
      return createJacobianSVDFactor(cameras, lambda);
    case JACOBIAN_Q:
      return createJacobianQFactor(cameras, lambda);
    default:
      throw std::runtime_error("SmartFactorlinearize: unknown mode");
    }
  }

  /**
   * Linearize to Gaussian Factor
   * @param values Values structure which must contain camera poses for this factor
   * @return a Gaussian factor
   */
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    Cameras cameras = this->cameras(values);
    return linearizeDamped(cameras, lambda);
  }

  /// linearize
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    return linearizeDamped(values);
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Cameras& cameras) const {
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      cameras.project2(*result_, boost::none, E);
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
      std::vector<typename Base::MatrixZD, Eigen::aligned_allocator<typename Base::MatrixZD> >& Fblocks, Matrix& E, Vector& b,
      const Cameras& cameras) const {

    if (!result_) {
      // Handle degeneracy
      // TODO check flag whether we should do this
      Unit3 backProjected = cameras[0].backprojectPointAtInfinity(
          this->measured_.at(0));
      Base::computeJacobians(Fblocks, E, b, cameras, backProjected);
    } else {
      // valid result: just return Base version
      Base::computeJacobians(Fblocks, E, b, cameras, *result_);
    }
  }

  /// Version that takes values, and creates the point
  bool triangulateAndComputeJacobians(
      std::vector<typename Base::MatrixZD, Eigen::aligned_allocator<typename Base::MatrixZD> >& Fblocks, Matrix& E, Vector& b,
      const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      computeJacobiansWithTriangulatedPoint(Fblocks, E, b, cameras);
    return nonDegenerate;
  }

  /// takes values
  bool triangulateAndComputeJacobiansSVD(
      std::vector<typename Base::MatrixZD, Eigen::aligned_allocator<typename Base::MatrixZD> >& Fblocks, Matrix& Enull, Vector& b,
      const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      Base::computeJacobiansSVD(Fblocks, Enull, b, cameras, *result_);
    return nonDegenerate;
  }

  /// Calculate vector of re-projection errors, before applying noise model
  Vector reprojectionErrorAfterTriangulation(const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      return Base::unwhitenedError(cameras, *result_);
    else
      return Vector::Zero(cameras.size() * 2);
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   */
  double totalReprojectionError(const Cameras& cameras,
      boost::optional<Point3> externalPoint = boost::none) const {

    if (externalPoint)
      result_ = TriangulationResult(*externalPoint);
    else
      result_ = triangulateSafe(cameras);

    if (result_)
      // All good, just use version in base class
      return Base::totalReprojectionError(cameras, *result_);
    else if (params_.degeneracyMode == HANDLE_INFINITY) {
      // Otherwise, manage the exceptions with rotation-only factors
      Unit3 backprojected = cameras.front().backprojectPointAtInfinity(
          this->measured_.at(0));
      return Base::totalReprojectionError(cameras, backprojected);
    } else
      // if we don't want to manage the exceptions we discard the factor
      return 0.0;
  }

  /// Calculate total reprojection error
  virtual double error(const Values& values) const {
    if (this->active(values)) {
      return totalReprojectionError(Base::cameras(values));
    } else { // else of active flag
      return 0.0;
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

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// @name Deprecated
  /// @{
  // It does not make sense to optimize for a camera where the pose would not be
  // the actual pose of the camera. An unfortunate consequence of deprecating
  // this constructor means that we cannot optimize for calibration when the
  // camera is offset from the body pose. That would need a new factor with
  // (body) pose and calibration as variables. However, that use case is
  // unlikely: when a global offset is know, calibration is typically known.
  SmartProjectionFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const boost::optional<Pose3> body_P_sensor,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel, body_P_sensor),
        params_(params),
        result_(TriangulationResult::Degenerate()) {}
  /// @}
#endif

 private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(params_);
    ar & BOOST_SERIALIZATION_NVP(result_);
    ar & BOOST_SERIALIZATION_NVP(cameraPosesTriangulation_);
  }
}
;

/// traits
template<class CAMERA>
struct traits<SmartProjectionFactor<CAMERA> > : public Testable<
    SmartProjectionFactor<CAMERA> > {
};

} // \ namespace gtsam
