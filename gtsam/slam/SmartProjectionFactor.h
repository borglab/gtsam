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

#include <gtsam/slam/SmartFactorBase.h>

#include <gtsam/geometry/triangulation.h>
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
struct SmartProjectionFactorState {

  // Hessian representation (after Schur complement)
  bool calculatedHessian;
  Matrix H;
  Vector gs_vector;
  std::vector<Matrix> Gs;
  std::vector<Vector> gs;
  double f;
};

/**
 * SmartProjectionFactor: triangulates point and keeps an estimate of it around.
 */
template<class CAMERA>
class SmartProjectionFactor: public SmartFactorBase<CAMERA> {

public:

  /// Linearization mode: what factor to linearize to
  enum LinearizationMode {
    HESSIAN, IMPLICIT_SCHUR, JACOBIAN_Q, JACOBIAN_SVD
  };

private:
  typedef SmartFactorBase<CAMERA> Base;
  typedef SmartProjectionFactor<CAMERA> This;

protected:

  LinearizationMode linearizeTo_; ///< How to linearize the factor

  /// @name Caching triangulation
  /// @{
  const TriangulationParameters parameters_;
  mutable TriangulationResult result_; ///< result from triangulateSafe

  const double retriangulationThreshold_; ///< threshold to decide whether to re-triangulate
  mutable std::vector<Pose3> cameraPosesTriangulation_; ///< current triangulation poses
  /// @}

  /// @name Parameters governing how triangulation result is treated
  /// @{
  const bool manageDegeneracy_; ///< if set to true will use the rotation-only version for degenerate cases
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)
  /// @}

  /// @name Caching linearization
  /// @{
  /// shorthand for smart projection factor state variable
  typedef boost::shared_ptr<SmartProjectionFactorState> SmartFactorStatePtr;
  SmartFactorStatePtr state_; ///< cached linearization

  const double linearizationThreshold_; ///< threshold to decide whether to re-linearize
  mutable std::vector<Pose3> cameraPosesLinearization_; ///< current linearization poses
  /// @}

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// shorthand for a set of cameras
  typedef CameraSet<CAMERA> Cameras;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param linThreshold threshold on relative pose changes used to decide whether to relinearize (selective relinearization)
   * @param manageDegeneracy is true, in presence of degenerate triangulation, the factor is converted to a rotation-only constraint,
   * otherwise the factor is simply neglected
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   */
  SmartProjectionFactor(LinearizationMode linearizationMode = HESSIAN,
      double rankTolerance = 1, double linThreshold = -1,
      bool manageDegeneracy = false, bool enableEPI = false,
      double landmarkDistanceThreshold = 1e10,
      double dynamicOutlierRejectionThreshold = -1, SmartFactorStatePtr state =
          SmartFactorStatePtr(new SmartProjectionFactorState())) :
      linearizeTo_(linearizationMode), parameters_(rankTolerance, enableEPI,
          landmarkDistanceThreshold, dynamicOutlierRejectionThreshold), //
      result_(TriangulationResult::Degenerate()), //
      retriangulationThreshold_(1e-5), manageDegeneracy_(manageDegeneracy), //
      throwCheirality_(false), verboseCheirality_(false), //
      state_(state), linearizationThreshold_(linThreshold) {
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
    std::cout << s << "SmartProjectionFactor\n";
    std::cout << "triangulationParameters:\n" << parameters_ << std::endl;
    std::cout << "result:\n" << result_ << std::endl;
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol);
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

    return retriangulate; // if we arrive to this point all poses are the same and we don't need re-triangulation
  }

  /// This function checks if the new linearization point is 'close'  to the previous one used for linearization
  bool decideIfLinearize(const Cameras& cameras) const {
    // "selective linearization"
    // The function evaluates how close are the old and the new poses, transformed in the ref frame of the first pose
    // (we only care about the "rigidity" of the poses, not about their absolute pose)

    if (linearizationThreshold_ < 0) //by convention if linearizationThreshold is negative we always relinearize
      return true;

    // if we do not have a previous linearization point or the new linearization point includes more poses
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
      if (!localCameraPose.equals(localCameraPoseOld, linearizationThreshold_))
        return true; // at least two "relative" poses are different, hence we re-linearize
    }
    return false; // if we arrive to this point all poses are the same and we don't need re-linearize
  }

  /// triangulateSafe
  TriangulationResult triangulateSafe(const Cameras& cameras) const {

    size_t m = cameras.size();
    if (m < 2) // if we have a single pose the corresponding factor is uninformative
      return TriangulationResult::Degenerate();

    bool retriangulate = decideIfTriangulate(cameras);
    if (retriangulate)
      result_ = gtsam::triangulateSafe(cameras, this->measured_, parameters_);
    return result_;
  }

  /// triangulate
  bool triangulateForLinearize(const Cameras& cameras) const {
    triangulateSafe(cameras); // imperative, might reset result_
    return (manageDegeneracy_ || result_);
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<Base::Dim> > createHessianFactor(
      const Cameras& cameras, const double lambda = 0.0) const {

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

    if (!manageDegeneracy_ && !result_) {
      // put in "empty" Hessian
      BOOST_FOREACH(Matrix& m, Gs)
        m = zeros(Base::Dim, Base::Dim);
      BOOST_FOREACH(Vector& v, gs)
        v = zero(Base::Dim);
      return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
          Gs, gs, 0.0);
    }

    // decide whether to re-linearize
    bool doLinearize = this->decideIfLinearize(cameras);

    if (linearizationThreshold_ >= 0 && doLinearize) // if we apply selective relinearization and we need to relinearize
      for (size_t i = 0; i < cameras.size(); i++)
        this->cameraPosesLinearization_[i] = cameras[i].pose();

    if (!doLinearize) { // return the previous Hessian factor
      std::cout << "=============================" << std::endl;
      std::cout << "doLinearize " << doLinearize << std::endl;
      std::cout << "linearizationThreshold_ " << linearizationThreshold_
          << std::endl;
      std::cout << "valid: " << isValid() << std::endl;
      std::cout
          << "something wrong in SmartProjectionHessianFactor: selective relinearization should be disabled"
          << std::endl;
      exit(1);
      return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_,
          this->state_->Gs, this->state_->gs, this->state_->f);
    }

    // ==================================================================
    Matrix F, E;
    Vector b;
    {
      std::vector<typename Base::MatrixZD> Fblocks;
      computeJacobiansWithTriangulatedPoint(Fblocks, E, b, cameras);
      Base::whitenJacobians(Fblocks, E, b);
      Base::FillDiagonalF(Fblocks, F); // expensive !
    }
    double f = b.squaredNorm();

    // Schur complement trick
    // Frank says: should be possible to do this more efficiently?
    // And we care, as in grouped factors this is called repeatedly
    Matrix H(Base::Dim * numKeys, Base::Dim * numKeys);
    Vector gs_vector;

    // Note P can 2*2 or 3*3
    Matrix P = Base::PointCov(E, lambda);

    // Create reduced Hessian matrix via Schur complement F'*F - F'*E*P*E'*F
    H.noalias() = F.transpose() * (F - (E * (P * (E.transpose() * F))));

    // Create reduced gradient - (F'*b - F'*E*P*E'*b)
    // Note the minus sign above! g has negative b.
    // Informal reasoning: when we write the error as 0.5*|Ax-b|^2
    // the derivative is A'*(Ax-b), and at x=0, this becomes -A'*b
    gs_vector.noalias() = -F.transpose()
        * (b - (E * (P * (E.transpose() * b))));

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
    if (linearizationThreshold_ >= 0) { // if we do not use selective relinearization we don't need to store these variables
      this->state_->Gs = Gs;
      this->state_->gs = gs;
      this->state_->f = f;
    }
    return boost::make_shared<RegularHessianFactor<Base::Dim> >(this->keys_, Gs,
        gs, f);
  }

  // create factor
  boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> > createRegularImplicitSchurFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createRegularImplicitSchurFactor(cameras, *result_, lambda);
    else
      return boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> >();
  }

  /// create factor
  boost::shared_ptr<JacobianFactorQ<Base::Dim, 2> > createJacobianQFactor(
      const Cameras& cameras, double lambda) const {
    if (triangulateForLinearize(cameras))
      return Base::createJacobianQFactor(cameras, *result_, lambda);
    else
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
  boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
      const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    Cameras cameras = this->cameras(values);
    switch (linearizeTo_) {
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

  /// linearize
  virtual boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const {
    return linearizeDamped(values);
  }

  /**
   * Triangulate and compute derivative of error with respect to point
   * @return whether triangulation worked
   */
  bool triangulateAndComputeE(Matrix& E, const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      cameras.project2(*result_, boost::none, E);
    return nonDegenerate;
  }

  /// Compute F, E only (called below in both vanilla and SVD versions)
  /// Assumes the point has been computed
  /// Note E can be 2m*3 or 2m*2, in case point is degenerate
  void computeJacobiansWithTriangulatedPoint(
      std::vector<typename Base::MatrixZD>& Fblocks, Matrix& E, Vector& b,
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
      std::vector<typename Base::MatrixZD>& Fblocks, Matrix& E, Vector& b,
      const Values& values) const {
    Cameras cameras = this->cameras(values);
    bool nonDegenerate = triangulateForLinearize(cameras);
    if (nonDegenerate)
      computeJacobiansWithTriangulatedPoint(Fblocks, E, b, cameras);
    return nonDegenerate;
  }

  /// takes values
  bool triangulateAndComputeJacobiansSVD(
      std::vector<typename Base::MatrixZD>& Fblocks, Matrix& Enull, Vector& b,
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

    if (externalPoint)
      result_ = TriangulationResult(*externalPoint);
    else
      result_ = triangulateSafe(cameras);

    if (result_)
      // All good, just use version in base class
      return Base::totalReprojectionError(cameras, *result_);
    else if (manageDegeneracy_) {
      // Otherwise, manage the exceptions with rotation-only factors
      const Point2& z0 = this->measured_.at(0);
      Unit3 backprojected = cameras.front().backprojectPointAtInfinity(z0);
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
  inline bool isValid() const {
    return result_;
  }

  /** return the degenerate state */
  inline bool isDegenerate() const {
    return result_.degenerate();
  }

  /** return the cheirality status flag */
  inline bool isPointBehindCamera() const {
    return result_.behindCamera();
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

/// traits
template<class CAMERA>
struct traits<SmartProjectionFactor<CAMERA> > : public Testable<
    SmartProjectionFactor<CAMERA> > {
};

} // \ namespace gtsam
