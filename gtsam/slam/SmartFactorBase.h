/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartFactorBase.h
 * @brief  Base class to create smart factors on poses or cameras
 * @author Luca Carlone
 * @author Antoni Rosinol
 * @author Zsolt Kira
 * @author Frank Dellaert
 * @author Chris Beall
 */

#pragma once

#include <gtsam/slam/JacobianFactorQ.h>
#include <gtsam/slam/JacobianFactorSVD.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/RegularHessianFactor.h>
#include <gtsam/geometry/CameraSet.h>

#include <optional>
#include <boost/serialization/optional.hpp>
#include <vector>

namespace gtsam {

/**
 * @brief  Base class for smart factors.
 * This base class has no internal point, but it has a measurement, noise model
 * and an optional sensor pose.
 * This class mainly computes the derivatives and returns them as a variety of
 * factors. The methods take a CameraSet<CAMERA> argument and the value of a
 * point, which is kept in the derived class.
 *
 * @tparam CAMERA should behave like a PinholeCamera.
 */
template<class CAMERA>
class SmartFactorBase: public NonlinearFactor {

private:
  typedef NonlinearFactor Base;
  typedef SmartFactorBase<CAMERA> This;
  typedef typename CAMERA::Measurement Z;
  typedef typename CAMERA::MeasurementVector ZVector;

public:

  static const int Dim = traits<CAMERA>::dimension; ///< Camera dimension
  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension
  typedef Eigen::Matrix<double, ZDim, Dim> MatrixZD; // F blocks (derivatives wrpt camera)
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks; // vector of F blocks

protected:
  /**
   * As of Feb 22, 2015, the noise model is the same for all measurements and
   * is isotropic. This allows for moving most calculations of Schur complement
   * etc. to be easily moved to CameraSet, and also agrees pragmatically
   * with what is normally done.
   */
  SharedIsotropic noiseModel_;

  /**
   * Measurements for each of the m views.
   * We keep a copy of the measurements for I/O and computing the error.
   * The order is kept the same as the keys that we use to create the factor.
   */
  ZVector measured_;

  std::optional<Pose3>
      body_P_sensor_;  ///< Pose of the camera in the body frame

  // Cache for Fblocks, to avoid a malloc ever time we re-linearize
  mutable FBlocks Fs;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for a smart pointer to a factor.
  typedef std::shared_ptr<This> shared_ptr;

  /// The CameraSet data structure is used to refer to a set of cameras.
  typedef CameraSet<CAMERA> Cameras;

  /// Default Constructor, for serialization.
  SmartFactorBase() {}

  /// Construct with given noise model and optional arguments.
  SmartFactorBase(const SharedNoiseModel& sharedNoiseModel,
                  std::optional<Pose3> body_P_sensor = {},
                  size_t expectedNumberCameras = 10)
      : body_P_sensor_(body_P_sensor), Fs(expectedNumberCameras) {

    if (!sharedNoiseModel)
      throw std::runtime_error("SmartFactorBase: sharedNoiseModel is required");

    SharedIsotropic sharedIsotropic = std::dynamic_pointer_cast<
        noiseModel::Isotropic>(sharedNoiseModel);

    if (!sharedIsotropic)
      throw std::runtime_error("SmartFactorBase: needs isotropic");

    noiseModel_ = sharedIsotropic;
  }

  /// Virtual destructor, subclasses from NonlinearFactor.
  ~SmartFactorBase() override {
  }

  /**
   * Add a new measurement and pose/camera key.
   * @param measured is the 2m dimensional projection of a single landmark
   * @param key is the index corresponding to the camera observing the landmark
   */
  void add(const Z& measured, const Key& key) {
    if(std::find(keys_.begin(), keys_.end(), key) != keys_.end()) {
      throw std::invalid_argument(
          "SmartFactorBase::add: adding duplicate measurement for key.");
    }
    this->measured_.push_back(measured);
    this->keys_.push_back(key);
  }

  /// Add a bunch of measurements, together with the camera keys.
  void add(const ZVector& measurements, const KeyVector& cameraKeys) {
    assert(measurements.size() == cameraKeys.size());
    for (size_t i = 0; i < measurements.size(); i++) {
      this->add(measurements[i], cameraKeys[i]);
    }
  }

  /**
   * Add an entire SfM_track (collection of cameras observing a single point).
   * The noise is assumed to be the same for all measurements.
   */
  template<class SFM_TRACK>
  void add(const SFM_TRACK& trackToAdd) {
    for (size_t k = 0; k < trackToAdd.numberMeasurements(); k++) {
      this->measured_.push_back(trackToAdd.measurements[k].second);
      this->keys_.push_back(trackToAdd.measurements[k].first);
    }
  }

  /// Return the dimension (number of rows!) of the factor.
  size_t dim() const override { return ZDim * this->measured_.size(); }

  /// Return the 2D measurements (ZDim, in general).
  const ZVector& measured() const { return measured_; }

  /// Collect all cameras: important that in key order.
  virtual Cameras cameras(const Values& values) const {
    Cameras cameras;
    for(const Key& k: this->keys_)
      cameras.push_back(values.at<CAMERA>(k));
    return cameras;
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "SmartFactorBase, z = \n";
    for (size_t k = 0; k < measured_.size(); ++k) {
      std::cout << "measurement " << k<<", px = \n" << measured_[k] << "\n";
      noiseModel_->print("noise model = ");
    }
    if(body_P_sensor_)
      body_P_sensor_->print("body_P_sensor_:\n");
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    if (const This* e = dynamic_cast<const This*>(&p)) {
      // Check that all measurements are the same.
      for (size_t i = 0; i < measured_.size(); i++) {
        if (!traits<Z>::Equals(this->measured_.at(i), e->measured_.at(i), tol))
          return false;
      }
      // If so, check base class.
      return Base::equals(p, tol);
    } else {
      return false;
    }
  }

  /** Compute reprojection errors [h(x)-z] = [cameras.project(p)-z] and
  * derivatives. This is the error before the noise model is applied.
  * The templated version described above must finally get resolved to this
  * function.
  */
  template <class POINT>
  Vector unwhitenedError(
      const Cameras& cameras, const POINT& point,
      typename Cameras::FBlocks* Fs = nullptr,  //
      Matrix* E = nullptr) const {
    // Reproject, with optional derivatives.
    Vector error = cameras.reprojectionError(point, measured_, Fs, E);

    // Apply chain rule if body_P_sensor_ is given.
    if (body_P_sensor_ && Fs) {
      const Pose3 sensor_P_body = body_P_sensor_->inverse();
      constexpr int camera_dim = traits<CAMERA>::dimension;
      constexpr int pose_dim = traits<Pose3>::dimension;

      for (size_t i = 0; i < Fs->size(); i++) {
        const Pose3 world_P_body = cameras[i].pose() * sensor_P_body;
        Eigen::Matrix<double, camera_dim, camera_dim> J;
        J.setZero();
        Eigen::Matrix<double, pose_dim, pose_dim> H;
        // Call compose to compute Jacobian for camera extrinsics
        world_P_body.compose(*body_P_sensor_, H);
        // Assign extrinsics part of the Jacobian
        J.template block<pose_dim, pose_dim>(0, 0) = H;
        Fs->at(i) = Fs->at(i) * J;
      }
    }

    // Correct the Jacobians in case some measurements are missing. 
    correctForMissingMeasurements(cameras, error, Fs, E);

    return error;
  }

  /** 
   * An overload of unwhitenedError. This allows
   * end users to provide optional arguments that are l-value references
   * to the matrices and vectors that will be used to store the results instead
   * of pointers.
   */
  template<class POINT, class ...OptArgs>
  Vector unwhitenedError(
      const Cameras& cameras, const POINT& point,
      OptArgs&&... optArgs) const {
    return unwhitenedError(cameras, point, (&optArgs)...);
  }

  /**
   * This corrects the Jacobians for the case in which some 2D measurement is
   * missing (nan). In practice, this does not do anything in the monocular
   * case, but it is implemented in the stereo version.
   */
  virtual void correctForMissingMeasurements(
      const Cameras& cameras, Vector& ue,
      typename Cameras::FBlocks* Fs = nullptr,
      Matrix* E = nullptr) const {}

  /**
   * An overload of correctForMissingMeasurements. This allows
   * end users to provide optional arguments that are l-value references
   * to the matrices and vectors that will be used to store the results instead
   * of pointers.
   */
  template<class ...OptArgs>
  void correctForMissingMeasurements(
      const Cameras& cameras, Vector& ue,
      OptArgs&&... optArgs) const {
    correctForMissingMeasurements(cameras, ue, (&optArgs)...);
  }

  /**
   * Calculate vector of re-projection errors [h(x)-z] = [cameras.project(p) -
   * z], with the noise model applied.
   */
  template<class POINT>
  Vector whitenedError(const Cameras& cameras, const POINT& point) const {
    Vector error = cameras.reprojectionError(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(error);
    return error;
  }

  /**
   * Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of
   * Gaussian. In this class, we take the raw prediction error \f$ h(x)-z \f$,
   * ask the noise model to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and
   * then multiply by 0.5. Will be used in "error(Values)" function required by
   * NonlinearFactor base class
   */
  template<class POINT>
  double totalReprojectionError(const Cameras& cameras,
      const POINT& point) const {
    Vector error = whitenedError(cameras, point);
    return 0.5 * error.dot(error);
  }

  /// Computes Point Covariance P from the "point Jacobian" E.
  static Matrix PointCov(const Matrix& E) {
    return (E.transpose() * E).inverse();
  }

  /**
   * Compute F, E, and b (called below in both vanilla and SVD versions), where
   * F is a vector of derivatives wrpt the cameras, and E the stacked
   * derivatives with respect to the point. The value of cameras/point are
   * passed as parameters.
   */
  template<class POINT>
  void computeJacobians(FBlocks& Fs, Matrix& E, Vector& b,
      const Cameras& cameras, const POINT& point) const {
    // Project into Camera set and calculate derivatives
    // As in expressionFactor, RHS vector b = - (h(x_bar) - z) = z-h(x_bar)
    // Indeed, nonlinear error |h(x_bar+dx)-z| ~ |h(x_bar) + A*dx - z|
    //                                         = |A*dx - (z-h(x_bar))|
    b = -unwhitenedError(cameras, point, &Fs, &E);
  }

  /**
   * SVD version that produces smaller Jacobian matrices by doing an SVD
   * decomposition on E, and returning the left nulkl-space of E.
   * See JacobianFactorSVD for more documentation.
   * */
  template<class POINT>
  void computeJacobiansSVD(FBlocks& Fs, Matrix& Enull,
      Vector& b, const Cameras& cameras, const POINT& point) const {

    Matrix E;
    computeJacobians(Fs, E, b, cameras, point);

    static const int N = FixedDimension<POINT>::value; // 2 (Unit3) or 3 (Point3)

    // Do SVD on A.
    Eigen::JacobiSVD<Matrix> svd(E, Eigen::ComputeFullU);
    size_t m = this->keys_.size();
    Enull = svd.matrixU().block(0, N, ZDim * m, ZDim * m - N); // last ZDim*m-N columns
  }

  /// Linearize to a Hessianfactor.
  // TODO(dellaert): Not used/tested anywhere and not properly whitened.
  std::shared_ptr<RegularHessianFactor<Dim> > createHessianFactor(
      const Cameras& cameras, const Point3& point, const double lambda = 0.0,
      bool diagonalDamping = false) const {

    Matrix E;
    Vector b;
    computeJacobians(Fs, E, b, cameras, point);

    // build augmented hessian
    SymmetricBlockMatrix augmentedHessian = Cameras::SchurComplement(Fs, E, b);

    return std::make_shared<RegularHessianFactor<Dim> >(keys_,
        augmentedHessian);
  }

  /**
   * Add the contribution of the smart factor to a pre-allocated Hessian,
   * using sparse linear algebra. More efficient than the creation of the
   * Hessian without preallocation of the SymmetricBlockMatrix
   */
  void updateAugmentedHessian(const Cameras& cameras, const Point3& point,
      const double lambda, bool diagonalDamping,
      SymmetricBlockMatrix& augmentedHessian,
      const KeyVector allKeys) const {
    Matrix E;
    Vector b;
    computeJacobians(Fs, E, b, cameras, point);
    Cameras::UpdateSchurComplement(Fs, E, b, allKeys, keys_, augmentedHessian);
  }

  /// Whiten the Jacobians computed by computeJacobians using noiseModel_
  void whitenJacobians(FBlocks& F, Matrix& E, Vector& b) const {
    noiseModel_->WhitenSystem(E, b);
    // TODO make WhitenInPlace work with any dense matrix type
    for (size_t i = 0; i < F.size(); i++)
      F[i] = noiseModel_->Whiten(F[i]);
  }

  /// Return Jacobians as RegularImplicitSchurFactor with raw access
  std::shared_ptr<RegularImplicitSchurFactor<CAMERA> > //
  createRegularImplicitSchurFactor(const Cameras& cameras, const Point3& point,
      double lambda = 0.0, bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    FBlocks F;
    computeJacobians(F, E, b, cameras, point);
    whitenJacobians(F, E, b);
    Matrix P = Cameras::PointCov(E, lambda, diagonalDamping);
    return std::make_shared<RegularImplicitSchurFactor<CAMERA> >(keys_, F, E,
        P, b);
  }

  /// Return Jacobians as JacobianFactorQ.
  std::shared_ptr<JacobianFactorQ<Dim, ZDim> > createJacobianQFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    FBlocks F;
    computeJacobians(F, E, b, cameras, point);
    const size_t M = b.size();
    Matrix P = Cameras::PointCov(E, lambda, diagonalDamping);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M, noiseModel_->sigma());
    return std::make_shared<JacobianFactorQ<Dim, ZDim> >(keys_, F, E, P, b, n);
  }

  /**
   * Return Jacobians as JacobianFactorSVD.
   * TODO(dellaert): lambda is currently ignored
   */
  std::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0) const {
    size_t m = this->keys_.size();
    FBlocks F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);
    computeJacobiansSVD(F, E0, b, cameras, point);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M - 3,
        noiseModel_->sigma());
    return std::make_shared<JacobianFactorSVD<Dim, ZDim> >(keys_, F, E0, b, n);
  }

  /// Create BIG block-diagonal matrix F from Fblocks
  static void FillDiagonalF(const FBlocks& Fs, Matrix& F) {
    size_t m = Fs.size();
    F.resize(ZDim * m, Dim * m);
    F.setZero();
    for (size_t i = 0; i < m; ++i)
      F.block<ZDim, Dim>(ZDim * i, Dim * i) = Fs.at(i);
  }

  // Return sensor pose.
  Pose3 body_P_sensor() const{
    if(body_P_sensor_)
      return *body_P_sensor_;
    else
      return Pose3(); // if unspecified, the transformation is the identity
  }

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION///
/// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
#endif
};
// end class SmartFactorBase

// Definitions need to avoid link errors (above are only declarations)
template<class CAMERA> const int SmartFactorBase<CAMERA>::Dim;
template<class CAMERA> const int SmartFactorBase<CAMERA>::ZDim;

} // \ namespace gtsam
