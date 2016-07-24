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

#include <boost/optional.hpp>
#include <boost/serialization/optional.hpp>
#include <boost/make_shared.hpp>
#include <vector>

namespace gtsam {

/**
 * @brief  Base class for smart factors
 * This base class has no internal point, but it has a measurement, noise model
 * and an optional sensor pose.
 * This class mainly computes the derivatives and returns them as a variety of factors.
 * The methods take a Cameras argument, which should behave like PinholeCamera, and
 * the value of a point, which is kept in the base class.
 */
template<class CAMERA>
class SmartFactorBase: public NonlinearFactor {

private:
  typedef NonlinearFactor Base;
  typedef SmartFactorBase<CAMERA> This;
  typedef typename CAMERA::Measurement Z;

public:

  static const int Dim = traits<CAMERA>::dimension; ///< Camera dimension
  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension
  typedef Eigen::Matrix<double, ZDim, Dim> MatrixZD; // F blocks (derivatives wrpt camera)

protected:
  /**
   * As of Feb 22, 2015, the noise model is the same for all measurements and
   * is isotropic. This allows for moving most calculations of Schur complement
   * etc to be moved to CameraSet very easily, and also agrees pragmatically
   * with what is normally done.
   */
  SharedIsotropic noiseModel_;

  /**
   * 2D measurement and noise model for each of the m views
   * We keep a copy of measurements for I/O and computing the error.
   * The order is kept the same as the keys that we use to create the factor.
   */
  std::vector<Z> measured_;

  /// @name Pose of the camera in the body frame
  boost::optional<Pose3> body_P_sensor_; ///< Pose of the camera in the body frame
  /// @}

  // Cache for Fblocks, to avoid a malloc ever time we re-linearize
  mutable std::vector<MatrixZD> Fblocks;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// We use the new CameraSte data structure to refer to a set of cameras
  typedef CameraSet<CAMERA> Cameras;

  /// Default Constructor, for serialization
  SmartFactorBase() {}

  /// Constructor
  SmartFactorBase(const SharedNoiseModel& sharedNoiseModel,
                  boost::optional<Pose3> body_P_sensor = boost::none,
                  size_t expectedNumberCameras = 10)
      : body_P_sensor_(body_P_sensor), Fblocks(expectedNumberCameras) {

    if (!sharedNoiseModel)
      throw std::runtime_error("SmartFactorBase: sharedNoiseModel is required");

    SharedIsotropic sharedIsotropic = boost::dynamic_pointer_cast<
        noiseModel::Isotropic>(sharedNoiseModel);

    if (!sharedIsotropic)
      throw std::runtime_error("SmartFactorBase: needs isotropic");

    noiseModel_ = sharedIsotropic;
  }

  /// Virtual destructor, subclasses from NonlinearFactor
  virtual ~SmartFactorBase() {
  }

  /**
   * Add a new measurement and pose key
   * @param measured_i is the 2m dimensional projection of a single landmark
   * @param poseKey is the index corresponding to the camera observing the landmark
   * @param sharedNoiseModel is the measurement noise
   */
  void add(const Z& measured_i, const Key& cameraKey_i) {
    this->measured_.push_back(measured_i);
    this->keys_.push_back(cameraKey_i);
  }

  /**
   * Add a bunch of measurements, together with the camera keys
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& cameraKeys) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(cameraKeys.at(i));
    }
  }

  /**
   * Adds an entire SfM_track (collection of cameras observing a single point).
   * The noise is assumed to be the same for all measurements
   */
  template<class SFM_TRACK>
  void add(const SFM_TRACK& trackToAdd) {
    for (size_t k = 0; k < trackToAdd.number_measurements(); k++) {
      this->measured_.push_back(trackToAdd.measurements[k].second);
      this->keys_.push_back(trackToAdd.measurements[k].first);
    }
  }

  /// get the dimension (number of rows!) of the factor
  virtual size_t dim() const {
    return ZDim * this->measured_.size();
  }

  /** return the measurements */
  const std::vector<Z>& measured() const {
    return measured_;
  }

  /// Collect all cameras: important that in key order
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
      DefaultKeyFormatter) const {
    std::cout << s << "SmartFactorBase, z = \n";
    for (size_t k = 0; k < measured_.size(); ++k) {
      std::cout << "measurement, p = " << measured_[k] << "\t";
      noiseModel_->print("noise model = ");
    }
    if(body_P_sensor_)
      body_P_sensor_->print("body_P_sensor_:\n");
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);

    bool areMeasurementsEqual = true;
    for (size_t i = 0; i < measured_.size(); i++) {
      if (traits<Z>::Equals(this->measured_.at(i), e->measured_.at(i), tol) == false)
        areMeasurementsEqual = false;
      break;
    }
    return e && Base::equals(p, tol) && areMeasurementsEqual;
  }

  /// Compute reprojection errors [h(x)-z] = [cameras.project(p)-z] and derivatives
  template<class POINT>
  Vector unwhitenedError(const Cameras& cameras, const POINT& point,
      boost::optional<typename Cameras::FBlocks&> Fs = boost::none, //
      boost::optional<Matrix&> E = boost::none) const {
    Vector ue = cameras.reprojectionError(point, measured_, Fs, E);
    if(body_P_sensor_){
      for(size_t i=0; i < Fs->size(); i++){
        Pose3 w_Pose_body = (cameras[i].pose()).compose(body_P_sensor_->inverse());
        Matrix J(6, 6);
        Pose3 world_P_body = w_Pose_body.compose(*body_P_sensor_, J);
        Fs->at(i) = Fs->at(i) * J;
      }
    }
    return ue;
  }

  /**
   * Calculate vector of re-projection errors [h(x)-z] = [cameras.project(p) - z]
   * Noise model applied
   */
  template<class POINT>
  Vector whitenedError(const Cameras& cameras, const POINT& point) const {
    Vector e = cameras.reprojectionError(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(e);
    return e;
  }

  /** Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   * Will be used in "error(Values)" function required by NonlinearFactor base class
   */
  template<class POINT>
  double totalReprojectionError(const Cameras& cameras,
      const POINT& point) const {
    Vector e = whitenedError(cameras, point);
    return 0.5 * e.dot(e);
  }

  /// Computes Point Covariance P from E
  static Matrix PointCov(Matrix& E) {
    return (E.transpose() * E).inverse();
  }

  /**
   *  Compute F, E, and b (called below in both vanilla and SVD versions), where
   *  F is a vector of derivatives wrpt the cameras, and E the stacked derivatives
   *  with respect to the point. The value of cameras/point are passed as parameters.
   *  TODO: Kill this obsolete method
   */
  template<class POINT>
  void computeJacobians(std::vector<MatrixZD>& Fblocks, Matrix& E, Vector& b,
      const Cameras& cameras, const POINT& point) const {
    // Project into Camera set and calculate derivatives
    // As in expressionFactor, RHS vector b = - (h(x_bar) - z) = z-h(x_bar)
    // Indeed, nonlinear error |h(x_bar+dx)-z| ~ |h(x_bar) + A*dx - z|
    //                                         = |A*dx - (z-h(x_bar))|
    b = -unwhitenedError(cameras, point, Fblocks, E);
  }

  /// SVD version
  template<class POINT>
  void computeJacobiansSVD(std::vector<MatrixZD>& Fblocks, Matrix& Enull,
      Vector& b, const Cameras& cameras, const POINT& point) const {

    Matrix E;
    computeJacobians(Fblocks, E, b, cameras, point);

    static const int N = FixedDimension<POINT>::value; // 2 (Unit3) or 3 (Point3)

    // Do SVD on A
    Eigen::JacobiSVD<Matrix> svd(E, Eigen::ComputeFullU);
    Vector s = svd.singularValues();
    size_t m = this->keys_.size();
    Enull = svd.matrixU().block(0, N, ZDim * m, ZDim * m - N); // last ZDim*m-N columns
  }

  /// Linearize to a Hessianfactor
  boost::shared_ptr<RegularHessianFactor<Dim> > createHessianFactor(
      const Cameras& cameras, const Point3& point, const double lambda = 0.0,
      bool diagonalDamping = false) const {

    Matrix E;
    Vector b;
    computeJacobians(Fblocks, E, b, cameras, point);

    // build augmented hessian
    SymmetricBlockMatrix augmentedHessian = Cameras::SchurComplement(Fblocks, E, b);

    return boost::make_shared<RegularHessianFactor<Dim> >(keys_,
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
      const FastVector<Key> allKeys) const {
    Matrix E;
    Vector b;
    computeJacobians(Fblocks, E, b, cameras, point);
    Cameras::UpdateSchurComplement(Fblocks, E, b, allKeys, keys_, augmentedHessian);
  }

  /// Whiten the Jacobians computed by computeJacobians using noiseModel_
  void whitenJacobians(std::vector<MatrixZD>& F, Matrix& E, Vector& b) const {
    noiseModel_->WhitenSystem(E, b);
    // TODO make WhitenInPlace work with any dense matrix type
    for (size_t i = 0; i < F.size(); i++)
      F[i] = noiseModel_->Whiten(F[i]);
  }

  /// Return Jacobians as RegularImplicitSchurFactor with raw access
  boost::shared_ptr<RegularImplicitSchurFactor<CAMERA> > //
  createRegularImplicitSchurFactor(const Cameras& cameras, const Point3& point,
      double lambda = 0.0, bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    std::vector<MatrixZD> F;
    computeJacobians(F, E, b, cameras, point);
    whitenJacobians(F, E, b);
    Matrix P = Cameras::PointCov(E, lambda, diagonalDamping);
    return boost::make_shared<RegularImplicitSchurFactor<CAMERA> >(keys_, F, E,
        P, b);
  }

  /**
   * Return Jacobians as JacobianFactorQ
   */
  boost::shared_ptr<JacobianFactorQ<Dim, ZDim> > createJacobianQFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    std::vector<MatrixZD> F;
    computeJacobians(F, E, b, cameras, point);
    const size_t M = b.size();
    Matrix P = Cameras::PointCov(E, lambda, diagonalDamping);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M, noiseModel_->sigma());
    return boost::make_shared<JacobianFactorQ<Dim, ZDim> >(keys_, F, E, P, b, n);
  }

  /**
   * Return Jacobians as JacobianFactorSVD
   * TODO lambda is currently ignored
   */
  boost::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0) const {
    size_t m = this->keys_.size();
    std::vector<MatrixZD> F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);
    computeJacobiansSVD(F, E0, b, cameras, point);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M - 3,
        noiseModel_->sigma());
    return boost::make_shared<JacobianFactorSVD<Dim, ZDim> >(keys_, F, E0, b, n);
  }

  /// Create BIG block-diagonal matrix F from Fblocks
  static void FillDiagonalF(const std::vector<MatrixZD>& Fblocks, Matrix& F) {
    size_t m = Fblocks.size();
    F.resize(ZDim * m, Dim * m);
    F.setZero();
    for (size_t i = 0; i < m; ++i)
      F.block<ZDim, Dim>(ZDim * i, Dim * i) = Fblocks.at(i);
  }


  Pose3 body_P_sensor() const{
    if(body_P_sensor_)
      return *body_P_sensor_;
    else
      return Pose3(); // if unspecified, the transformation is the identity
  }

private:

/// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};
// end class SmartFactorBase

// Definitions need to avoid link errors (above are only declarations)
template<class CAMERA> const int SmartFactorBase<CAMERA>::Dim;
template<class CAMERA> const int SmartFactorBase<CAMERA>::ZDim;

} // \ namespace gtsam
