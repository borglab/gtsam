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
 */

#pragma once

#include <gtsam/slam/JacobianFactorQ.h>
#include <gtsam/slam/JacobianFactorSVD.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/RegularHessianFactor.h>
#include <gtsam/geometry/CameraSet.h>

#include <boost/optional.hpp>
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

  /**
   * As of Feb 22, 2015, the noise model is the same for all measurements and
   * is isotropic. This allows for moving most calculations of Schur complement
   * etc to be moved to CameraSet very easily, and also agrees pragmatically
   * with what is normally done.
   */
  SharedIsotropic noiseModel_;

protected:

  /**
   * 2D measurement and noise model for each of the m views
   * We keep a copy of measurements for I/O and computing the error.
   * The order is kept the same as the keys that we use to create the factor.
   */
  std::vector<Z> measured_;

  static const int Dim = traits<CAMERA>::dimension; ///< Camera dimension
  static const int ZDim = traits<Z>::dimension; ///< Measurement dimension

  // Definitions for block matrices used internally
  typedef Eigen::Matrix<double, Dim, ZDim> MatrixD2; // F'
  typedef Eigen::Matrix<double, Dim, Dim> MatrixDD; // camera hessian block
  typedef Eigen::Matrix<double, ZDim, 3> Matrix23;
  typedef Eigen::Matrix<double, Dim, 1> VectorD;
  typedef Eigen::Matrix<double, ZDim, ZDim> Matrix2;

  // check that noise model is isotropic and the same
  // TODO, this is hacky, we should just do this via constructor, not add
  void maybeSetNoiseModel(const SharedNoiseModel& sharedNoiseModel) {
    if (!sharedNoiseModel)
      return;
    SharedIsotropic sharedIsotropic = boost::dynamic_pointer_cast<
        noiseModel::Isotropic>(sharedNoiseModel);
    if (!sharedIsotropic)
      throw std::runtime_error("SmartFactorBase: needs isotropic");
    if (!noiseModel_)
      noiseModel_ = sharedIsotropic;
    else if (!sharedIsotropic->equals(*noiseModel_))
      throw std::runtime_error(
          "SmartFactorBase: cannot add measurements with different noise model");
  }

public:

  // Definitions for blocks of F, externally visible
  typedef Eigen::Matrix<double, ZDim, Dim> Matrix2D; // F
  typedef std::pair<Key, Matrix2D> KeyMatrix2D; // Fblocks

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// We use the new CameraSte data structure to refer to a set of cameras
  typedef CameraSet<CAMERA> Cameras;

  /// Virtual destructor, subclasses from NonlinearFactor
  virtual ~SmartFactorBase() {
  }

  /**
   * Add a new measurement and pose key
   * @param measured_i is the 2m dimensional projection of a single landmark
   * @param poseKey is the index corresponding to the camera observing the landmark
   * @param sharedNoiseModel is the measurement noise
   */
  void add(const Z& measured_i, const Key& cameraKey_i,
      const SharedNoiseModel& sharedNoiseModel) {
    this->measured_.push_back(measured_i);
    this->keys_.push_back(cameraKey_i);
    maybeSetNoiseModel(sharedNoiseModel);
  }

  /**
   * Add a bunch of measurements, together with the camera keys and noises
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& cameraKeys,
      std::vector<SharedNoiseModel>& noises) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(cameraKeys.at(i));
      maybeSetNoiseModel(noises.at(i));
    }
  }

  /**
   * Add a bunch of measurements and uses the same noise model for all of them
   */
  void add(std::vector<Z>& measurements, std::vector<Key>& cameraKeys,
      const SharedNoiseModel& noise) {
    for (size_t i = 0; i < measurements.size(); i++) {
      this->measured_.push_back(measurements.at(i));
      this->keys_.push_back(cameraKeys.at(i));
      maybeSetNoiseModel(noise);
    }
  }

  /**
   * Adds an entire SfM_track (collection of cameras observing a single point).
   * The noise is assumed to be the same for all measurements
   */
  template<class SFM_TRACK>
  void add(const SFM_TRACK& trackToAdd, const SharedNoiseModel& noise) {
    for (size_t k = 0; k < trackToAdd.number_measurements(); k++) {
      this->measured_.push_back(trackToAdd.measurements[k].second);
      this->keys_.push_back(trackToAdd.measurements[k].first);
      maybeSetNoiseModel(noise);
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
  Cameras cameras(const Values& values) const {
    Cameras cameras;
    BOOST_FOREACH(const Key& k, this->keys_)
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
    Base::print("", keyFormatter);
  }

  /// equals
  virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This*>(&p);

    bool areMeasurementsEqual = true;
    for (size_t i = 0; i < measured_.size(); i++) {
      if (this->measured_.at(i).equals(e->measured_.at(i), tol) == false)
        areMeasurementsEqual = false;
      break;
    }
    return e && Base::equals(p, tol) && areMeasurementsEqual;
  }

  /// Compute reprojection errors
  Vector reprojectionError(const Cameras& cameras, const Point3& point) const {
    return cameras.reprojectionError(point, measured_);
  }

  /**
   *  Compute reprojection errors and derivatives
   */
  Vector reprojectionError(const Cameras& cameras, const Point3& point,
      typename Cameras::FBlocks& F, Matrix& E) const {
    return cameras.reprojectionError(point, measured_, F, E);
  }

  /// Calculate vector of re-projection errors, noise model applied
  Vector whitenedErrors(const Cameras& cameras, const Point3& point) const {
    Vector b = cameras.reprojectionError(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(b);
    return b;
  }

  /// Calculate vector of re-projection errors, noise model applied
  // TODO: Unit3
  Vector whitenedErrorsAtInfinity(const Cameras& cameras,
      const Point3& point) const {
    Vector b = cameras.reprojectionErrorAtInfinity(point, measured_);
    if (noiseModel_)
      noiseModel_->whitenInPlace(b);
    return b;
  }

  /** Calculate the error of the factor.
   * This is the log-likelihood, e.g. \f$ 0.5(h(x)-z)^2/\sigma^2 \f$ in case of Gaussian.
   * In this class, we take the raw prediction error \f$ h(x)-z \f$, ask the noise model
   * to transform it to \f$ (h(x)-z)^2/\sigma^2 \f$, and then multiply by 0.5.
   * This is different from reprojectionError(cameras,point) as each point is whitened
   */
  double totalReprojectionError(const Cameras& cameras,
      const Point3& point) const {
    Vector b = whitenedErrors(cameras, point);
    return 0.5 * b.dot(b);
  }

  /// Version for infinity
  // TODO: Unit3
  double totalReprojectionErrorAtInfinity(const Cameras& cameras,
      const Point3& point) const {
    Vector b = whitenedErrorsAtInfinity(cameras, point);
    return 0.5 * b.dot(b);
  }

  /// Computes Point Covariance P from E
  static Matrix3 PointCov(Matrix& E) {
    return (E.transpose() * E).inverse();
  }

  /// Computes Point Covariance P, with lambda parameter
  static Matrix3 PointCov(Matrix& E, double lambda,
      bool diagonalDamping = false) {

    Matrix3 EtE = E.transpose() * E;

    if (diagonalDamping) { // diagonal of the hessian
      EtE(0, 0) += lambda * EtE(0, 0);
      EtE(1, 1) += lambda * EtE(1, 1);
      EtE(2, 2) += lambda * EtE(2, 2);
    } else {
      EtE(0, 0) += lambda;
      EtE(1, 1) += lambda;
      EtE(2, 2) += lambda;
    }

    return (EtE).inverse();
  }

  /**
   *  Compute F, E, and b (called below in both vanilla and SVD versions), where
   *  F is a vector of derivatives wrpt the cameras, and E the stacked derivatives
   *  with respect to the point. The value of cameras/point are passed as parameters.
   */
  double computeJacobians(std::vector<KeyMatrix2D>& Fblocks, Matrix& E,
      Vector& b, const Cameras& cameras, const Point3& point) const {

    // Project into Camera set and calculate derivatives
    typename Cameras::FBlocks F;
    b = reprojectionError(cameras, point, F, E);

    // Now calculate f and divide up the F derivatives into Fblocks
    double f = 0.0;
    size_t m = keys_.size();
    for (size_t i = 0, row = 0; i < m; i++, row += ZDim) {

      // Accumulate normalized error
      f += b.segment<ZDim>(row).squaredNorm();

      // Push piece of F onto Fblocks with associated key
      Fblocks.push_back(KeyMatrix2D(keys_[i], F[i]));
    }
    return f;
  }

  /// SVD version
  double computeJacobiansSVD(std::vector<KeyMatrix2D>& Fblocks, Matrix& Enull,
      Vector& b, const Cameras& cameras, const Point3& point) const {

    Matrix E;
    double f = computeJacobians(Fblocks, E, b, cameras, point);

    // Do SVD on A
    Eigen::JacobiSVD<Matrix> svd(E, Eigen::ComputeFullU);
    Vector s = svd.singularValues();
    size_t m = this->keys_.size();
    Enull = svd.matrixU().block(0, 3, ZDim * m, ZDim * m - 3); // last ZDim*m-3 columns

    return f;
  }

  /**
   * Linearize returns a Hessianfactor that is an approximation of error(p)
   */
  boost::shared_ptr<RegularHessianFactor<Dim> > createHessianFactor(
      const Cameras& cameras, const Point3& point, const double lambda = 0.0,
      bool diagonalDamping = false) const {

    int m = this->keys_.size();
    std::vector<KeyMatrix2D> Fblocks;
    Matrix E;
    Vector b;
    double f = computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);

    // Create a SymmetricBlockMatrix
    size_t M1 = Dim * m + 1;
    std::vector<DenseIndex> dims(m + 1); // this also includes the b term
    std::fill(dims.begin(), dims.end() - 1, Dim);
    dims.back() = 1;
    SymmetricBlockMatrix augmentedHessian(dims, Matrix::Zero(M1, M1));

    // build augmented hessian
    sparseSchurComplement(Fblocks, E, P, b, augmentedHessian);
    augmentedHessian(m, m)(0, 0) = f;

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
    std::vector<KeyMatrix2D> Fblocks;
    double f = computeJacobians(Fblocks, E, b, cameras, point);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    RegularImplicitSchurFactor<Dim, ZDim>::updateSparseSchurComplement(Fblocks,
        E, P, b, f, allKeys, keys_, augmentedHessian);
  }

  /// Whiten the Jacobians computed by computeJacobians using noiseModel_
  void whitenJacobians(std::vector<KeyMatrix2D>& F, Matrix& E,
      Vector& b) const {
    noiseModel_->WhitenSystem(E, b);
    // TODO make WhitenInPlace work with any dense matrix type
    BOOST_FOREACH(KeyMatrix2D& Fblock,F)
      Fblock.second = noiseModel_->Whiten(Fblock.second);
  }

  /**
   * Return Jacobians as RegularImplicitSchurFactor with raw access
   */
  boost::shared_ptr<RegularImplicitSchurFactor<Dim, ZDim> > //
  createRegularImplicitSchurFactor(const Cameras& cameras, const Point3& point,
      double lambda = 0.0, bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    std::vector<KeyMatrix2D> F;
    computeJacobians(F, E, b, cameras, point);
    whitenJacobians(F, E, b);
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    return boost::make_shared<RegularImplicitSchurFactor<Dim, ZDim> >(F, E, P,
        b);
  }

  /**
   * Return Jacobians as JacobianFactorQ
   */
  boost::shared_ptr<JacobianFactorQ<Dim, ZDim> > createJacobianQFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0,
      bool diagonalDamping = false) const {
    Matrix E;
    Vector b;
    std::vector<KeyMatrix2D> F;
    computeJacobians(F, E, b, cameras, point);
    const size_t M = b.size();
    Matrix3 P = PointCov(E, lambda, diagonalDamping);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M, noiseModel_->sigma());
    return boost::make_shared<JacobianFactorQ<Dim, ZDim> >(F, E, P, b, n);
  }

  /**
   * Return Jacobians as JacobianFactor
   * TODO lambda is currently ignored
   */
  boost::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0) const {
    size_t m = this->keys_.size();
    std::vector<KeyMatrix2D> F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);
    computeJacobiansSVD(F, E0, b, cameras, point);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M-3, noiseModel_->sigma());
    return boost::make_shared<JacobianFactorSVD<Dim, ZDim> >(F, E0, b, n);
  }

  /// Create BIG block-diagonal matrix F from Fblocks
  static void FillDiagonalF(const std::vector<KeyMatrix2D>& Fblocks,
      Matrix& F) {
    size_t m = Fblocks.size();
    F.resize(ZDim * m, Dim * m);
    F.setZero();
    for (size_t i = 0; i < m; ++i)
      F.block<This::ZDim, Dim>(This::ZDim * i, Dim * i) = Fblocks.at(i).second;
  }

private:

/// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }
};
// end class SmartFactorBase

// Definitions need to avoid link errors (above are only declarations)
template<class CAMERA> const int SmartFactorBase<CAMERA>::Dim;
template<class CAMERA> const int SmartFactorBase<CAMERA>::ZDim;

} // \ namespace gtsam
