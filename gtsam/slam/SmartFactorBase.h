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
#include <gtsam/base/SymmetricBlockMatrix.h>

#include <algorithm>
#include <optional>
#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/optional.hpp>
#include <boost/serialization/version.hpp>
#endif
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
   * The order is kept the same as the keys that we use to create the factor;
   * a measurement whose camera is fixed keeps its place but has no key.
   */
  ZVector measured_;

  std::optional<Pose3>
      body_P_sensor_;  ///< Pose of the camera in the body frame

  // Cache for Fblocks, to avoid a malloc ever time we re-linearize
  mutable FBlocks Fs;

  /// Cameras held constant, see conditionOn; parallel to fixedMeasurements_.
  CameraSet<CAMERA> fixedCameras_;

  /// Increasing indices into measured_ of the measurements with a fixed camera.
  FastVector<size_t> fixedMeasurements_;

 public:
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
#ifndef NDEBUG
    if (measurements.size() != cameraKeys.size()) {
      throw std::runtime_error("Number of measurements and camera keys do not match");
    }
#endif
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

  /// Collect all cameras in measurement order, fixed cameras included.
  virtual Cameras cameras(const Values& values) const {
    return assembleCameras([&](size_t, size_t keyIndex) {
      return camera(this->keys_[keyIndex], values);
    });
  }

  /// @name Fixed cameras
  /// @{

  /// Cameras held constant, in measurement order.
  const Cameras& fixedCameras() const { return fixedCameras_; }

  /// Indices of the measurements whose camera is fixed.
  const FastVector<size_t>& fixedMeasurements() const {
    return fixedMeasurements_;
  }

  /// Whether the camera of measurement i is fixed.
  bool isFixedMeasurement(size_t i) const {
    return std::binary_search(fixedMeasurements_.begin(),
                              fixedMeasurements_.end(), i);
  }

  /// Indices of the measurements whose camera is not fixed.
  FastVector<size_t> activeMeasurements() const {
    FastVector<size_t> active;
    for (size_t i = 0; i < measured_.size(); ++i)
      if (!isFixedMeasurement(i)) active.push_back(i);
    return active;
  }

  /// @}

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
    for (size_t k = 0; k < fixedCameras_.size(); ++k) {
      std::cout << "fixed camera for measurement " << fixedMeasurements_[k]
                << ":\n";
      fixedCameras_[k].print();
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    if (const This* e = dynamic_cast<const This*>(&p)) {
      // Check that all measurements are the same.
      if (measured_.size() != e->measured_.size()) return false;
      for (size_t i = 0; i < measured_.size(); i++) {
        if (!traits<Z>::Equals(this->measured_.at(i), e->measured_.at(i), tol))
          return false;
      }
      // Check that the same measurements are fixed at the same cameras.
      if (fixedMeasurements_ != e->fixedMeasurements_ ||
          !fixedCameras_.equals(e->fixedCameras_, tol))
        return false;
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
  template<class POINT, class ...OptArgs, typename = std::enable_if_t<sizeof...(OptArgs)!=0>>
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
    // Fixed cameras are constants: their rows only constrain the landmark.
    for (size_t i : fixedMeasurements_) Fs.at(i).setZero();
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
    size_t m = this->measured_.size();
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
    SymmetricBlockMatrix augmentedHessian =
        activeBlocks(Cameras::SchurComplement(Fs, E, b));

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
    if (!fixedMeasurements_.empty())
      throw std::invalid_argument(
          "SmartFactorBase::updateAugmentedHessian does not support fixed "
          "cameras");
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
    if (!fixedMeasurements_.empty())
      throw std::invalid_argument(
          "SmartFactorBase::createRegularImplicitSchurFactor does not support "
          "fixed cameras");
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
    return std::make_shared<JacobianFactorQ<Dim, ZDim> >(
        keys_, activeFBlocks(std::move(F)), E, P, b, n, activeMeasurements());
  }

  /**
   * Return Jacobians as JacobianFactorSVD.
   * TODO(dellaert): lambda is currently ignored
   */
  std::shared_ptr<JacobianFactor> createJacobianSVDFactor(
      const Cameras& cameras, const Point3& point, double lambda = 0.0) const {
    size_t m = this->measured_.size();
    FBlocks F;
    Vector b;
    const size_t M = ZDim * m;
    Matrix E0(M, M - 3);
    computeJacobiansSVD(F, E0, b, cameras, point);
    SharedIsotropic n = noiseModel::Isotropic::Sigma(M - 3,
        noiseModel_->sigma());
    return std::make_shared<JacobianFactorSVD<Dim, ZDim> >(
        keys_, activeFBlocks(std::move(F)), E0, b, n, activeMeasurements());
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

 protected:
  /// Camera of a key; the default reads it from values.
  virtual CAMERA camera(Key key, const Values& values) const {
    return values.at<CAMERA>(key);
  }

  /**
   * Cameras in measurement order: the fixed ones, and liveCamera(i, k) for
   * measurement i whose camera is the k-th that is not fixed.
   */
  template <class LIVE_CAMERA>
  Cameras assembleCameras(const LIVE_CAMERA& liveCamera) const {
    Cameras cameras;
    cameras.reserve(measured_.size());
    size_t keyIndex = 0, fixedIndex = 0;
    for (size_t i = 0; i < measured_.size(); ++i) {
      if (isFixedMeasurement(i))
        cameras.push_back(fixedCameras_[fixedIndex++]);
      else
        cameras.push_back(liveCamera(i, keyIndex++));
    }
    return cameras;
  }

  /// The F blocks of the cameras that are not fixed.
  FBlocks activeFBlocks(FBlocks F) const {
    if (fixedMeasurements_.empty()) return F;
    FBlocks active;
    for (size_t i : activeMeasurements()) active.push_back(F[i]);
    return active;
  }

  /**
   * Restrict an augmented Hessian over all measurements, with zero F blocks
   * for the fixed cameras, to the cameras that are not fixed.
   */
  SymmetricBlockMatrix activeBlocks(SymmetricBlockMatrix full) const {
    if (fixedMeasurements_.empty()) return full;
    const FastVector<size_t> active = activeMeasurements();
    const size_t n = active.size(), m = measured_.size();
    std::vector<DenseIndex> dims(n + 1, Dim);
    dims.back() = 1;
    SymmetricBlockMatrix result(dims, Matrix::Zero(Dim * n + 1, Dim * n + 1));
    for (size_t i = 0; i < n; ++i) {
      const Matrix diagonal = full.diagonalBlock(active[i]);
      result.setDiagonalBlock(i, diagonal);
      for (size_t j = i + 1; j < n; ++j)
        result.setOffDiagonalBlock(
            i, j, full.aboveDiagonalBlock(active[i], active[j]));
      result.setOffDiagonalBlock(i, n, full.aboveDiagonalBlock(active[i], m));
    }
    const Matrix constant = full.diagonalBlock(m);
    result.setDiagonalBlock(n, constant);
    return result;
  }

  /// Hold the camera of measurement measurementIndex constant at camera.
  void fixMeasurementInPlace(size_t measurementIndex, const CAMERA& camera) {
    const auto position = std::lower_bound(
        fixedMeasurements_.begin(), fixedMeasurements_.end(), measurementIndex);
    fixedCameras_.insert(
        fixedCameras_.begin() + (position - fixedMeasurements_.begin()),
        camera);
    fixedMeasurements_.insert(position, measurementIndex);
  }

  /// Position of key in keys(); throws if the factor does not have it.
  KeyVector::iterator findKey(Key key) {
    const auto keyIterator =
        std::find(this->keys_.begin(), this->keys_.end(), key);
    if (keyIterator == this->keys_.end())
      throw std::invalid_argument("SmartFactorBase: key " +
                                  DefaultKeyFormatter(key) +
                                  " is not a key of this factor");
    return keyIterator;
  }

  /**
   * Hold the camera of key constant at camera(key, values), removing the key
   * and keeping its measurement. Only for fresh copies: a factor in a graph
   * must not change. Factors whose cameras() does not use camera() override
   * this.
   */
  virtual void fixKeyInPlace(Key key, const Values& values) {
    const auto keyIterator = findKey(key);
    const size_t keyIndex = keyIterator - this->keys_.begin();
    fixMeasurementInPlace(activeMeasurements().at(keyIndex),
                          camera(key, values));
    this->keys_.erase(keyIterator);
  }

  /// Erase measurement i; derived classes also erase their data for it.
  virtual void eraseMeasurementAt(size_t i) {
    measured_.erase(measured_.begin() + i);
  }

  /// Drop the oldest fixed camera together with its measurement.
  void eraseOldestFixedMeasurement() {
    eraseMeasurementAt(fixedMeasurements_.front());
    fixedCameras_.erase(fixedCameras_.begin());
    fixedMeasurements_.erase(fixedMeasurements_.begin());
    for (size_t& index : fixedMeasurements_) --index;
  }

  /**
   * Implements NonlinearFactor::conditionOn: fix every key with a value in
   * fixedValues on a copy, then keep at most maxFixedCameras fixed cameras,
   * the newest (zero keeps all).
   */
  NonlinearFactor::shared_ptr conditionOnKeys(const Values& fixedValues,
                                              size_t maxFixedCameras) const {
    KeyVector keysToFix;
    for (Key key : this->keys_)
      if (fixedValues.exists(key)) keysToFix.push_back(key);
    if (keysToFix.empty() || keysToFix.size() == this->keys_.size())
      return nullptr;
    auto fixed = std::static_pointer_cast<This>(this->clone());
    for (Key key : keysToFix) fixed->fixKeyInPlace(key, fixedValues);
    while (maxFixedCameras > 0 && fixed->fixedCameras_.size() > maxFixedCameras)
      fixed->eraseOldestFixedMeasurement();
    return fixed;
  }

private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION///
/// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(noiseModel_);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
    if (version > 0) {
      ar & BOOST_SERIALIZATION_NVP(fixedCameras_);
      ar & BOOST_SERIALIZATION_NVP(fixedMeasurements_);
    }
  }
#endif
};
// end class SmartFactorBase

// Definitions need to avoid link errors (above are only declarations)
template<class CAMERA> const int SmartFactorBase<CAMERA>::Dim;
template<class CAMERA> const int SmartFactorBase<CAMERA>::ZDim;

} // \ namespace gtsam

#if GTSAM_ENABLE_BOOST_SERIALIZATION
namespace boost {
namespace serialization {

/** Version 1 adds the fixed cameras. */
template <class CAMERA>
struct version<gtsam::SmartFactorBase<CAMERA>> {
  typedef mpl::int_<1> type;
  typedef mpl::integral_c_tag tag;
  BOOST_STATIC_CONSTANT(int, value = type::value);
};

}  // namespace serialization
}  // namespace boost
#endif
