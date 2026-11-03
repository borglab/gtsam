/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionRigFactor.h
 * @brief  Smart factor on poses, assuming camera calibration is fixed.
 *         Same as SmartProjectionPoseFactor, except:
 *         - it is templated on CAMERA (i.e., it allows cameras beyond pinhole)
 *         - it admits a different calibration for each measurement (i.e., it
 * can model a multi-camera rig system)
 *         - it allows multiple observations from the same pose/key (again, to
 * model a multi-camera system)
 * @author Luca Carlone
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {
/**
 *
 * @ingroup slam
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014.
 */

/**
 * This factor assumes that camera calibration is fixed (but each measurement
 * can be taken by a different camera in the rig, hence can have a different
 * extrinsic and intrinsic calibration). The factor only constrains poses
 * (variable dimension is 6 for each pose). This factor requires that values
 * contains the involved poses (Pose3). If all measurements share the same
 * calibration (i.e., are from the same camera), use SmartProjectionPoseFactor
 * instead! If the calibration should be optimized, as well, use
 * SmartProjectionFactor instead!
 *
 * <b>Note on Template Parameter `CAMERA`:</b>
 * While this factor is templated on `CAMERA` to allow for generality (e.g.,
 * `SphericalCamera`), the current internal implementation for linearization
 * (specifically, methods like `createHessianFactor` involving Schur complement
 * calculations inherited or adapted from base classes) has limitations. It
 * implicitly assumes that the CAMERA only has a Pose3 unknown.
 *
 * Consequently:
 * - This factor works correctly with `CAMERA` types where this is the case,
 *   such as `PinholePose<CALIBRATION>` or `SphericalCamera`.
 * - Using `CAMERA` types where `dimension != 6`, such as
 *   `PinholeCamera<CALIBRATION>` (which has dimension `6 + CalDim`), will lead
 *   to compilation errors due to matrix dimension mismatches.
 *
 * Therefore, for standard pinhole cameras within a fixed rig, use
 * `PinholePose<CALIBRATION>` as the `CAMERA` template parameter when defining
 * the `CameraSet` passed to this factor's constructor.
 *
 * TODO(dellaert): Refactor the internal linearization logic (e.g., in
 * `createHessianFactor`) to explicitly compute Jacobians with respect to the
 * 6-DoF body pose by correctly applying the chain rule, rather than relying on
 * `traits<CAMERA>::dimension` for downstream calculations.
 *
 * @ingroup slam
 */
template <class CAMERA>
class SmartProjectionRigFactor : public SmartProjectionFactor<CAMERA> {
 private:
  typedef SmartProjectionFactor<CAMERA> Base;
  typedef SmartProjectionRigFactor<CAMERA> This;
  typedef typename CAMERA::CalibrationType CALIBRATION;
  typedef typename CAMERA::Measurement MEASUREMENT;
  typedef typename CAMERA::MeasurementVector MEASUREMENTS;

  static const int DimPose = 6;  ///< Pose3 dimension
  static const int ZDim = 2;     ///< Measurement dimension

 protected:
  /// vector of keys (one for each observation) with potentially repeated keys
  KeyVector nonUniqueKeys_;

  /// cameras in the rig (fixed poses wrt body and intrinsics, for each camera)
  std::shared_ptr<typename Base::Cameras> cameraRig_;

  /// vector of camera Ids (one for each observation, in the same order),
  /// identifying which camera took the measurement
  FastVector<size_t> cameraIds_;

 public:
  typedef CAMERA Camera;
  typedef CameraSet<CAMERA> Cameras;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor, only for serialization
  SmartProjectionRigFactor() {}

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature
   * measurements
   * @param cameraRig set of cameras (fixed poses wrt body and intrinsics) in
   * the camera rig
   * @param params parameters for the smart projection factors
   */
  SmartProjectionRigFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const std::shared_ptr<Cameras>& cameraRig,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel, params), cameraRig_(cameraRig) {
    // throw exception if configuration is not supported by this factor
    if (Base::params_.degeneracyMode != gtsam::ZERO_ON_DEGENERACY)
      throw std::runtime_error(
          "SmartProjectionRigFactor: "
          "degeneracyMode must be set to ZERO_ON_DEGENERACY");
    if (Base::params_.linearizationMode != gtsam::HESSIAN)
      throw std::runtime_error(
          "SmartProjectionRigFactor: "
          "linearizationMode must be set to HESSIAN");
  }

  /**
   * add a new measurement, corresponding to an observation from pose "poseKey"
   * and taken from the camera in the rig identified by "cameraId"
   * @param measured 2-dimensional location of the projection of a
   * single landmark in a single view (the measurement)
   * @param poseKey key corresponding to the body pose of the camera taking the
   * measurement
   * @param cameraId ID of the camera in the rig taking the measurement (default
   * 0)
   */
  void add(const MEASUREMENT& measured, const Key& poseKey,
           const size_t& cameraId = 0) {
    // store measurement and key
    this->measured_.push_back(measured);
    this->nonUniqueKeys_.push_back(poseKey);

    //  also store keys in the keys_ vector: these keys are assumed to be
    //  unique, so we avoid duplicates here
    if (std::find(this->keys_.begin(), this->keys_.end(), poseKey) ==
        this->keys_.end())
      this->keys_.push_back(poseKey);  // add only unique keys

    // store id of the camera taking the measurement
    cameraIds_.push_back(cameraId);
  }

  /**
   * Variant of the previous "add" function in which we include multiple
   * measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m views (the measurements)
   * @param poseKeys keys corresponding to the body poses of the cameras taking
   * the measurements
   * @param cameraIds IDs of the cameras in the rig taking each measurement
   * (same order as the measurements)
   */
  void add(const MEASUREMENTS& measurements, const KeyVector& poseKeys,
           const FastVector<size_t>& cameraIds = FastVector<size_t>()) {
    if (poseKeys.size() != measurements.size() ||
        (poseKeys.size() != cameraIds.size() && cameraIds.size() != 0)) {
      throw std::runtime_error(
          "SmartProjectionRigFactor: "
          "trying to add inconsistent inputs");
    }
    if (cameraIds.size() == 0 && cameraRig_->size() > 1) {
      throw std::runtime_error(
          "SmartProjectionRigFactor: "
          "camera rig includes multiple camera "
          "but add did not input cameraIds");
    }
    for (size_t i = 0; i < measurements.size(); i++) {
      add(measurements[i], poseKeys[i],
          cameraIds.size() == 0 ? 0 : cameraIds[i]);
    }
  }

  /// return (for each observation) the (possibly non unique) keys involved in
  /// the measurements
  const KeyVector& nonUniqueKeys() const { return nonUniqueKeys_; }

  /// return the calibration object
  const std::shared_ptr<Cameras>& cameraRig() const { return cameraRig_; }

  /// return the calibration object
  const FastVector<size_t>& cameraIds() const { return cameraIds_; }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "SmartProjectionRigFactor: \n ";
    for (size_t i = 0; i < nonUniqueKeys_.size(); i++) {
      std::cout << "-- Measurement nr " << i << std::endl;
      std::cout << "key: " << keyFormatter(nonUniqueKeys_[i]) << std::endl;
      std::cout << "cameraId: " << cameraIds_[i] << std::endl;
      (*cameraRig_)[cameraIds_[i]].print("camera in rig:\n");
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && nonUniqueKeys_ == e->nonUniqueKeys() &&
           cameraRig_->equals(*(e->cameraRig())) &&
           std::equal(cameraIds_.begin(), cameraIds_.end(),
                      e->cameraIds().begin());
  }

  /// Copy of this factor, as a NonlinearFactor.
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<This>(*this);
  }

  /// SmartProjectionFactorBase::fixCamera, returning the concrete type.
  shared_ptr fixCamera(Key key, const Values& values) const {
    return std::static_pointer_cast<This>(Base::fixCamera(key, values));
  }

  /// Rekey; the base version clones and renames keys_ only, so rename the
  /// key of every measurement as well.
  NonlinearFactor::shared_ptr rekey(
      const std::map<Key, Key>& rekey_mapping) const override {
    auto factor = std::static_pointer_cast<This>(Base::rekey(rekey_mapping));
    for (Key& key : factor->nonUniqueKeys_) {
      const auto mapping = rekey_mapping.find(key);
      if (mapping != rekey_mapping.end()) key = mapping->second;
    }
    return factor;
  }

  /// Rekey with a full replacement of keys_, see above.
  NonlinearFactor::shared_ptr rekey(const KeyVector& new_keys) const override {
    assert(new_keys.size() == this->keys_.size());
    std::map<Key, Key> rekey_mapping;
    for (size_t i = 0; i < new_keys.size(); ++i)
      rekey_mapping[this->keys_[i]] = new_keys[i];
    return rekey(rekey_mapping);
  }

  /// Camera of measurement i for the given body pose.
  Camera cameraForMeasurement(size_t i, const Pose3& world_P_body) const {
    const typename Base::Camera& camera_i = (*cameraRig_)[cameraIds_[i]];
    return Camera(world_P_body * camera_i.pose(),  // = world_P_cam_i
                  std::make_shared<typename CAMERA::CalibrationType>(
                      camera_i.calibration()));
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain body poses corresponding
   * to keys involved in this factor
   * @return vector of cameras
   */
  typename Base::Cameras cameras(const Values& values) const override {
    return this->assembleCameras([&](size_t i, size_t) {
      return cameraForMeasurement(i, values.at<Pose3>(nonUniqueKeys_[i]));
    });
  }

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override {
    if (this->active(values)) {
      return this->totalReprojectionError(this->cameras(values));
    } else {  // else of active flag
      return 0.0;
    }
  }

  /**
   * Compute jacobian F, E and error vector at a given linearization point
   * @param values Values structure which must contain camera poses
   * corresponding to keys involved in this factor
   * @return Return arguments are the camera jacobians Fs (including the
   * jacobian with respect to both body poses we interpolate from), the point
   * Jacobian E, and the error vector b. Note that the jacobians are computed
   * for a given point.
   */
  void computeJacobiansWithTriangulatedPoint(typename Base::FBlocks& Fs,
                                             Matrix& E, Vector& b,
                                             const Cameras& cameras) const {
    if (!this->result_) {
      throw("computeJacobiansWithTriangulatedPoint");
    } else {  // valid result: compute jacobians
      b = -cameras.reprojectionError(*this->result_, this->measured_, Fs, E);
      for (size_t i = 0; i < Fs.size(); i++) {
        const Pose3& body_P_sensor = (*cameraRig_)[cameraIds_[i]].pose();
        const Pose3 world_P_body = cameras[i].pose() * body_P_sensor.inverse();
        Eigen::Matrix<double, DimPose, DimPose> H;
        world_P_body.compose(body_P_sensor, H);
        Fs.at(i) = Fs.at(i) * H;
      }
      // Fixed cameras are constants: their rows only constrain the landmark.
      for (size_t i : this->fixedMeasurements_) Fs.at(i).setZero();
    }
  }

  /// linearize and return a Hessianfactor that is an approximation of error(p)
  std::shared_ptr<RegularHessianFactor<DimPose> > createHessianFactor(
      const Values& values, const double& lambda = 0.0,
      bool diagonalDamping = false) const {
    // we may have multiple observation sharing the same keys (e.g., 2 cameras
    // measuring from the same body pose), hence the number of unique keys may
    // be smaller than nrMeasurements
    size_t nrUniqueKeys =
        this->keys_
            .size();  // note: by construction, keys_ only contains unique keys

    Cameras cameras = this->cameras(values);

    // Create structures for Hessian Factors
    std::vector<size_t> js;
    std::vector<Matrix> Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    std::vector<Vector> gs(nrUniqueKeys);

    if (this->measured_.size() != cameras.size())  // 1 observation per camera
      throw std::runtime_error(
          "SmartProjectionRigFactor: "
          "measured_.size() inconsistent with input");

    // triangulate 3D point at given linearization point
    this->triangulateSafe(cameras);

    if (!this->result_) {  // failed: return "empty/zero" Hessian
      if (this->params_.degeneracyMode == ZERO_ON_DEGENERACY) {
        for (Matrix& m : Gs) m = Matrix::Zero(DimPose, DimPose);
        for (Vector& v : gs) v = Vector::Zero(DimPose);
        return std::make_shared<RegularHessianFactor<DimPose> >(this->keys_, Gs,
                                                                gs, 0.0);
      } else {
        throw std::runtime_error(
            "SmartProjectionRigFactor: "
            "only supported degeneracy mode is ZERO_ON_DEGENERACY");
      }
    }

    // compute Jacobian given triangulated 3D Point
    typename Base::FBlocks Fs;
    Matrix E;
    Vector b;
    this->computeJacobiansWithTriangulatedPoint(Fs, E, b, cameras);

    // Whiten using noise model
    this->noiseModel_->WhitenSystem(E, b);
    for (size_t i = 0; i < Fs.size(); i++) {
      Fs[i] = this->noiseModel_->Whiten(Fs[i]);
    }

    const Matrix3 P = Base::Cameras::PointCov(E, lambda, diagonalDamping);

    // Build augmented Hessian (with last row/column being the information
    // vector) Note: we need to get the augumented hessian wrt the unique keys
    // in key_
    // Fixed measurements have zero F blocks: any key can take them.
    KeyVector jacobianKeys = nonUniqueKeys_;
    for (size_t i : this->fixedMeasurements_)
      jacobianKeys[i] = this->keys_.front();
    SymmetricBlockMatrix augmentedHessianUniqueKeys =
        Base::Cameras::template SchurComplementAndRearrangeBlocks<3, 6, 6>(
            Fs, E, P, b, jacobianKeys, this->keys_);

    return std::make_shared<RegularHessianFactor<DimPose> >(
        this->keys_, augmentedHessianUniqueKeys);
  }

  /**
   * Linearize to Gaussian Factor (possibly adding a damping factor Lambda for
   * LM)
   * @param values Values structure which must contain camera poses and
   * extrinsic pose for this factor
   * @return a Gaussian factor
   */
  std::shared_ptr<GaussianFactor> linearizeDamped(
      const Values& values, const double& lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different
    // linear factors
    switch (this->params_.linearizationMode) {
      case HESSIAN:
        return this->createHessianFactor(values, lambda);
      default:
        throw std::runtime_error(
            "SmartProjectionRigFactor: unknown linearization mode");
    }
  }

  /// linearize
  std::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    return this->linearizeDamped(values);
  }

 protected:
  /// Fix every measurement taken from the body pose key at its value.
  void fixKeyInPlace(Key key, const Values& values) override {
    const auto keyIterator = this->findKey(key);
    const Pose3 world_P_body = values.at<Pose3>(key);
    for (size_t i = 0; i < nonUniqueKeys_.size(); i++)
      if (nonUniqueKeys_[i] == key && !this->isFixedMeasurement(i))
        this->fixMeasurementInPlace(i, cameraForMeasurement(i, world_P_body));
    this->keys_.erase(keyIterator);
  }

  /// Erase the key and camera id of measurement i as well.
  void eraseMeasurementAt(size_t i) override {
    nonUniqueKeys_.erase(nonUniqueKeys_.begin() + i);
    cameraIds_.erase(cameraIds_.begin() + i);
    Base::eraseMeasurementAt(i);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    // ar& BOOST_SERIALIZATION_NVP(nonUniqueKeys_);
    // ar& BOOST_SERIALIZATION_NVP(cameraRig_);
    // ar& BOOST_SERIALIZATION_NVP(cameraIds_);
  }
#endif
};
// end of class declaration

/// traits
template <class CAMERA>
struct traits<SmartProjectionRigFactor<CAMERA> >
    : public Testable<SmartProjectionRigFactor<CAMERA> > {};

}  // namespace gtsam
