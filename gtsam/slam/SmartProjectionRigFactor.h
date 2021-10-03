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
 *         - it admits a different calibration for each measurement (i.e., it can model a multi-camera rig system)
 *         - it allows multiple observations from the same pose/key (again, to model a multi-camera system)
 * @author Luca Carlone
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {
/**
 *
 * @addtogroup SLAM
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating conditionally
 * independent sets in factor graphs: a unifying perspective based on smart factors,
 * Int. Conf. on Robotics and Automation (ICRA), 2014.
 */

/**
 * This factor assumes that camera calibration is fixed (but each camera
 * measurement can have a different extrinsic and intrinsic calibration).
 * The factor only constrains poses (variable dimension is 6).
 * This factor requires that values contains the involved poses (Pose3).
 * If all measurements share the same calibration (i.e., are from the same camera), use SmartProjectionPoseFactor instead!
 * If the calibration should be optimized, as well, use SmartProjectionFactor instead!
 * @addtogroup SLAM
 */
template<class CAMERA>
class SmartProjectionRigFactor : public SmartProjectionFactor<CAMERA> {

 private:
  typedef SmartProjectionFactor<CAMERA> Base;
  typedef SmartProjectionRigFactor<CAMERA> This;
  typedef typename CAMERA::CalibrationType CALIBRATION;

  static const int DimPose = 6;  ///< Pose3 dimension
  static const int ZDim = 2;  ///< Measurement dimension (Point2)

 protected:

  /// vector of keys (one for each observation) with potentially repeated keys
  KeyVector nonUniqueKeys_;

  /// cameras in the rig (fixed poses wrt body + fixed intrinsics)
  typename Base::Cameras cameraRig_;

  /// vector of camera Ids (one for each observation), identifying which camera took the measurement
  FastVector<size_t> cameraIds_;

 public:
  typedef CAMERA Camera;
  typedef CameraSet<CAMERA> Cameras;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor, only for serialization
  SmartProjectionRigFactor() {
  }

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param cameraRig set of cameras (fixed poses wrt body and intrinsics) in the camera rig
   * @param params parameters for the smart projection factors
   */
  SmartProjectionRigFactor(const SharedNoiseModel& sharedNoiseModel,
                         const Cameras& cameraRig,
                         const SmartProjectionParams& params =
                             SmartProjectionParams())
      : Base(sharedNoiseModel, params), cameraRig_(cameraRig) {
    // use only configuration that works with this factor
    Base::params_.degeneracyMode = gtsam::ZERO_ON_DEGENERACY;
    Base::params_.linearizationMode = gtsam::HESSIAN;
  }

  /** Virtual destructor */
  ~SmartProjectionRigFactor() override {
  }

  /**
   * add a new measurement, corresponding to an observation from pose "poseKey"
   * and taken from the camera in the rig identified by "cameraId"
   * @param measured 2-dimensional location of the projection of a
   * single landmark in a single view (the measurement)
   * @param poseKey key corresponding to the body pose of the camera taking the measurement
   * @param cameraId ID of the camera in the rig taking the measurement
   */
  void add(const Point2& measured, const Key& poseKey, const size_t cameraId) {
    // store measurement and key
    this->measured_.push_back(measured);
    this->nonUniqueKeys_.push_back(poseKey);

    //  also store keys in the keys_ vector: these keys are assumed to be unique, so we avoid duplicates here
    if (std::find(this->keys_.begin(), this->keys_.end(), poseKey) == this->keys_.end())
      this->keys_.push_back(poseKey);  // add only unique keys

    // store id of the camera taking the measurement
    cameraIds_.push_back(cameraId);
  }

  /**
   * Variant of the previous "add" function in which we include multiple measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m views (the measurements)
   * @param poseKeys keys corresponding to the body poses of the cameras taking the measurements
   * @param cameraIds IDs of the cameras in the rig taking each measurement (same order as measurements)
   */
  void add(const Point2Vector& measurements, const KeyVector& poseKeys,
           const FastVector<size_t>& cameraIds) {
    assert(poseKeys.size() == measurements.size());
    assert(poseKeys.size() == cameraIds.size());
    for (size_t i = 0; i < measurements.size(); i++) {
      add(measurements[i], poseKeys[i], cameraIds[i]);
    }
  }

  /// return (for each observation) the (possibly non unique) keys involved in the measurements
  const KeyVector nonUniqueKeys() const {
    return nonUniqueKeys_;
  }

  /// return the calibration object
  inline Cameras cameraRig() const {
    return cameraRig_;
  }

  /// return the calibration object
  inline FastVector<size_t> cameraIds() const {
    return cameraIds_;
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "SmartProjectionRigFactor: \n ";
    for (size_t i = 0; i < nonUniqueKeys_.size(); i++) {
      std::cout << "-- Measurement nr " << i << std::endl;
      std::cout << "key: " << keyFormatter(nonUniqueKeys_[i]) << std::endl;
      std::cout << "cameraId: " << cameraIds_[i] << std::endl;
      cameraRig_[ cameraIds_[i] ].print("camera in rig:\n");
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol)
    && nonUniqueKeys_ == e->nonUniqueKeys()
    && cameraRig_.equals(e->cameraRig())
    && std::equal(cameraIds_.begin(), cameraIds_.end(), e->cameraIds().begin());
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain body poses corresponding
   * to keys involved in this factor
   * @return vector of cameras
   */
  typename Base::Cameras cameras(const Values& values) const override {
    typename Base::Cameras cameras;
    for (size_t i = 0; i < nonUniqueKeys_.size(); i++) {
      const Key cameraId = cameraIds_[i];
      const Pose3& body_P_cam_i = cameraRig_[cameraId].pose();
      const Pose3 world_P_sensor_i = values.at<Pose3>(nonUniqueKeys_[i])
          * body_P_cam_i;
      cameras.emplace_back(world_P_sensor_i,
                           make_shared<typename CAMERA::CalibrationType>(cameraRig_[cameraId].calibration()));
    }
    return cameras;
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
   * @return Return arguments are the camera jacobians Fs (including the jacobian with
   * respect to both body poses we interpolate from), the point Jacobian E,
   * and the error vector b. Note that the jacobians are computed for a given point.
   */
  void computeJacobiansWithTriangulatedPoint(typename Base::FBlocks& Fs,
                                             Matrix& E, Vector& b,
                                             const Cameras& cameras) const {
    if (!this->result_) {
      throw("computeJacobiansWithTriangulatedPoint");
    } else {  // valid result: compute jacobians
      b = -cameras.reprojectionError(*this->result_, this->measured_, Fs, E);
      for (size_t i = 0; i < Fs.size(); i++) {
        const Key cameraId = cameraIds_[i];
        const Pose3 body_P_sensor = cameraRig_[cameraId].pose();
        const Pose3 sensor_P_body = body_P_sensor.inverse();
        const Pose3 world_P_body = cameras[i].pose() * sensor_P_body;
        Eigen::Matrix<double, DimPose, DimPose> H;
        world_P_body.compose(body_P_sensor, H);
        Fs.at(i) = Fs.at(i) * H;
      }
    }
  }

  /// linearize and return a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<DimPose> > createHessianFactor(
      const Values& values, const double lambda = 0.0, bool diagonalDamping =
          false) const {

    // we may have multiple observation sharing the same keys (e.g., 2 cameras measuring from the same body pose),
    // hence the number of unique keys may be smaller than nrMeasurements
    size_t nrUniqueKeys = this->keys_.size(); // note: by construction, keys_ only contains unique keys

    Cameras cameras = this->cameras(values);

    // Create structures for Hessian Factors
    FastVector<size_t> js;
    FastVector < Matrix > Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    FastVector < Vector > gs(nrUniqueKeys);

    if (this->measured_.size() != this->cameras(values).size())  // 1 observation per camera
      throw std::runtime_error("SmartProjectionRigFactor: "
                               "measured_.size() inconsistent with input");

    // triangulate 3D point at given linearization point
    this->triangulateSafe(cameras);

    if (!this->result_) {  // failed: return "empty/zero" Hessian
      if (this->params_.degeneracyMode == ZERO_ON_DEGENERACY) {
        for (Matrix& m : Gs)
          m = Matrix::Zero(DimPose, DimPose);
        for (Vector& v : gs)
          v = Vector::Zero(DimPose);
        return boost::make_shared < RegularHessianFactor<DimPose>
            > (this->keys_, Gs, gs, 0.0);
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
    for (size_t i = 0; i < Fs.size(); i++){
      Fs[i] = this->noiseModel_->Whiten(Fs[i]);
    }

    const Matrix3 P = Base::Cameras::PointCov(E, lambda, diagonalDamping);

    // Build augmented Hessian (with last row/column being the information vector)
    // Note: we need to get the augumented hessian wrt the unique keys in key_
    SymmetricBlockMatrix augmentedHessianUniqueKeys =
        Base::Cameras::template SchurComplementAndRearrangeBlocks<3, 6, 6>(
            Fs, E, P, b, nonUniqueKeys_, this->keys_);

    return boost::make_shared < RegularHessianFactor<DimPose>
    > (this->keys_, augmentedHessianUniqueKeys);
  }

  /**
   * Linearize to Gaussian Factor (possibly adding a damping factor Lambda for LM)
   * @param values Values structure which must contain camera poses and extrinsic pose for this factor
   * @return a Gaussian factor
   */
  boost::shared_ptr<GaussianFactor> linearizeDamped(
      const Values& values, const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    switch (this->params_.linearizationMode) {
      case HESSIAN:
        return this->createHessianFactor(values, lambda);
      default:
        throw std::runtime_error(
            "SmartProjectionRigFactor: unknown linearization mode");
    }
  }

  /// linearize
  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const
          override {
    return this->linearizeDamped(values);
  }

 private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(nonUniqueKeys_);
    ar & BOOST_SERIALIZATION_NVP(cameraRig_);
    ar & BOOST_SERIALIZATION_NVP(cameraIds_);
  }

};
// end of class declaration

/// traits
template<class CAMERA>
struct traits<SmartProjectionRigFactor<CAMERA> > : public Testable<
    SmartProjectionRigFactor<CAMERA> > {
};

}  // \ namespace gtsam
