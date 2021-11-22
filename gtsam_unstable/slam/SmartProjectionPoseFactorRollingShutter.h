/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionPoseFactorRollingShutter.h
 * @brief  Smart projection factor on poses modeling rolling shutter effect with
 * given readout time
 * @author Luca Carlone
 */

#pragma once

#include <gtsam/geometry/CameraSet.h>
#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {
/**
 *
 * @addtogroup SLAM
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert,
 * Eliminating conditionally independent sets in factor graphs:
 * a unifying perspective based on smart factors,
 * Int. Conf. on Robotics and Automation (ICRA), 2014.
 */

/**
 * This factor optimizes two consecutive poses of the body assuming a rolling
 * shutter model of the camera with given readout time. The factor requires that
 * values contain (for each pixel observation) two consecutive camera poses from
 * which the pixel observation pose can be interpolated.
 * @addtogroup SLAM
 */
template <class CAMERA>
class SmartProjectionPoseFactorRollingShutter
    : public SmartProjectionFactor<CAMERA> {
 private:
  typedef SmartProjectionFactor<CAMERA> Base;
  typedef SmartProjectionPoseFactorRollingShutter<CAMERA> This;
  typedef typename CAMERA::CalibrationType CALIBRATION;

 protected:
  /// The keys of the pose of the body (with respect to an external world
  /// frame): two consecutive poses for each observation
  std::vector<std::pair<Key, Key>> world_P_body_key_pairs_;

  /// interpolation factor (one for each observation) to interpolate between
  /// pair of consecutive poses
  std::vector<double> alphas_;

  /// one or more cameras taking observations (fixed poses wrt body + fixed
  /// intrinsics)
  boost::shared_ptr<typename Base::Cameras> cameraRig_;

  /// vector of camera Ids (one for each observation, in the same order),
  /// identifying which camera took the measurement
  FastVector<size_t> cameraIds_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef CAMERA Camera;
  typedef CameraSet<CAMERA> Cameras;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  static const int DimBlock =
      12;  ///< size of the variable stacking 2 poses from which the observation
           ///< pose is interpolated
  static const int DimPose = 6;  ///< Pose3 dimension
  static const int ZDim = 2;     ///< Measurement dimension (Point2)
  typedef Eigen::Matrix<double, ZDim, DimBlock>
      MatrixZD;  // F blocks (derivatives wrt block of 2 poses)
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD>>
      FBlocks;  // vector of F blocks

  /// Default constructor, only for serialization
  SmartProjectionPoseFactorRollingShutter() {}

  /**
   * Constructor
   * @param Isotropic measurement noise
   * @param cameraRig set of cameras (fixed poses wrt body and intrinsics)
   * taking the measurements
   * @param params internal parameters of the smart factors
   */
  SmartProjectionPoseFactorRollingShutter(
      const SharedNoiseModel& sharedNoiseModel,
      const boost::shared_ptr<Cameras>& cameraRig,
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

  /** Virtual destructor */
  ~SmartProjectionPoseFactorRollingShutter() override = default;

  /**
   * add a new measurement, with 2 pose keys, interpolation factor, and cameraId
   * @param measured 2-dimensional location of the projection of a single
   *  landmark in a single view (the measurement), interpolated from the 2 poses
   * @param world_P_body_key1 key corresponding to the first body poses (time <=
   *  time pixel is acquired)
   * @param world_P_body_key2 key corresponding to the second body poses (time
   *  >= time pixel is acquired)
   * @param alpha interpolation factor in [0,1], such that if alpha = 0 the
   *  interpolated pose is the same as world_P_body_key1
   * @param cameraId ID of the camera taking the measurement (default 0)
   */
  void add(const Point2& measured, const Key& world_P_body_key1,
           const Key& world_P_body_key2, const double& alpha,
           const size_t& cameraId = 0) {
    // store measurements in base class
    this->measured_.push_back(measured);

    // store the pair of keys for each measurement, in the same order
    world_P_body_key_pairs_.push_back(
        std::make_pair(world_P_body_key1, world_P_body_key2));

    //  also store keys in the keys_ vector: these keys are assumed to be
    //  unique, so we avoid duplicates here
    if (std::find(this->keys_.begin(), this->keys_.end(), world_P_body_key1) ==
        this->keys_.end())
      this->keys_.push_back(world_P_body_key1);  // add only unique keys
    if (std::find(this->keys_.begin(), this->keys_.end(), world_P_body_key2) ==
        this->keys_.end())
      this->keys_.push_back(world_P_body_key2);  // add only unique keys

    // store interpolation factor
    alphas_.push_back(alpha);

    // store id of the camera taking the measurement
    cameraIds_.push_back(cameraId);
  }

  /**
   * Variant of the previous "add" function in which we include multiple
   * measurements
   * @param measurements vector of the 2m dimensional location of the projection
   *  of a single landmark in the m views (the measurements)
   * @param world_P_body_key_pairs vector where the i-th element contains a pair
   *  of keys corresponding to the pair of poses from which the observation pose
   *  for the i0-th measurement can be interpolated
   * @param alphas vector of interpolation params (in [0,1]), one for each
   * measurement (in the same order)
   * @param cameraIds IDs of the cameras taking each measurement (same order as
   * the measurements)
   */
  void add(const Point2Vector& measurements,
           const std::vector<std::pair<Key, Key>>& world_P_body_key_pairs,
           const std::vector<double>& alphas,
           const FastVector<size_t>& cameraIds = FastVector<size_t>()) {
    if (world_P_body_key_pairs.size() != measurements.size() ||
        world_P_body_key_pairs.size() != alphas.size() ||
        (world_P_body_key_pairs.size() != cameraIds.size() &&
         cameraIds.size() != 0)) {  // cameraIds.size()=0 is default
      throw std::runtime_error(
          "SmartProjectionPoseFactorRollingShutter: "
          "trying to add inconsistent inputs");
    }
    if (cameraIds.size() == 0 && cameraRig_->size() > 1) {
      throw std::runtime_error(
          "SmartProjectionPoseFactorRollingShutter: "
          "camera rig includes multiple camera "
          "but add did not input cameraIds");
    }
    for (size_t i = 0; i < measurements.size(); i++) {
      add(measurements[i], world_P_body_key_pairs[i].first,
          world_P_body_key_pairs[i].second, alphas[i],
          cameraIds.size() == 0 ? 0
                                : cameraIds[i]);  // use 0 as default if
                                                  // cameraIds was not specified
    }
  }

  /// return (for each observation) the keys of the pair of poses from which we
  /// interpolate
  const std::vector<std::pair<Key, Key>>& world_P_body_key_pairs() const {
    return world_P_body_key_pairs_;
  }

  /// return the interpolation factors alphas
  const std::vector<double>& alphas() const { return alphas_; }

  /// return the calibration object
  const boost::shared_ptr<Cameras>& cameraRig() const { return cameraRig_; }

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
    std::cout << s << "SmartProjectionPoseFactorRollingShutter: \n ";
    for (size_t i = 0; i < cameraIds_.size(); i++) {
      std::cout << "-- Measurement nr " << i << std::endl;
      std::cout << " pose1 key: "
                << keyFormatter(world_P_body_key_pairs_[i].first) << std::endl;
      std::cout << " pose2 key: "
                << keyFormatter(world_P_body_key_pairs_[i].second) << std::endl;
      std::cout << " alpha: " << alphas_[i] << std::endl;
      std::cout << "cameraId: " << cameraIds_[i] << std::endl;
      (*cameraRig_)[cameraIds_[i]].print("camera in rig:\n");
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const SmartProjectionPoseFactorRollingShutter<CAMERA>* e =
        dynamic_cast<const SmartProjectionPoseFactorRollingShutter<CAMERA>*>(
            &p);

    double keyPairsEqual = true;
    if (this->world_P_body_key_pairs_.size() ==
        e->world_P_body_key_pairs().size()) {
      for (size_t k = 0; k < this->world_P_body_key_pairs_.size(); k++) {
        const Key key1own = world_P_body_key_pairs_[k].first;
        const Key key1e = e->world_P_body_key_pairs()[k].first;
        const Key key2own = world_P_body_key_pairs_[k].second;
        const Key key2e = e->world_P_body_key_pairs()[k].second;
        if (!(key1own == key1e) || !(key2own == key2e)) {
          keyPairsEqual = false;
          break;
        }
      }
    } else {
      keyPairsEqual = false;
    }

    return e && Base::equals(p, tol) && alphas_ == e->alphas() &&
           keyPairsEqual && cameraRig_->equals(*(e->cameraRig())) &&
           std::equal(cameraIds_.begin(), cameraIds_.end(),
                      e->cameraIds().begin());
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses
   * corresponding to keys involved in this factor
   * @return Cameras
   */
  typename Base::Cameras cameras(const Values& values) const override {
    typename Base::Cameras cameras;
    for (size_t i = 0; i < this->measured_.size();
         i++) {  // for each measurement
      const Pose3& w_P_body1 =
          values.at<Pose3>(world_P_body_key_pairs_[i].first);
      const Pose3& w_P_body2 =
          values.at<Pose3>(world_P_body_key_pairs_[i].second);
      double interpolationFactor = alphas_[i];
      const Pose3& w_P_body =
          interpolate<Pose3>(w_P_body1, w_P_body2, interpolationFactor);
      const typename Base::Camera& camera_i = (*cameraRig_)[cameraIds_[i]];
      const Pose3& body_P_cam = camera_i.pose();
      const Pose3& w_P_cam = w_P_body.compose(body_P_cam);
      cameras.emplace_back(w_P_cam,
                           make_shared<typename CAMERA::CalibrationType>(
                               camera_i.calibration()));
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
   * @return Return arguments are the camera jacobians Fs (including the
   * jacobian with respect to both body poses we interpolate from), the point
   * Jacobian E, and the error vector b. Note that the jacobians are computed
   * for a given point.
   */
  void computeJacobiansWithTriangulatedPoint(FBlocks& Fs, Matrix& E, Vector& b,
                                             const Values& values) const {
    if (!this->result_) {
      throw("computeJacobiansWithTriangulatedPoint");
    } else {  // valid result: compute jacobians
      size_t numViews = this->measured_.size();
      E = Matrix::Zero(2 * numViews,
                       3);  // a Point2 for each view (point jacobian)
      b = Vector::Zero(2 * numViews);  // a Point2 for each view
      // intermediate Jacobians
      Eigen::Matrix<double, ZDim, DimPose> dProject_dPoseCam;
      Eigen::Matrix<double, DimPose, DimPose> dInterpPose_dPoseBody1,
          dInterpPose_dPoseBody2, dPoseCam_dInterpPose;
      Eigen::Matrix<double, ZDim, 3> Ei;

      for (size_t i = 0; i < numViews; i++) {  // for each camera/measurement
        auto w_P_body1 = values.at<Pose3>(world_P_body_key_pairs_[i].first);
        auto w_P_body2 = values.at<Pose3>(world_P_body_key_pairs_[i].second);
        double interpolationFactor = alphas_[i];
        // get interpolated pose:
        auto w_P_body =
            interpolate<Pose3>(w_P_body1, w_P_body2, interpolationFactor,
                               dInterpPose_dPoseBody1, dInterpPose_dPoseBody2);
        const typename Base::Camera& camera_i = (*cameraRig_)[cameraIds_[i]];
        auto body_P_cam = camera_i.pose();
        auto w_P_cam = w_P_body.compose(body_P_cam, dPoseCam_dInterpPose);
        PinholeCamera<CALIBRATION> camera(w_P_cam, camera_i.calibration());

        // get jacobians and error vector for current measurement
        Point2 reprojectionError_i =
            Point2(camera.project(*this->result_, dProject_dPoseCam, Ei) -
                   this->measured_.at(i));
        Eigen::Matrix<double, ZDim, DimBlock> J;  // 2 x 12
        J.block(0, 0, ZDim, 6) =
            dProject_dPoseCam * dPoseCam_dInterpPose *
            dInterpPose_dPoseBody1;  // (2x6) * (6x6) * (6x6)
        J.block(0, 6, ZDim, 6) =
            dProject_dPoseCam * dPoseCam_dInterpPose *
            dInterpPose_dPoseBody2;  // (2x6) * (6x6) * (6x6)

        // fit into the output structures
        Fs.push_back(J);
        size_t row = 2 * i;
        b.segment<ZDim>(row) = -reprojectionError_i;
        E.block<ZDim, 3>(row, 0) = Ei;
      }
    }
  }

  /// linearize and return a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<DimPose>> createHessianFactor(
      const Values& values, const double& lambda = 0.0,
      bool diagonalDamping = false) const {
    // we may have multiple observation sharing the same keys (due to the
    // rolling shutter interpolation), hence the number of unique keys may be
    // smaller than 2 * nrMeasurements
    size_t nrUniqueKeys =
        this->keys_
            .size();  // note: by construction, keys_ only contains unique keys

    typename Base::Cameras cameras = this->cameras(values);

    // Create structures for Hessian Factors
    KeyVector js;
    std::vector<Matrix> Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    std::vector<Vector> gs(nrUniqueKeys);

    if (this->measured_.size() !=
        cameras.size())  // 1 observation per interpolated camera
      throw std::runtime_error(
          "SmartProjectionPoseFactorRollingShutter: "
          "measured_.size() inconsistent with input");

    // triangulate 3D point at given linearization point
    this->triangulateSafe(cameras);

    if (!this->result_) {  // failed: return "empty/zero" Hessian
      if (this->params_.degeneracyMode == ZERO_ON_DEGENERACY) {
        for (Matrix& m : Gs) m = Matrix::Zero(DimPose, DimPose);
        for (Vector& v : gs) v = Vector::Zero(DimPose);
        return boost::make_shared<RegularHessianFactor<DimPose>>(this->keys_,
                                                                 Gs, gs, 0.0);
      } else {
        throw std::runtime_error(
            "SmartProjectionPoseFactorRollingShutter: "
            "only supported degeneracy mode is ZERO_ON_DEGENERACY");
      }
    }
    // compute Jacobian given triangulated 3D Point
    FBlocks Fs;
    Matrix E;
    Vector b;
    this->computeJacobiansWithTriangulatedPoint(Fs, E, b, values);

    // Whiten using noise model
    this->noiseModel_->WhitenSystem(E, b);
    for (size_t i = 0; i < Fs.size(); i++)
      Fs[i] = this->noiseModel_->Whiten(Fs[i]);

    Matrix3 P = Base::Cameras::PointCov(E, lambda, diagonalDamping);

    // Collect all the key pairs: these are the keys that correspond to the
    // blocks in Fs (on which we apply the Schur Complement)
    KeyVector nonuniqueKeys;
    for (size_t i = 0; i < world_P_body_key_pairs_.size(); i++) {
      nonuniqueKeys.push_back(world_P_body_key_pairs_.at(i).first);
      nonuniqueKeys.push_back(world_P_body_key_pairs_.at(i).second);
    }

    // Build augmented Hessian (with last row/column being the information
    // vector) Note: we need to get the augumented hessian wrt the unique keys
    // in key_
    SymmetricBlockMatrix augmentedHessianUniqueKeys =
        Base::Cameras::template SchurComplementAndRearrangeBlocks<3, 12, 6>(
            Fs, E, P, b, nonuniqueKeys, this->keys_);

    return boost::make_shared<RegularHessianFactor<DimPose>>(
        this->keys_, augmentedHessianUniqueKeys);
  }

  /**
   * Linearize to Gaussian Factor (possibly adding a damping factor Lambda for
   * LM)
   * @param values Values structure which must contain camera poses and
   * extrinsic pose for this factor
   * @return a Gaussian factor
   */
  boost::shared_ptr<GaussianFactor> linearizeDamped(
      const Values& values, const double& lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different
    // linear factors
    switch (this->params_.linearizationMode) {
      case HESSIAN:
        return this->createHessianFactor(values, lambda);
      default:
        throw std::runtime_error(
            "SmartProjectionPoseFactorRollingShutter: "
            "unknown linearization mode");
    }
  }

  /// linearize
  boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    return this->linearizeDamped(values);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};
// end of class declaration

/// traits
template <class CAMERA>
struct traits<SmartProjectionPoseFactorRollingShutter<CAMERA>>
    : public Testable<SmartProjectionPoseFactorRollingShutter<CAMERA>> {};

}  // namespace gtsam
