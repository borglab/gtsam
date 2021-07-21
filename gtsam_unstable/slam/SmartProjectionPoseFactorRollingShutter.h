/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionPoseFactorRollingShutter.h
 * @brief  Smart projection factor on poses modeling rolling shutter effect with given readout time
 * @author Luca Carlone
 */

#pragma once

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
 * This factor optimizes the pose of the body assuming a rolling shutter model of the camera with given readout time.
 * This factor requires that values contain (for each pixel observation) consecutive camera poses
 * from which the pixel observation pose can be interpolated.
 * @addtogroup SLAM
 */
template<class CALIBRATION>
class SmartProjectionPoseFactorRollingShutter : public SmartProjectionFactor<
    PinholePose<CALIBRATION> > {

 protected:
  /// shared pointer to calibration object (one for each observation)
  std::vector<boost::shared_ptr<CALIBRATION> > K_all_;

  /// The keys of the pose of the body (with respect to an external world frame): two consecutive poses for each observation
  std::vector<std::pair<Key, Key>> world_P_body_key_pairs_;

  /// interpolation factor (one for each observation) to interpolate between pair of consecutive poses
  std::vector<double> gammas_;

  /// Pose of the camera in the body frame
  std::vector<Pose3> body_P_sensors_;  ///< Pose of the camera in the body frame

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef PinholePose<CALIBRATION> Camera;
  /// shorthand for base class type
  typedef SmartProjectionFactor<Camera> Base;

  /// shorthand for this class
  typedef SmartProjectionPoseFactorRollingShutter This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  static const int DimBlock = 12;  ///< size of the variable stacking 2 poses from which the observation pose is interpolated
  static const int DimPose = 6;  ///< Pose3 dimension
  static const int ZDim = 2;  ///< Measurement dimension (Point2)
  typedef Eigen::Matrix<double, ZDim, DimBlock> MatrixZD;  // F blocks (derivatives wrt camera)
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks;  // vector of F blocks

  /**
   * Constructor
   * @param Isotropic measurement noise
   * @param params internal parameters of the smart factors
   */
  SmartProjectionPoseFactorRollingShutter(
      const SharedNoiseModel& sharedNoiseModel,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel, params) {
  }

  /** Virtual destructor */
  ~SmartProjectionPoseFactorRollingShutter() override = default;

  /**
   * add a new measurement, with 2 pose keys, camera calibration, and observed pixel.
   * @param measured is the 2-dimensional location of the projection of a
   * single landmark in the a single view (the measurement), interpolated from the 2 poses
   * @param world_P_body_key1 is the key corresponding to the first body poses (time <= time pixel is acquired)
   * @param world_P_body_key2 is the key corresponding to the second body poses (time >= time pixel is acquired)
   * @param gamma in [0,1] is the interpolation factor, such that if gamma = 0 the interpolated pose is the same as world_P_body_key
   * @param K is the (fixed) camera intrinsic calibration
   */
  void add(const Point2& measured, const Key& world_P_body_key1,
           const Key& world_P_body_key2, const double& gamma,
           const boost::shared_ptr<CALIBRATION>& K, const Pose3 body_P_sensor) {
    // store measurements in base class (note: we only store the first key there)
    Base::add(measured, world_P_body_key1);
    // but we also store the extrinsic calibration keys in the same order
    world_P_body_key_pairs_.push_back(
        std::make_pair(world_P_body_key1, world_P_body_key2));

    // pose keys are assumed to be unique, so we avoid duplicates here
    if (std::find(this->keys_.begin(), this->keys_.end(), world_P_body_key1)
        == this->keys_.end())
      this->keys_.push_back(world_P_body_key1);  // add only unique keys
    if (std::find(this->keys_.begin(), this->keys_.end(), world_P_body_key2)
        == this->keys_.end())
      this->keys_.push_back(world_P_body_key2);  // add only unique keys

    // store fixed calibration
    K_all_.push_back(K);

    // store extrinsics of the camera
    body_P_sensors_.push_back(body_P_sensor);
  }

  /**
   * Variant of the previous one in which we include a set of measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m views (the measurements)
   * @param world_P_body_key_pairs vector of  (1 for each view) containing the pair of poses from which each view can be interpolated
   * @param Ks vector of intrinsic calibration objects
   */
  void add(const std::vector<Point2>& measurements,
           const std::vector<std::pair<Key, Key>>& world_P_body_key_pairs,
           const std::vector<double>& gammas,
           const std::vector<boost::shared_ptr<CALIBRATION>>& Ks,
           const std::vector<Pose3> body_P_sensors) {
    assert(world_P_body_key_pairs.size() == measurements.size());
    assert(world_P_body_key_pairs.size() == gammas.size());
    assert(world_P_body_key_pairs.size() == Ks.size());
    for (size_t i = 0; i < measurements.size(); i++) {
      add(measurements[i], world_P_body_key_pairs[i].first,
          world_P_body_key_pairs[i].second, gammas[i], Ks[i],
          body_P_sensors[i]);
    }
  }

  /**
   * Variant of the previous one in which we include a set of measurements with
   * the same (intrinsic and extrinsic) calibration
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m views (the measurements)
   * @param world_P_body_key_pairs vector of  (1 for each view) containing the pair of poses from which each view can be interpolated
   * @param K the (known) camera calibration (same for all measurements)
   */
  void add(const std::vector<Point2>& measurements,
           const std::vector<std::pair<Key, Key>>& world_P_body_key_pairs,
           const std::vector<double>& gammas,
           const boost::shared_ptr<CALIBRATION>& K, const Pose3 body_P_sensor) {
    assert(world_P_body_key_pairs.size() == measurements.size());
    assert(world_P_body_key_pairs.size() == gammas.size());
    for (size_t i = 0; i < measurements.size(); i++) {
      add(measurements[i], world_P_body_key_pairs[i].first,
          world_P_body_key_pairs[i].second, gammas[i], K, body_P_sensor);
    }
  }

  /// return the calibration object
  inline std::vector<boost::shared_ptr<CALIBRATION>> calibration() const {
    return K_all_;
  }

  /// return (for each observation) the key of the pair of poses from which we interpolate
  const std::vector<std::pair<Key, Key>> world_P_body_key_pairs() const {
    return world_P_body_key_pairs_;
  }

  /// return the interpolation factors gammas
  const std::vector<double> getGammas() const {
    return gammas_;
  }

  /// return the extrinsic camera calibration body_P_sensors
  const std::vector<Pose3> body_P_sensors() const {
    return body_P_sensors_;
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override {
    std::cout << s << "SmartProjectionPoseFactorRollingShutter: \n ";
    for (size_t i = 0; i < K_all_.size(); i++) {
      std::cout << "-- Measurement nr " << i << std::endl;
      std::cout << " pose1 key: "
          << keyFormatter(world_P_body_key_pairs_[i].first) << std::endl;
      std::cout << " pose2 key: "
          << keyFormatter(world_P_body_key_pairs_[i].second) << std::endl;
      std::cout << " gamma: " << gammas_[i] << std::endl;
      body_P_sensors_[i].print("extrinsic calibration:\n");
      K_all_[i]->print("intrinsic calibration = ");
    }
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const SmartProjectionPoseFactorRollingShutter<CALIBRATION>* e =
        dynamic_cast<const SmartProjectionPoseFactorRollingShutter<CALIBRATION>*>(&p);

    double keyPairsEqual = true;
    if(this->world_P_body_key_pairs_.size() == e->world_P_body_key_pairs().size()){
      for(size_t k=0; k< this->world_P_body_key_pairs_.size(); k++){
        const Key key1own = world_P_body_key_pairs_[k].first;
        const Key key1e = e->world_P_body_key_pairs()[k].first;
        const Key key2own = world_P_body_key_pairs_[k].second;
        const Key key2e = e->world_P_body_key_pairs()[k].second;
        if ( !(key1own == key1e) || !(key2own == key2e) ){
          keyPairsEqual = false; break;
        }
      }
    }else{ keyPairsEqual = false; }

    double extrinsicCalibrationEqual = true;
    if(this->body_P_sensors_.size() == e->body_P_sensors().size()){
      for(size_t i=0; i< this->body_P_sensors_.size(); i++){
        if (!body_P_sensors_[i].equals(e->body_P_sensors()[i])){
          extrinsicCalibrationEqual = false; break;
        }
      }
    }else{ extrinsicCalibrationEqual = false; }

    return e && Base::equals(p, tol) && K_all_ == e->calibration()
        && gammas_ == e->getGammas() && keyPairsEqual && extrinsicCalibrationEqual;
  }

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override {
    if (this->active(values)) {
      return this->totalReprojectionError(cameras(values));
    } else {  // else of active flag
      return 0.0;
    }
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses
   * corresponding to keys involved in this factor
   * @return Cameras
   */
  typename Base::Cameras cameras(const Values& values) const override {
    assert(world_P_body_keys_.size() == K_all_.size());
    assert(world_P_body_keys_.size() == body_P_cam_keys_.size());
    typename Base::Cameras cameras;
    for (size_t i = 0; i < world_P_body_key_pairs_.size(); i++) {
      Pose3 w_P_body1 = values.at<Pose3>(world_P_body_key_pairs_[i].first);
      Pose3 w_P_body2 = values.at<Pose3>(world_P_body_key_pairs_[i].second);
      double interpolationFactor = gammas_[i];
      // get interpolated pose:
      Pose3 w_P_body = w_P_body1.interpolateRt(w_P_body2, interpolationFactor);
      Pose3 body_P_cam = body_P_sensors_[i];
      Pose3 w_P_cam = w_P_body.compose(body_P_cam);
      cameras.emplace_back(w_P_cam, K_all_[i]);
    }
    return cameras;
  }

  /**
   * Compute jacobian F, E and error vector at a given linearization point
   * @param values Values structure which must contain camera poses
   * corresponding to keys involved in this factor
   * @return Return arguments are the camera jacobians Fs (including the jacobian with
   * respect to both the body pose and extrinsic pose), the point Jacobian E,
   * and the error vector b. Note that the jacobians are computed for a given point.
   */
  void computeJacobiansWithTriangulatedPoint(FBlocks& Fs, Matrix& E, Vector& b,
                                             const Values& values) const {
    if (!this->result_) {
      throw("computeJacobiansWithTriangulatedPoint");
    } else {  // valid result: compute jacobians
      size_t numViews = this->measured_.size();
      E = Matrix::Zero(2 * numViews, 3);  // a Point2 for each view (point jacobian)
      b = Vector::Zero(2 * numViews);  // a Point2 for each view
      Eigen::Matrix<double, ZDim, DimPose> dProject_dPoseCam;
      Eigen::Matrix<double, DimPose, DimPose> dInterpPose_dPoseBody1,
          dInterpPose_dPoseBody2, dPoseCam_dInterpPose;
      Eigen::Matrix<double, ZDim, 3> Ei;

      for (size_t i = 0; i < numViews; i++) {  // for each camera/measurement
        Pose3 w_P_body1 = values.at<Pose3>(world_P_body_key_pairs_[i].first);
        Pose3 w_P_body2 = values.at<Pose3>(world_P_body_key_pairs_[i].second);
        double interpolationFactor = gammas_[i];
        // get interpolated pose:
        std::cout << "TODO: need to add proper interpolation and Jacobians here" << std::endl;
        Pose3 w_P_body = w_P_body1.interpolateRt(w_P_body2,
                                                 interpolationFactor); /*dInterpPose_dPoseBody1, dInterpPose_dPoseBody2  */
        Pose3 body_P_cam = body_P_sensors_[i];
        Pose3 w_P_cam = w_P_body.compose(body_P_cam, dPoseCam_dInterpPose);
        PinholeCamera<CALIBRATION> camera(w_P_cam, K_all_[i]);

        // get jacobians and error vector for current measurement
        Point2 reprojectionError_i = Point2(
            camera.project(*this->result_, dProject_dPoseCam, Ei)
                - this->measured_.at(i));
        Eigen::Matrix<double, ZDim, DimBlock> J;  // 2 x 12
        J.block<ZDim, 6>(0, 0) = dProject_dPoseCam * dPoseCam_dInterpPose
            * dInterpPose_dPoseBody1;  // (2x6) * (6x6) * (6x6)
        J.block<ZDim, 6>(0, 6) = dProject_dPoseCam * dPoseCam_dInterpPose
            * dInterpPose_dPoseBody2;  // (2x6) * (6x6) * (6x6)

        // fit into the output structures
        Fs.push_back(J);
        size_t row = 2 * i;
        b.segment<ZDim>(row) = -reprojectionError_i;
        E.block<3, 3>(row, 0) = Ei;
      }
    }
  }

  /// linearize and return a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<DimBlock> > createHessianFactor(
      const Values& values, const double lambda = 0.0, bool diagonalDamping =
          false) const {

    // we may have multiple cameras sharing the same extrinsic cals, hence the number
    // of keys may be smaller than 2 * nrMeasurements (which is the upper bound where we
    // have a body key and an extrinsic calibration key for each measurement)
    size_t nrUniqueKeys =  this->keys_.size();
    size_t nrNonuniqueKeys = 2*world_P_body_key_pairs_.size();

    // Create structures for Hessian Factors
    KeyVector js;
    std::vector < Matrix > Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    std::vector<Vector> gs(nrUniqueKeys);

    if (this->measured_.size() != cameras(values).size())
      throw std::runtime_error("SmartProjectionPoseFactorRollingShutter: "
                               "measured_.size() inconsistent with input");

//    // triangulate 3D point at given linearization point
//    triangulateSafe(cameras(values));
//
//    if (!this->result_) { // failed: return "empty/zero" Hessian
//      for (Matrix& m : Gs)
//        m = Matrix::Zero(DimPose, DimPose);
//      for (Vector& v : gs)
//        v = Vector::Zero(DimPose);
//      return boost::make_shared < RegularHessianFactor<DimPose>
//          > ( this->keys_, Gs, gs, 0.0);
//    }
//
//    // compute Jacobian given triangulated 3D Point
//    FBlocks Fs;
//    Matrix F, E;
//    Vector b;
//    computeJacobiansWithTriangulatedPoint(Fs, E, b, values);
//
//    // Whiten using noise model
//    this->noiseModel_->WhitenSystem(E, b);
//    for (size_t i = 0; i < Fs.size(); i++)
//      Fs[i] = this->noiseModel_->Whiten(Fs[i]);
//
//    // build augmented Hessian (with last row/column being the information vector)
//    Matrix3 P;
//    This::Cameras::ComputePointCovariance<3>(P, E, lambda, diagonalDamping);
//
//    // marginalize point: note - we reuse the standard SchurComplement function
//    SymmetricBlockMatrix augmentedHessian = This::Cameras::SchurComplement<2,DimBlock>(Fs, E, P, b);

//    // now pack into an Hessian factor
//    std::vector<DenseIndex> dims(nrUniqueKeys + 1);  // this also includes the b term
//    std::fill(dims.begin(), dims.end() - 1, 6);
//    dims.back() = 1;
//    SymmetricBlockMatrix augmentedHessianUniqueKeys;
//
//    // here we have to deal with the fact that some cameras may share the same extrinsic key
//    if (nrUniqueKeys == nrNonuniqueKeys) {  // if there is 1 calibration key per camera
//      augmentedHessianUniqueKeys = SymmetricBlockMatrix(
//          dims, Matrix(augmentedHessian.selfadjointView()));
//    } else {  // if multiple cameras share a calibration we have to rearrange
//      // the results of the Schur complement matrix
//      std::vector<DenseIndex> nonuniqueDims(nrNonuniqueKeys + 1);  // this also includes the b term
//      std::fill(nonuniqueDims.begin(), nonuniqueDims.end() - 1, 6);
//      nonuniqueDims.back() = 1;
//      augmentedHessian = SymmetricBlockMatrix(
//          nonuniqueDims, Matrix(augmentedHessian.selfadjointView()));
//
//      // these are the keys that correspond to the blocks in augmentedHessian (output of SchurComplement)
//      KeyVector nonuniqueKeys;
//      for (size_t i = 0; i < world_P_body_key_pairs_.size(); i++) {
//        nonuniqueKeys.push_back(world_P_body_key_pairs_.at(i));
//        nonuniqueKeys.push_back(body_P_cam_ this->keys_.at(i));
//      }
//
//      // get map from key to location in the new augmented Hessian matrix (the one including only unique keys)
//      std::map<Key, size_t> keyToSlotMap;
//      for (size_t k = 0; k < nrUniqueKeys; k++) {
//        keyToSlotMap[ this->keys_[k]] = k;
//      }
//
//      // initialize matrix to zero
//      augmentedHessianUniqueKeys = SymmetricBlockMatrix(
//          dims, Matrix::Zero(6 * nrUniqueKeys + 1, 6 * nrUniqueKeys + 1));
//
//      // add contributions for each key: note this loops over the hessian with nonUnique keys (augmentedHessian)
//      // and populates an Hessian that only includes the unique keys (that is what we want to return)
//      for (size_t i = 0; i < nrNonuniqueKeys; i++) {  // rows
//        Key key_i = nonuniqueKeys.at(i);
//
//        // update information vector
//        augmentedHessianUniqueKeys.updateOffDiagonalBlock(
//            keyToSlotMap[key_i], nrUniqueKeys,
//            augmentedHessian.aboveDiagonalBlock(i, nrNonuniqueKeys));
//
//        // update blocks
//        for (size_t j = i; j < nrNonuniqueKeys; j++) {  // cols
//          Key key_j = nonuniqueKeys.at(j);
//          if (i == j) {
//            augmentedHessianUniqueKeys.updateDiagonalBlock(
//                keyToSlotMap[key_i], augmentedHessian.diagonalBlock(i));
//          } else {  // (i < j)
//            if (keyToSlotMap[key_i] != keyToSlotMap[key_j]) {
//              augmentedHessianUniqueKeys.updateOffDiagonalBlock(
//                  keyToSlotMap[key_i], keyToSlotMap[key_j],
//                  augmentedHessian.aboveDiagonalBlock(i, j));
//            } else {
//              augmentedHessianUniqueKeys.updateDiagonalBlock(
//                  keyToSlotMap[key_i],
//                  augmentedHessian.aboveDiagonalBlock(i, j)
//                      + augmentedHessian.aboveDiagonalBlock(i, j).transpose());
//            }
//          }
//        }
//      }
//      // update bottom right element of the matrix
//      augmentedHessianUniqueKeys.updateDiagonalBlock(
//          nrUniqueKeys, augmentedHessian.diagonalBlock(nrNonuniqueKeys));
//    }
//    return boost::make_shared < RegularHessianFactor<DimPose>
//        > ( this->keys_, augmentedHessianUniqueKeys);
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
        return createHessianFactor(values, lambda);
      default:
        throw std::runtime_error(
            "SmartProjectionPoseFactorRollingShutter: unknown linearization mode");
    }
  }

  /// linearize
  boost::shared_ptr<GaussianFactor> linearize(const Values& values) const
      override {
    return linearizeDamped(values);
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_all_);
  }

};
// end of class declaration

/// traits
template<class CALIBRATION>
struct traits<SmartProjectionPoseFactorRollingShutter<CALIBRATION> > :
    public Testable<SmartProjectionPoseFactorRollingShutter<CALIBRATION> > {
};

}  // namespace gtsam
