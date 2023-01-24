/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionFactorPP.h
 * @brief  Smart stereo factor on poses (P) and camera extrinsic pose (P) calibrations
 * @author Luca Carlone
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/slam/SmartStereoProjectionFactor.h>

namespace gtsam {
/**
 *
 * @ingroup slam
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert,
 * Eliminating conditionally independent sets in factor graphs:
 * a unifying perspective based on smart factors,
 * Int. Conf. on Robotics and Automation (ICRA), 2014.
 */

/**
 * This factor optimizes the pose of the body as well as the extrinsic camera
 * calibration (pose of camera wrt body). Each camera may have its own extrinsic
 * calibration or the same calibration can be shared by multiple cameras. This
 * factor requires that values contain the involved poses and extrinsics (both
 * are Pose3 variables).
 * @ingroup slam
 */
class GTSAM_UNSTABLE_EXPORT SmartStereoProjectionFactorPP
    : public SmartStereoProjectionFactor {
 protected:
  /// shared pointer to calibration object (one for each camera)
  std::vector<std::shared_ptr<Cal3_S2Stereo>> K_all_;

  /// The keys corresponding to the pose of the body (with respect to an external world frame) for each view
  KeyVector world_P_body_keys_;

  /// The keys corresponding to the extrinsic pose calibration for each view (pose that transform from camera to body)
  KeyVector body_P_cam_keys_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for base class type
  typedef SmartStereoProjectionFactor Base;

  /// shorthand for this class
  typedef SmartStereoProjectionFactorPP This;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  static const int DimBlock = 12;  ///< Camera dimension: 6 for body pose, 6 for extrinsic pose
  static const int DimPose = 6;  ///< Pose3 dimension
  static const int ZDim = 3;  ///< Measurement dimension (for a StereoPoint2 measurement)
  typedef Eigen::Matrix<double, ZDim, DimBlock> MatrixZD;  // F blocks (derivatives wrt camera)
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks;  // vector of F blocks

  /**
   * Constructor
   * @param Isotropic measurement noise
   * @param params internal parameters of the smart factors
   */
  SmartStereoProjectionFactorPP(const SharedNoiseModel& sharedNoiseModel,
                                const SmartStereoProjectionParams& params =
                                    SmartStereoProjectionParams());

  /** Virtual destructor */
  ~SmartStereoProjectionFactorPP() override = default;

  /**
   * add a new measurement, with a pose key, and an extrinsic pose key
   * @param measured is the 3-dimensional location of the projection of a
   * single landmark in the a single (stereo) view (the measurement)
   * @param world_P_body_key is the key corresponding to the body poses observing the same landmark
   * @param body_P_cam_key is the key corresponding to the extrinsic camera-to-body pose calibration
   * @param K is the (fixed) camera intrinsic calibration
   */
  void add(const StereoPoint2& measured, const Key& world_P_body_key,
           const Key& body_P_cam_key,
           const std::shared_ptr<Cal3_S2Stereo>& K);

  /**
   *  Variant of the previous one in which we include a set of measurements
   * @param measurements vector of the 3m dimensional location of the projection
   * of a single landmark in the m (stereo) view (the measurements)
   * @param w_P_body_keys are the ordered keys corresponding to the body poses observing the same landmark
   * @param body_P_cam_keys are the ordered keys corresponding to the extrinsic camera-to-body poses calibration
   * (note: elements of this vector do not need to be unique: 2 camera views can share the same calibration)
   * @param Ks vector of intrinsic calibration objects
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& w_P_body_keys, const KeyVector& body_P_cam_keys,
           const std::vector<std::shared_ptr<Cal3_S2Stereo>>& Ks);

  /**
   * Variant of the previous one in which we include a set of measurements with
   * the same noise and calibration
   * @param measurements vector of the 3m dimensional location of the projection
   * of a single landmark in the m (stereo) view (the measurements)
   * @param w_P_body_keys are the ordered keys corresponding to the body poses observing the same landmark
   * @param body_P_cam_keys are the ordered keys corresponding to the extrinsic camera-to-body poses calibration
   * (note: elements of this vector do not need to be unique: 2 camera views can share the same calibration)
   * @param K the (known) camera calibration (same for all measurements)
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& w_P_body_keys, const KeyVector& body_P_cam_keys,
           const std::shared_ptr<Cal3_S2Stereo>& K);

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                 DefaultKeyFormatter) const override;

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override;

  /// equals
  const KeyVector& getExtrinsicPoseKeys() const {
    return body_P_cam_keys_;
  }

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override;

  /** return the calibration object */
  inline std::vector<std::shared_ptr<Cal3_S2Stereo>> calibration() const {
    return K_all_;
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses
   * corresponding
   * to keys involved in this factor
   * @return vector of Values
   */
  Base::Cameras cameras(const Values& values) const override;

  /**
   * Compute jacobian F, E and error vector at a given linearization point
   * @param values Values structure which must contain camera poses
   * corresponding to keys involved in this factor
   * @return Return arguments are the camera jacobians Fs (including the jacobian with
   * respect to both the body pose and extrinsic pose), the point Jacobian E,
   * and the error vector b. Note that the jacobians are computed for a given point.
   */
  void computeJacobiansAndCorrectForMissingMeasurements(
      FBlocks& Fs, Matrix& E, Vector& b, const Values& values) const {
    if (!result_) {
      throw("computeJacobiansWithTriangulatedPoint");
    } else {  // valid result: compute jacobians
      size_t numViews = measured_.size();
      E = Matrix::Zero(3 * numViews, 3); // a StereoPoint2 for each view (point jacobian)
      b = Vector::Zero(3 * numViews);  // a StereoPoint2 for each view
      Matrix dPoseCam_dPoseBody_i, dPoseCam_dPoseExt_i, dProject_dPoseCam_i, Ei;

      for (size_t i = 0; i < numViews; i++) {  // for each camera/measurement
        Pose3 w_P_body = values.at<Pose3>(world_P_body_keys_.at(i));
        Pose3 body_P_cam = values.at<Pose3>(body_P_cam_keys_.at(i));
        StereoCamera camera(
            w_P_body.compose(body_P_cam, dPoseCam_dPoseBody_i, dPoseCam_dPoseExt_i),
            K_all_[i]);
        // get jacobians and error vector for current measurement
        StereoPoint2 reprojectionError_i = StereoPoint2(
            camera.project(*result_, dProject_dPoseCam_i, Ei) - measured_.at(i));
        Eigen::Matrix<double, ZDim, DimBlock> J;  // 3 x 12
        J.block<ZDim, 6>(0, 0) = dProject_dPoseCam_i * dPoseCam_dPoseBody_i;  // (3x6) * (6x6)
        J.block<ZDim, 6>(0, 6) = dProject_dPoseCam_i * dPoseCam_dPoseExt_i;  // (3x6) * (6x6)
        // if the right pixel is invalid, fix jacobians
        if (std::isnan(measured_.at(i).uR()))
        {
          J.block<1, 12>(1, 0) = Matrix::Zero(1, 12);
          Ei.block<1, 3>(1, 0) = Matrix::Zero(1, 3);
          reprojectionError_i = StereoPoint2(reprojectionError_i.uL(), 0.0,
                                           reprojectionError_i.v());
        }
        // fit into the output structures
        Fs.push_back(J);
        size_t row = 3 * i;
        b.segment<ZDim>(row) = -reprojectionError_i.vector();
        E.block<3, 3>(row, 0) = Ei;
      }
    }
  }

  /// linearize and return a Hessianfactor that is an approximation of error(p)
  std::shared_ptr<RegularHessianFactor<DimPose> > createHessianFactor(
      const Values& values, const double lambda = 0.0, bool diagonalDamping =
          false) const {

    // we may have multiple cameras sharing the same extrinsic cals, hence the number
    // of keys may be smaller than 2 * nrMeasurements (which is the upper bound where we
    // have a body key and an extrinsic calibration key for each measurement)
    size_t nrUniqueKeys = keys_.size();

    // Create structures for Hessian Factors
    KeyVector js;
    std::vector < Matrix > Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    std::vector<Vector> gs(nrUniqueKeys);

    if (this->measured_.size() != cameras(values).size())
      throw std::runtime_error("SmartStereoProjectionHessianFactor: this->"
                               "measured_.size() inconsistent with input");

    // triangulate 3D point at given linearization point
    triangulateSafe(cameras(values));

    if (!result_) { // failed: return "empty/zero" Hessian
      for (Matrix& m : Gs)
        m = Matrix::Zero(DimPose, DimPose);
      for (Vector& v : gs)
        v = Vector::Zero(DimPose);
      return std::make_shared < RegularHessianFactor<DimPose>
          > (keys_, Gs, gs, 0.0);
    }

    // compute Jacobian given triangulated 3D Point
    FBlocks Fs;
    Matrix F, E;
    Vector b;
    computeJacobiansAndCorrectForMissingMeasurements(Fs, E, b, values);

    // Whiten using noise model
    noiseModel_->WhitenSystem(E, b);
    for (size_t i = 0; i < Fs.size(); i++)
      Fs[i] = noiseModel_->Whiten(Fs[i]);

    // build augmented Hessian (with last row/column being the information vector)
    Matrix3 P;
    Cameras::ComputePointCovariance <3> (P, E, lambda, diagonalDamping);

    // these are the keys that correspond to the blocks in augmentedHessian (output of SchurComplement)
    KeyVector nonuniqueKeys;
    for (size_t i = 0; i < world_P_body_keys_.size(); i++) {
      nonuniqueKeys.push_back(world_P_body_keys_.at(i));
      nonuniqueKeys.push_back(body_P_cam_keys_.at(i));
    }
    // but we need to get the augumented hessian wrt the unique keys in key_
    SymmetricBlockMatrix augmentedHessianUniqueKeys =
        Cameras::SchurComplementAndRearrangeBlocks<3,DimBlock,DimPose>(Fs,E,P,b,
                  nonuniqueKeys, keys_);

    return std::make_shared < RegularHessianFactor<DimPose>
        > (keys_, augmentedHessianUniqueKeys);
  }

  /**
   * Linearize to Gaussian Factor (possibly adding a damping factor Lambda for LM)
   * @param values Values structure which must contain camera poses and extrinsic pose for this factor
   * @return a Gaussian factor
   */
  std::shared_ptr<GaussianFactor> linearizeDamped(
      const Values& values, const double lambda = 0.0) const {
    // depending on flag set on construction we may linearize to different linear factors
    switch (params_.linearizationMode) {
      case HESSIAN:
        return createHessianFactor(values, lambda);
      default:
        throw std::runtime_error(
            "SmartStereoProjectionFactorPP: unknown linearization mode");
    }
  }

  /// linearize
  std::shared_ptr<GaussianFactor> linearize(const Values& values) const
      override {
    return linearizeDamped(values);
  }

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_all_);
  }
#endif
};
// end of class declaration

/// traits
template<>
struct traits<SmartStereoProjectionFactorPP> : public Testable<
    SmartStereoProjectionFactorPP> {
};

}  // namespace gtsam
