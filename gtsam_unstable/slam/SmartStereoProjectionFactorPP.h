/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionFactorPP.h
 * @brief  Smart stereo factor on poses and extrinsic calibration
 * @author Luca Carlone
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam_unstable/slam/SmartStereoProjectionFactor.h>

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
 * This factor optimizes the extrinsic camera calibration (pose of camera wrt body),
 * and each camera has its own extrinsic calibration.
 * This factor requires that values contain the involved poses and extrinsics (both Pose3).
 * @addtogroup SLAM
 */
class SmartStereoProjectionFactorPP : public SmartStereoProjectionFactor {
 protected:
  /// shared pointer to calibration object (one for each camera)
  std::vector<boost::shared_ptr<Cal3_S2Stereo>> K_all_;

  /// The keys corresponding to the pose of the body for each view
  KeyVector w_P_body_keys_;

  /// The keys corresponding to the extrinsic pose calibration for each view
  KeyVector body_P_cam_keys_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for base class type
  typedef SmartStereoProjectionFactor Base;

  /// shorthand for this class
  typedef SmartStereoProjectionFactorPP This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  static const int Dim = 12; ///< Camera dimension
  static const int DimPose = 6; ///< Camera dimension
  static const int ZDim = 3; ///< Measurement dimension
  typedef Eigen::Matrix<double, ZDim, Dim> MatrixZD; // F blocks (derivatives wrpt camera)
  typedef std::vector<MatrixZD, Eigen::aligned_allocator<MatrixZD> > FBlocks; // vector of F blocks

  /**
   * Constructor
   * @param Isotropic measurement noise
   * @param params internal parameters of the smart factors
   */
  SmartStereoProjectionFactorPP(
      const SharedNoiseModel& sharedNoiseModel,
      const SmartStereoProjectionParams& params = SmartStereoProjectionParams());

  /** Virtual destructor */
  ~SmartStereoProjectionFactorPP() override = default;

  /**
   * add a new measurement, with a pose key, and an extrinsic pose key
   * @param measured is the 3-dimensional location of the projection of a
   * single landmark in the a single view (the measurement)
   * @param w_P_body_key is key corresponding to the camera observing the same landmark
   * @param body_P_cam_key is key corresponding to the camera observing the same landmark
   * @param K is the (fixed) camera calibration
   */
  void add(const StereoPoint2& measured,
           const Key& w_P_body_key, const Key& body_P_cam_key,
           const boost::shared_ptr<Cal3_S2Stereo>& K);

  /**
   *  Variant of the previous one in which we include a set of measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m view (the measurements)
   * @param w_P_body_keys are the ordered keys corresponding to the camera observing the same landmark
   * @param body_P_cam_keys are the ordered keys corresponding to the camera observing the same landmark
   * @param Ks vector of calibration objects
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& w_P_body_keys, const KeyVector& body_P_cam_keys,
           const std::vector<boost::shared_ptr<Cal3_S2Stereo>>& Ks);

  /**
   * Variant of the previous one in which we include a set of measurements with
   * the same noise and calibration
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m view (the measurement)
   * @param w_P_body_keys are the ordered keys corresponding to the camera observing the same landmark
   * @param body_P_cam_keys are the ordered keys corresponding to the camera observing the same landmark
   * @param K the (known) camera calibration (same for all measurements)
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& w_P_body_keys, const KeyVector& body_P_cam_keys,
           const boost::shared_ptr<Cal3_S2Stereo>& K);

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
  const KeyVector& getExtrinsicPoseKeys() const {return body_P_cam_keys_;};

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override;

  /** return the calibration object */
  inline std::vector<boost::shared_ptr<Cal3_S2Stereo>> calibration() const {
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

  /// Compute F, E only (called below in both vanilla and SVD versions)
  /// Assumes the point has been computed
  /// Note E can be 2m*3 or 2m*2, in case point is degenerate
  void computeJacobiansAndCorrectForMissingMeasurements(
      FBlocks& Fs,
      Matrix& E, Vector& b, const Values& values) const {
    if (!result_) {
      throw ("computeJacobiansWithTriangulatedPoint");
    } else { // valid result: compute jacobians
      size_t numViews = measured_.size();
      E = Matrix::Zero(3*numViews,3); // a StereoPoint2 for each view
      b = Vector::Zero(3*numViews); // a StereoPoint2 for each view
      Matrix dPoseCam_dPoseBody,dPoseCam_dPoseExt, dProject_dPoseCam,Ei;

      for (size_t i = 0; i < numViews; i++) { // for each camera/measurement
        Pose3 w_P_body = values.at<Pose3>(w_P_body_keys_.at(i));
        Pose3 body_P_cam = values.at<Pose3>(body_P_cam_keys_.at(i));
        StereoCamera camera(w_P_body.compose(body_P_cam, dPoseCam_dPoseBody, dPoseCam_dPoseExt), K_all_[i]);
        StereoPoint2 reprojectionError = StereoPoint2(camera.project(*result_, dProject_dPoseCam, Ei) - measured_.at(i));
        Eigen::Matrix<double, ZDim, Dim> J; // 3 x 12
        J.block<ZDim,6>(0,0) = dProject_dPoseCam * dPoseCam_dPoseBody; // (3x6) * (6x6)
        J.block<ZDim,6>(0,6) = dProject_dPoseCam * dPoseCam_dPoseExt; // (3x6) * (6x6)
        if(std::isnan(measured_.at(i).uR())) // if the right pixel is invalid
        {
          J.block<1,12>(1,0) = Matrix::Zero(1,12);
          Ei.block<1,3>(1,0) = Matrix::Zero(1,3);
          reprojectionError = StereoPoint2(reprojectionError.uL(), 0.0, reprojectionError.v() );
        }
        Fs.push_back(J);
        size_t row = 3*i;
        b.segment<ZDim>(row) = - reprojectionError.vector();
        E.block<3,3>(row,0) = Ei;
      }
    }
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<DimPose> > createHessianFactor(
      const Values& values, const double lambda = 0.0,  bool diagonalDamping =
          false) const {

    size_t nrUniqueKeys = keys_.size();

    // Create structures for Hessian Factors
    KeyVector js;
    std::vector<Matrix> Gs(nrUniqueKeys * (nrUniqueKeys + 1) / 2);
    std::vector<Vector> gs(nrUniqueKeys);

    if (this->measured_.size() != cameras(values).size())
      throw std::runtime_error("SmartStereoProjectionHessianFactor: this->"
          "measured_.size() inconsistent with input");

    triangulateSafe(cameras(values));

    if (!result_) {
      // failed: return"empty" Hessian
      for(Matrix& m: Gs)
        m = Matrix::Zero(DimPose,DimPose);
      for(Vector& v: gs)
        v = Vector::Zero(DimPose);
      return boost::make_shared<RegularHessianFactor<DimPose> >(keys_,
                                                                  Gs, gs, 0.0);
    }

    // Jacobian could be 3D Point3 OR 2D Unit3, difference is E.cols().
    FBlocks Fs;
    Matrix F, E;
    Vector b;
    computeJacobiansAndCorrectForMissingMeasurements(Fs, E, b, values);

    // Whiten using noise model
    noiseModel_->WhitenSystem(E, b);
    for (size_t i = 0; i < Fs.size(); i++)
      Fs[i] = noiseModel_->Whiten(Fs[i]);

    // build augmented hessian
    Matrix3 P;
    Cameras::ComputePointCovariance<3>(P, E, lambda, diagonalDamping);

    // marginalize point
    SymmetricBlockMatrix augmentedHessian = //
        Cameras::SchurComplement<3,Dim>(Fs, E, P, b);

    // now pack into an Hessian factor
    std::vector<DenseIndex> dims(nrUniqueKeys + 1); // this also includes the b term
    std::fill(dims.begin(), dims.end() - 1, 6);
    dims.back() = 1;

    size_t nrNonuniqueKeys = w_P_body_keys_.size() + body_P_cam_keys_.size();
    SymmetricBlockMatrix augmentedHessianUniqueKeys;
    if ( nrUniqueKeys == nrNonuniqueKeys ){ // if there is 1 calibration key per camera
      augmentedHessianUniqueKeys = SymmetricBlockMatrix(dims, Matrix(augmentedHessian.selfadjointView()));
    }else{ // if multiple cameras share a calibration
      std::vector<DenseIndex> nonuniqueDims(nrNonuniqueKeys + 1); // this also includes the b term
      std::fill(nonuniqueDims.begin(), nonuniqueDims.end() - 1, 6);
      nonuniqueDims.back() = 1;
      augmentedHessian = SymmetricBlockMatrix(nonuniqueDims, Matrix(augmentedHessian.selfadjointView()));

      // these are the keys that correspond to the blocks in augmentedHessian (output of SchurComplement)
      KeyVector nonuniqueKeys;
      for(size_t i=0; i < w_P_body_keys_.size();i++){
        nonuniqueKeys.push_back(w_P_body_keys_.at(i));
        nonuniqueKeys.push_back(body_P_cam_keys_.at(i));
      }

      // get map from key to location in the new augmented Hessian matrix (the one including only unique keys)
      std::map<Key,size_t> keyToSlotMap;
      for(size_t k=0; k<nrUniqueKeys;k++){
        keyToSlotMap[keys_[k]] = k;
      }

      // initialize matrix to zero
      augmentedHessianUniqueKeys = SymmetricBlockMatrix(dims, Matrix::Zero(6*nrUniqueKeys+1,6*nrUniqueKeys+1));

      // add contributions for each key: note this loops over the hessian with nonUnique keys (augmentedHessian)
      for(size_t i=0; i<nrNonuniqueKeys;i++){ // rows
        Key key_i = nonuniqueKeys.at(i);

        // update information vector
        augmentedHessianUniqueKeys.updateOffDiagonalBlock( keyToSlotMap[key_i] , nrUniqueKeys,
                                                               augmentedHessian.aboveDiagonalBlock(i,nrNonuniqueKeys));

        // update blocks
        for(size_t j=i; j<nrNonuniqueKeys;j++){ // cols
          Key key_j = nonuniqueKeys.at(j);
          if(i==j){
            augmentedHessianUniqueKeys.updateDiagonalBlock( keyToSlotMap[key_i] , augmentedHessian.diagonalBlock(i));
          }else if(i < j){
            if( keyToSlotMap[key_i] != keyToSlotMap[key_j] ){
              augmentedHessianUniqueKeys.updateOffDiagonalBlock( keyToSlotMap[key_i] , keyToSlotMap[key_j],
                                                                 augmentedHessian.aboveDiagonalBlock(i,j));
            }else{
              augmentedHessianUniqueKeys.updateDiagonalBlock( keyToSlotMap[key_i] ,
                                                              augmentedHessian.aboveDiagonalBlock(i,j) +
                                                              augmentedHessian.aboveDiagonalBlock(i,j).transpose());
            }
          }
          else{ //TODO: remove else
            augmentedHessianUniqueKeys.updateOffDiagonalBlock( keyToSlotMap[key_i] , keyToSlotMap[key_j],
                                                       augmentedHessian.aboveDiagonalBlock(j,i).transpose());
          }
        }
      }
      augmentedHessianUniqueKeys.updateDiagonalBlock(nrUniqueKeys, augmentedHessian.diagonalBlock(nrNonuniqueKeys));
    }

    return boost::make_shared<RegularHessianFactor<DimPose> >(keys_, augmentedHessianUniqueKeys);
  }

  /**
    * Linearize to Gaussian Factor
    * @param values Values structure which must contain camera poses and extrinsic pose for this factor
    * @return a Gaussian factor
    */
   boost::shared_ptr<GaussianFactor> linearizeDamped(const Values& values,
       const double lambda = 0.0) const {
     // depending on flag set on construction we may linearize to different linear factors
     switch (params_.linearizationMode) {
     case HESSIAN:
       return createHessianFactor(values, lambda);
     default:
       throw std::runtime_error("SmartStereoFactorlinearize: unknown mode");
     }
   }

   /// linearize
   boost::shared_ptr<GaussianFactor> linearize(
       const Values& values) const override {
     return linearizeDamped(values);
   }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(K_all_);
  }

};  // end of class declaration

/// traits
template <>
struct traits<SmartStereoProjectionFactorPP>
    : public Testable<SmartStereoProjectionFactorPP> {};

}  // namespace gtsam
