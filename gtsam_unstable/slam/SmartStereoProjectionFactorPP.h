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
  void computeJacobiansWithTriangulatedPoint(
      FBlocks& Fs,
      Matrix& E, Vector& b, const Values& values) const {
    if (!result_) {
      throw ("computeJacobiansWithTriangulatedPoint");
    } else {
      // valid result: compute jacobians
      Matrix H0,H1,H3,H02;
      for (size_t i = 0; i < keys_.size(); i++) { // for each camera/measurement
        Pose3 w_P_body = values.at<Pose3>(keys_.at(i));
        Pose3 body_P_cam = values.at<Pose3>(body_P_cam_keys_.at(i));
        StereoCamera camera(w_P_body.compose(body_P_cam, H0, H02), K_all_[i]);
        StereoPoint2 reprojectionError = StereoPoint2(camera.project(*result_, H1, H3) - measured_.at(i));
        std::cout << "H0 \n" << H0 << std::endl;
        std::cout << "H1 \n" << H1 << std::endl;
        std::cout << "H3 \n" << H3 << std::endl;
        std::cout << "H02 \n" << H02 << std::endl;
        Eigen::Matrix<double, ZDim, Dim> J; // 3 x 12
        std::cout << "H1 * H0 \n" << H1 * H0 << std::endl;
        std::cout << "H1 * H02 \n" << H1 * H02 << std::endl;
        J.block<ZDim,6>(0,0) = H1 * H0; // (3x6) * (6x6)
        J.block<ZDim,6>(0,6) = H1 * H02; // (3x6) * (6x6)
        std::cout << "J \n" << J << std::endl;
        Fs.push_back(J);
      }
    }
  }

  /// linearize returns a Hessianfactor that is an approximation of error(p)
  boost::shared_ptr<RegularHessianFactor<Dim> > createHessianFactor(
      const Values& values, const double lambda = 0.0,  bool diagonalDamping =
          false) const {

    size_t numKeys = this->keys_.size();
    // Create structures for Hessian Factors
    KeyVector js;
    std::vector<Matrix> Gs(numKeys * (numKeys + 1) / 2);
    std::vector<Vector> gs(numKeys);

    std::cout <<"using my hessian!!!!!!!!!!1" << std::endl;

    if (this->measured_.size() != cameras(values).size())
      throw std::runtime_error("SmartStereoProjectionHessianFactor: this->"
          "measured_.size() inconsistent with input");

    triangulateSafe(cameras(values));
    std::cout <<"passed triangulateSafe!!!!!!!!!!1" << std::endl;

    if (params_.degeneracyMode == ZERO_ON_DEGENERACY && !result_) {
      // failed: return"empty" Hessian
      for(Matrix& m: Gs)
        m = Matrix::Zero(Dim,Dim);
      for(Vector& v: gs)
        v = Vector::Zero(Dim);
      return boost::make_shared<RegularHessianFactor<Dim> >(this->keys_,
                                                                  Gs, gs, 0.0);
    }

    // Jacobian could be 3D Point3 OR 2D Unit3, difference is E.cols().
    FBlocks Fs;
    Matrix F, E;
    Vector b;
    std::cout <<"before computeJacobiansWithTriangulatedPoint!!!!!!!!!!1" << std::endl;
    computeJacobiansWithTriangulatedPoint(Fs, E, b, values);
    std::cout << "Fs.at(0) \n"<< Fs.at(0) << std::endl;

//    // Whiten using noise model
//    Base::whitenJacobians(Fs, E, b);
//
//    // build augmented hessian
//    SymmetricBlockMatrix augmentedHessian = //
//        Cameras::SchurComplement(Fs, E, b, lambda, diagonalDamping);
//
//    return boost::make_shared<RegularHessianFactor<Dim> >(this->keys_,
//                                                                augmentedHessian);
    return boost::make_shared<RegularHessianFactor<Dim> >(this->keys_,
                                                                      Gs, gs, 0.0);
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
