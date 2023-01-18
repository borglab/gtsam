/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionPoseFactor.h
 * @brief  Smart stereo factor on poses, assuming camera calibration is fixed
 * @author Luca Carlone
 * @author Antoni Rosinol
 * @author Chris Beall
 * @author Zsolt Kira
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
 *
 */

/**
 * This factor assumes that camera calibration is fixed, but each camera
 * has its own calibration.
 * The factor only constrains poses (variable dimension is 6).
 * This factor requires that values contains the involved poses (Pose3).
 * @ingroup slam
 */
class GTSAM_UNSTABLE_EXPORT SmartStereoProjectionPoseFactor
    : public SmartStereoProjectionFactor {
 protected:
  /// shared pointer to calibration object (one for each camera)
  std::vector<std::shared_ptr<Cal3_S2Stereo>> K_all_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// shorthand for base class type
  typedef SmartStereoProjectionFactor Base;

  /// shorthand for this class
  typedef SmartStereoProjectionPoseFactor This;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param Isotropic measurement noise
   * @param params internal parameters of the smart factors
   */
  SmartStereoProjectionPoseFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const SmartStereoProjectionParams& params = SmartStereoProjectionParams(),
      const std::optional<Pose3>& body_P_sensor = {});

  /** Virtual destructor */
  ~SmartStereoProjectionPoseFactor() override = default;

  /**
   * add a new measurement and pose key
   * @param measured is the 2m dimensional location of the projection of a
   * single landmark in the m view (the measurement)
   * @param poseKey is key corresponding to the camera observing the same
   * landmark
   * @param K is the (fixed) camera calibration
   */
  void add(const StereoPoint2& measured, const Key& poseKey,
           const std::shared_ptr<Cal3_S2Stereo>& K);

  /**
   *  Variant of the previous one in which we include a set of measurements
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m view (the measurement)
   * @param poseKeys vector of keys corresponding to the camera observing
   * the same landmark
   * @param Ks vector of calibration objects
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& poseKeys,
           const std::vector<std::shared_ptr<Cal3_S2Stereo>>& Ks);

  /**
   * Variant of the previous one in which we include a set of measurements with
   * the same noise and calibration
   * @param measurements vector of the 2m dimensional location of the projection
   * of a single landmark in the m view (the measurement)
   * @param poseKeys vector of keys corresponding to the camera observing the
   * same landmark
   * @param K the (known) camera calibration (same for all measurements)
   */
  void add(const std::vector<StereoPoint2>& measurements,
           const KeyVector& poseKeys,
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

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(K_all_);
  }
#endif

};  // end of class declaration

/// traits
template <>
struct traits<SmartStereoProjectionPoseFactor>
    : public Testable<SmartStereoProjectionPoseFactor> {};

}  // namespace gtsam
