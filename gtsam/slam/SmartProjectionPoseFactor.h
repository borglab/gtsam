/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionPoseFactor.h
 * @brief  Smart factor on poses, assuming camera calibration is fixed
 * @author Luca Carlone
 * @author Chris Beall
 * @author Zsolt Kira
 */

#pragma once

#include <gtsam/slam/SmartProjectionFactor.h>

namespace gtsam {
/**
 *
 * @ingroup slam
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating conditionally
 * independent sets in factor graphs: a unifying perspective based on smart factors,
 * Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 */

/**
 * This factor assumes that camera calibration is fixed, and that
 * the calibration is the same for all cameras involved in this factor.
 * The factor only constrains poses (variable dimension is 6).
 * This factor requires that values contains the involved poses (Pose3).
 * If the calibration should be optimized, as well, use SmartProjectionFactor instead!
 * @ingroup slam
 */
template <class CALIBRATION>
class SmartProjectionPoseFactor
    : public SmartProjectionFactor<PinholePose<CALIBRATION> > {
 private:
  typedef PinholePose<CALIBRATION> Camera;
  typedef SmartProjectionFactor<Camera> Base;
  typedef SmartProjectionPoseFactor<CALIBRATION> This;

protected:

  std::shared_ptr<CALIBRATION> K_; ///< calibration object (one for all cameras)

public:

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Default constructor, only for serialization
   */
  SmartProjectionPoseFactor() {}

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param K (fixed) calibration, assumed to be the same for all cameras
   * @param params parameters for the smart projection factors
   */
  SmartProjectionPoseFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const std::shared_ptr<CALIBRATION> K,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel, params), K_(K) {
  }

  /**
   * Constructor
   * @param sharedNoiseModel isotropic noise model for the 2D feature measurements
   * @param K (fixed) calibration, assumed to be the same for all cameras
   * @param body_P_sensor pose of the camera in the body frame (optional)
   * @param params parameters for the smart projection factors
   */
  SmartProjectionPoseFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const std::shared_ptr<CALIBRATION> K,
      const std::optional<Pose3> body_P_sensor,
      const SmartProjectionParams& params = SmartProjectionParams())
      : SmartProjectionPoseFactor(sharedNoiseModel, K, params) {
    this->body_P_sensor_ = body_P_sensor;
  }

  /** Virtual destructor */
  ~SmartProjectionPoseFactor() override {
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "SmartProjectionPoseFactor, z = \n ";
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol);
  }

  /**
   * error calculates the error of the factor.
   */
  double error(const Values& values) const override {
    if (this->active(values)) {
      return this->totalReprojectionError(cameras(values));
    } else { // else of active flag
      return 0.0;
    }
  }

  /** return calibration shared pointers */
  inline const std::shared_ptr<CALIBRATION> calibration() const {
    return K_;
  }

  /**
   * Collect all cameras involved in this factor
   * @param values Values structure which must contain camera poses corresponding
   * to keys involved in this factor
   * @return vector of Values
   */
  typename Base::Cameras cameras(const Values& values) const override {
    typename Base::Cameras cameras;
    for (const Key& k : this->keys_) {
      const Pose3 world_P_sensor_k =
          Base::body_P_sensor_ ? values.at<Pose3>(k) * *Base::body_P_sensor_
                               : values.at<Pose3>(k);
      cameras.emplace_back(world_P_sensor_k, K_);
    }
    return cameras;
  }

 private:

  /// Serialization function
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }
#endif
};
// end of class declaration

/// traits
template<class CALIBRATION>
struct traits<SmartProjectionPoseFactor<CALIBRATION> > : public Testable<
    SmartProjectionPoseFactor<CALIBRATION> > {
};

} // \ namespace gtsam
