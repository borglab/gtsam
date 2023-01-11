/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartStereoProjectionPoseFactor.cpp
 * @brief  Smart stereo factor on poses, assuming camera calibration is fixed
 * @author Luca Carlone
 * @author Antoni Rosinol
 * @author Chris Beall
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

namespace gtsam {

SmartStereoProjectionPoseFactor::SmartStereoProjectionPoseFactor(
    const SharedNoiseModel& sharedNoiseModel,
    const SmartStereoProjectionParams& params,
    const std::optional<Pose3>& body_P_sensor)
    : Base(sharedNoiseModel, params, body_P_sensor) {}

void SmartStereoProjectionPoseFactor::add(
    const StereoPoint2& measured, const Key& poseKey,
    const boost::shared_ptr<Cal3_S2Stereo>& K) {
  Base::add(measured, poseKey);
  K_all_.push_back(K);
}

void SmartStereoProjectionPoseFactor::add(
    const std::vector<StereoPoint2>& measurements, const KeyVector& poseKeys,
    const std::vector<boost::shared_ptr<Cal3_S2Stereo>>& Ks) {
  assert(measurements.size() == poseKeys.size());
  assert(poseKeys.size() == Ks.size());
  Base::add(measurements, poseKeys);
  K_all_.insert(K_all_.end(), Ks.begin(), Ks.end());
}

void SmartStereoProjectionPoseFactor::add(
    const std::vector<StereoPoint2>& measurements, const KeyVector& poseKeys,
    const boost::shared_ptr<Cal3_S2Stereo>& K) {
  assert(poseKeys.size() == measurements.size());
  for (size_t i = 0; i < measurements.size(); i++) {
    Base::add(measurements[i], poseKeys[i]);
    K_all_.push_back(K);
  }
}

void SmartStereoProjectionPoseFactor::print(
    const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "SmartStereoProjectionPoseFactor, z = \n ";
  for (const boost::shared_ptr<Cal3_S2Stereo>& K : K_all_) {
    K->print("calibration = ");
  }
  Base::print("", keyFormatter);
}

bool SmartStereoProjectionPoseFactor::equals(const NonlinearFactor& p,
                                             double tol) const {
  const SmartStereoProjectionPoseFactor* e =
      dynamic_cast<const SmartStereoProjectionPoseFactor*>(&p);

  return e && Base::equals(p, tol);
}

double SmartStereoProjectionPoseFactor::error(const Values& values) const {
  if (this->active(values)) {
    return this->totalReprojectionError(cameras(values));
  } else {  // else of active flag
    return 0.0;
  }
}

SmartStereoProjectionPoseFactor::Base::Cameras
SmartStereoProjectionPoseFactor::cameras(const Values& values) const {
  assert(keys_.size() == K_all_.size());
  Base::Cameras cameras;
  for (size_t i = 0; i < keys_.size(); i++) {
    Pose3 pose = values.at<Pose3>(keys_[i]);
    if (Base::body_P_sensor_) {
      pose = pose.compose(*(Base::body_P_sensor_));
    }
    cameras.push_back(StereoCamera(pose, K_all_[i]));
  }
  return cameras;
}

}  // \ namespace gtsam
