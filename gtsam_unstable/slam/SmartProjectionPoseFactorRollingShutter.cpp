/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionPoseFactorRollingShutter.h
 * @brief  Smart projection factor on poses modeling rolling shutter effect
 * @author Luca Carlone
 */

//#include <gtsam_unstable/slam/SmartProjectionPoseFactorRollingShutter.h>

namespace gtsam {


//
//void SmartProjectionPoseFactorRollingShutter::add(
//    const std::vector<StereoPoint2>& measurements,
//    const KeyVector& world_P_body_keys, const KeyVector& body_P_cam_keys,
//    const std::vector<boost::shared_ptr<Cal3_S2Stereo>>& Ks) {
//  assert(world_P_body_keys.size() == measurements.size());
//  assert(world_P_body_keys.size() == body_P_cam_keys.size());
//  assert(world_P_body_keys.size() == Ks.size());
//  for (size_t i = 0; i < measurements.size(); i++) {
//      Base::add(measurements[i], world_P_body_keys[i]);
//      // pose keys are assumed to be unique (1 observation per time stamp), but calibration can be shared
//      if(std::find(keys_.begin(), keys_.end(), body_P_cam_keys[i]) == keys_.end())
//          keys_.push_back(body_P_cam_keys[i]); // add only unique keys
//
//      world_P_body_keys_.push_back(world_P_body_keys[i]);
//      body_P_cam_keys_.push_back(body_P_cam_keys[i]);
//
//      K_all_.push_back(Ks[i]);
//    }
//}
//
//void SmartProjectionPoseFactorRollingShutter::add(
//    const std::vector<StereoPoint2>& measurements,
//    const KeyVector& world_P_body_keys, const KeyVector& body_P_cam_keys,
//    const boost::shared_ptr<Cal3_S2Stereo>& K) {
//  assert(world_P_body_keys.size() == measurements.size());
//  assert(world_P_body_keys.size() == body_P_cam_keys.size());
//  for (size_t i = 0; i < measurements.size(); i++) {
//    Base::add(measurements[i], world_P_body_keys[i]);
//    // pose keys are assumed to be unique (1 observation per time stamp), but calibration can be shared
//    if(std::find(keys_.begin(), keys_.end(), body_P_cam_keys[i]) == keys_.end())
//      keys_.push_back(body_P_cam_keys[i]); // add only unique keys
//
//    world_P_body_keys_.push_back(world_P_body_keys[i]);
//    body_P_cam_keys_.push_back(body_P_cam_keys[i]);
//
//    K_all_.push_back(K);
//  }
//}
//
//void SmartProjectionPoseFactorRollingShutter::print(
//    const std::string& s, const KeyFormatter& keyFormatter) const {
//  std::cout << s << "SmartProjectionPoseFactorRollingShutter: \n ";
//  for (size_t i = 0; i < K_all_.size(); i++) {
//    K_all_[i]->print("calibration = ");
//    std::cout << " extrinsic pose key: " << keyFormatter(body_P_cam_keys_[i]) << std::endl;
//  }
//  Base::print("", keyFormatter);
//}
//
//bool SmartProjectionPoseFactorRollingShutter::equals(const NonlinearFactor& p,
//                                             double tol) const {
//  const SmartProjectionPoseFactorRollingShutter* e =
//      dynamic_cast<const SmartProjectionPoseFactorRollingShutter*>(&p);
//
//  return e && Base::equals(p, tol) &&
//      body_P_cam_keys_ == e->getExtrinsicPoseKeys();
//}
//
//double SmartProjectionPoseFactorRollingShutter::error(const Values& values) const {
//  if (this->active(values)) {
//    return this->totalReprojectionError(cameras(values));
//  } else {  // else of active flag
//    return 0.0;
//  }
//}
//
//SmartProjectionPoseFactorRollingShutter::Base::Cameras
//SmartProjectionPoseFactorRollingShutter::cameras(const Values& values) const {
//  assert(world_P_body_keys_.size() == K_all_.size());
//  assert(world_P_body_keys_.size() == body_P_cam_keys_.size());
//  Base::Cameras cameras;
//  for (size_t i = 0; i < world_P_body_keys_.size(); i++) {
//    Pose3 w_P_body = values.at<Pose3>(world_P_body_keys_[i]);
//    Pose3 body_P_cam = values.at<Pose3>(body_P_cam_keys_[i]);
//    Pose3 w_P_cam = w_P_body.compose(body_P_cam);
//    cameras.push_back(StereoCamera(w_P_cam, K_all_[i]));
//  }
//  return cameras;
//}

}  // \ namespace gtsam
