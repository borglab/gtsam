/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactorRollingShutter.cpp
 * @brief Basic projection factor for rolling shutter cameras
 * @author Yotam Stern
 */

#include <gtsam_unstable/slam/ProjectionFactorRollingShutter.h>

namespace gtsam {

Vector ProjectionFactorRollingShutter::evaluateError(
    const Pose3& pose_a, const Pose3& pose_b, const Point3& point,
    OptionalMatrixType H1, OptionalMatrixType H2,
    OptionalMatrixType H3) const {
  try {
    Pose3 pose = interpolate<Pose3>(pose_a, pose_b, alpha_, H1, H2);
    gtsam::Matrix Hprj;
    if (body_P_sensor_) {
      if (H1 || H2 || H3) {
        gtsam::Matrix HbodySensor;
        PinholeCamera<Cal3_S2> camera(
            pose.compose(*body_P_sensor_, HbodySensor), *K_);
        Point2 reprojectionError(camera.project(point, Hprj, H3, boost::none) -
                                 measured_);
        if (H1) *H1 = Hprj * HbodySensor * (*H1);
        if (H2) *H2 = Hprj * HbodySensor * (*H2);
        return reprojectionError;
      } else {
        PinholeCamera<Cal3_S2> camera(pose.compose(*body_P_sensor_), *K_);
        return camera.project(point) - measured_;
      }
    } else {
      PinholeCamera<Cal3_S2> camera(pose, *K_);
      Point2 reprojectionError(camera.project(point, Hprj, H3, boost::none) -
                               measured_);
      if (H1) *H1 = Hprj * (*H1);
      if (H2) *H2 = Hprj * (*H2);
      return reprojectionError;
    }
  } catch (CheiralityException& e) {
    if (H1) *H1 = Matrix::Zero(2, 6);
    if (H2) *H2 = Matrix::Zero(2, 6);
    if (H3) *H3 = Matrix::Zero(2, 3);
    if (verboseCheirality_)
      std::cout << e.what() << ": Landmark "
                << DefaultKeyFormatter(this->key<2>()) << " moved behind camera "
                << DefaultKeyFormatter(this->key<1>()) << std::endl;
    if (throwCheirality_) throw CheiralityException(this->key<2>());
  }
  return Vector2::Constant(2.0 * K_->fx());
}

}  // namespace gtsam
