/*
 * LocalOrientedPlane3Factor.cpp
 *
 *  Author: David Wisth
 *  Created on: February 12, 2021
 */

#include "LocalOrientedPlane3Factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void LocalOrientedPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << s << (s == "" ? "" : "\n");
  cout << "LocalOrientedPlane3Factor Factor (" << keyFormatter(key1()) << ", "
       << keyFormatter(key2()) << ", " << keyFormatter(key3()) << ")\n";
  measured_p_.print("Measured Plane");
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
Vector LocalOrientedPlane3Factor::evaluateError(const Pose3& basePose,
    const Pose3& anchorPose, const OrientedPlane3& plane,
    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
    boost::optional<Matrix&> H3) const {

  Matrix66 pose_H_anchorPose, pose_H_basePose;
  Matrix36 predicted_H_pose;
  Matrix33 predicted_H_plane, error_H_predicted;

  // T_LB = inv(T_WL) * T_WB
  const Pose3 relativePose = anchorPose.transformPoseTo(basePose,
      H2 ? &pose_H_anchorPose : nullptr,
      H1 ? &pose_H_basePose : nullptr);

  const OrientedPlane3 predicted_plane = plane.transform(relativePose,
      H2 ? &predicted_H_plane : nullptr,
      (H1 || H3) ? &predicted_H_pose  : nullptr);

  const Vector3 err = measured_p_.error(predicted_plane,
    boost::none, (H1 || H2 || H3) ? &error_H_predicted : nullptr);

  // const Vector3 err = predicted_plane.errorVector(measured_p_,
  //   (H1 || H2 || H3) ? &error_H_predicted : nullptr);

  // Apply the chain rule to calculate the derivatives.
  if (H1) {
    *H1 = error_H_predicted * predicted_H_pose * pose_H_basePose;
    // std::cout << "H1:\n" << *H1 << std::endl;
  }

  if (H2) {
    *H2 = error_H_predicted * predicted_H_pose * pose_H_anchorPose;
    // std::cout << "H2:\n" << *H2 << std::endl;
  }

  if (H3) {
    *H3 = error_H_predicted * predicted_H_plane;
    // std::cout << "H3:\n" << *H3 << std::endl;

    // measured_p_.print();
    // predicted_plane.print();

    // std::cout << "pose_H_anchorPose:\n" << pose_H_anchorPose << std::endl;
    // std::cout << "pose_H_basePose:\n" << pose_H_basePose << std::endl;
    // std::cout << "predicted_H_pose:\n" << predicted_H_pose << std::endl;
    // std::cout << "error_H_predicted:\n" << error_H_predicted << std::endl;
    // std::cout << "predicted_H_plane:\n" << predicted_H_plane << std::endl;

    std::cout << "H3^T x error:\n" << (*H3).transpose() * err << std::endl;
    // std::cout << "H3:\n" << *H3 << std::endl;
  }

  // std::cout << "Error: " << err.transpose() << std::endl;

  return err;
}

}  // namespace gtsam
