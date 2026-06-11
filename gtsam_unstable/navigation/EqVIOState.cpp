/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOState.cpp
 * @brief Dynamic EqVIO manifold state implementation.
 * @author Rohan Bansal
 */

#include <gtsam_unstable/navigation/EqVIOState.h>

namespace gtsam {
namespace eqvio {

State::State(const Se23& kinematics_, const Bias& bias_,
             const Pose3& cameraOffset_, const std::vector<Point3>& lms)
    : kinematics(kinematics_),
      bias(bias_),
      cameraOffset(cameraOffset_),
      cameraLandmarks(lms) {}

size_t State::n() const { return cameraLandmarks.size(); }

int State::dim() const { return stateDim(n()); }

Pose3 State::pose() const {
  return Pose3(kinematics.rotation(), kinematics.x(1));
}

Vector3 State::velocity() const { return kinematics.x(0); }

Vector3 State::gravityDir() const {
  return kinematics.rotation().unrotate(Vector3::UnitZ());
}

State State::retract(const TangentVector& v, ChartJacobian H1,
                     ChartJacobian H2) const {
  const int d = dim();

  Matrix66 Hpose1, Hpose2, Hcam1, Hcam2;
  State out(*this);
  const Pose3 currentPose = pose();
  const Vector3 currentVelocity = velocity();

  out.bias = bias.retract(v.segment<6>(0));
  const Pose3 nextPose = currentPose.retract(
      v.segment<6>(6), H1 ? &Hpose1 : nullptr, H2 ? &Hpose2 : nullptr);
  const Vector3 nextVelocity = currentVelocity + v.segment<3>(12);
  out.kinematics =
      Se23(nextPose.rotation(), nextVelocity, nextPose.translation());
  out.cameraOffset = cameraOffset.retract(
      v.segment<6>(15), H1 ? &Hcam1 : nullptr, H2 ? &Hcam2 : nullptr);

  for (size_t i = 0; i < n(); ++i) {
    out.cameraLandmarks[i] += v.segment<3>(21 + 3 * static_cast<int>(i));
  }

  if (H1) {
    H1->setZero(d, d);
    H1->block<6, 6>(0, 0).setIdentity();
    H1->block<6, 6>(6, 6) = Hpose1;
    H1->block<3, 3>(12, 12).setIdentity();
    H1->block<6, 6>(15, 15) = Hcam1;
    for (size_t i = 0; i < n(); ++i) {
      const int row = 21 + 3 * static_cast<int>(i);
      H1->block<3, 3>(row, row).setIdentity();
    }
  }

  if (H2) {
    H2->setZero(d, d);
    H2->block<6, 6>(0, 0).setIdentity();
    H2->block<6, 6>(6, 6) = Hpose2;
    H2->block<3, 3>(12, 12).setIdentity();
    H2->block<6, 6>(15, 15) = Hcam2;
    for (size_t i = 0; i < n(); ++i) {
      const int row = 21 + 3 * static_cast<int>(i);
      H2->block<3, 3>(row, row).setIdentity();
    }
  }

  return out;
}

State::TangentVector State::localCoordinates(const State& other,
                                             ChartJacobian H1,
                                             ChartJacobian H2) const {
  const int d = dim();

  Matrix66 Hpose1, Hpose2, Hcam1, Hcam2;
  TangentVector out = Vector::Zero(d);
  const Pose3 thisPose = pose();
  const Pose3 otherPose = other.pose();
  const Vector3 thisVelocity = velocity();
  const Vector3 otherVelocity = other.velocity();

  out.segment<6>(0) = bias.localCoordinates(other.bias);
  out.segment<6>(6) = thisPose.localCoordinates(
      otherPose, H1 ? &Hpose1 : nullptr, H2 ? &Hpose2 : nullptr);
  out.segment<3>(12) = otherVelocity - thisVelocity;
  out.segment<6>(15) = cameraOffset.localCoordinates(
      other.cameraOffset, H1 ? &Hcam1 : nullptr, H2 ? &Hcam2 : nullptr);

  for (size_t i = 0; i < n(); ++i) {
    out.segment<3>(21 + 3 * static_cast<int>(i)) =
        other.cameraLandmarks[i] - cameraLandmarks[i];
  }

  if (H1) {
    H1->setZero(d, d);
    H1->block<6, 6>(0, 0) = -I_6x6;
    H1->block<6, 6>(6, 6) = Hpose1;
    H1->block<3, 3>(12, 12) = -I_3x3;
    H1->block<6, 6>(15, 15) = Hcam1;
    for (size_t i = 0; i < n(); ++i) {
      const int row = 21 + 3 * static_cast<int>(i);
      H1->block<3, 3>(row, row) = -I_3x3;
    }
  }

  if (H2) {
    H2->setZero(d, d);
    H2->block<6, 6>(0, 0).setIdentity();
    H2->block<6, 6>(6, 6) = Hpose2;
    H2->block<3, 3>(12, 12).setIdentity();
    H2->block<6, 6>(15, 15) = Hcam2;
    for (size_t i = 0; i < n(); ++i) {
      const int row = 21 + 3 * static_cast<int>(i);
      H2->block<3, 3>(row, row).setIdentity();
    }
  }

  return out;
}

void State::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << std::endl;
  std::cout << "State(dim=" << dim() << ", n=" << n() << ")" << std::endl;
  gtsam::print(Vector(bias.vector()), "  bias");
  pose().print("  pose");
  gtsam::print(Vector(velocity()), "  velocity");
  cameraOffset.print("  cameraOffset");
  for (size_t i = 0; i < cameraLandmarks.size(); ++i) {
    std::cout << "  landmark[" << i << "] " << cameraLandmarks[i].transpose()
              << std::endl;
  }
}

bool State::equals(const State& other, double tol) const {
  if (cameraLandmarks.size() != other.cameraLandmarks.size()) return false;
  if (!bias.equals(other.bias, tol)) return false;
  if (!kinematics.equals(other.kinematics, tol)) return false;
  if (!cameraOffset.equals(other.cameraOffset, tol)) return false;
  for (size_t i = 0; i < cameraLandmarks.size(); ++i) {
    if (!equal_with_abs_tol(cameraLandmarks[i], other.cameraLandmarks[i],
                            tol)) {
      return false;
    }
  }
  return true;
}

}  // namespace eqvio
}  // namespace gtsam
