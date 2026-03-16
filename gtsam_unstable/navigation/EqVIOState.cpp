/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOState.cpp
/// @brief Dynamic EqVIO manifold state implementation.
/// @author Rohan Bansal

#include <gtsam_unstable/navigation/EqVIOState.h>

#include <iostream>
#include <stdexcept>

namespace gtsam {
namespace eqvio {


void Landmark::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << " ";
  std::cout << "Landmark{id=" << id << ", p=" << p.transpose() << "}"
            << std::endl;
}

bool Landmark::equals(const Landmark& other, double tol) const {
  return id == other.id && equal_with_abs_tol(p, other.p, tol);
}

Vector3 VIOSensorState::gravityDir() const {
  return pose.rotation().unrotate(Vector3::UnitZ());
}

void VIOSensorState::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << std::endl;
  gtsam::print(Vector(inputBias.vector()), "  inputBias");
  pose.print("  pose");
  gtsam::print(Vector(velocity), "  velocity");
  cameraOffset.print("  cameraOffset");
}

bool VIOSensorState::equals(const VIOSensorState& other, double tol) const {
  return inputBias.equals(other.inputBias, tol) &&
         pose.equals(other.pose, tol) &&
         equal_with_abs_tol(velocity, other.velocity, tol) &&
         cameraOffset.equals(other.cameraOffset, tol);
}

VIOState::VIOState(const VIOSensorState& sensor_,
                   const std::vector<Landmark>& lms)
    : sensor(sensor_), cameraLandmarks(lms) {}

size_t VIOState::n() const { return cameraLandmarks.size(); }

int VIOState::dim() const {
  return VIOSensorState::CompDim + Landmark::CompDim * static_cast<int>(n());
}

std::vector<int> VIOState::ids() const {
  std::vector<int> out;
  out.reserve(cameraLandmarks.size());
  for (const Landmark& lm : cameraLandmarks) out.push_back(lm.id);
  return out;
}

VIOState VIOState::retract(const TangentVector& v, ChartJacobian H1,
                           ChartJacobian H2) const {
  const int d = dim();

  Matrix66 Hpose1, Hpose2, Hcam1, Hcam2;
  VIOState out(*this);

  out.sensor.inputBias = sensor.inputBias.retract(v.segment<6>(0));
  out.sensor.pose =
      sensor.pose.retract(v.segment<6>(6), H1 ? &Hpose1 : nullptr,
                          H2 ? &Hpose2 : nullptr);
  out.sensor.velocity += v.segment<3>(12);
  out.sensor.cameraOffset =
      sensor.cameraOffset.retract(v.segment<6>(15), H1 ? &Hcam1 : nullptr,
                                  H2 ? &Hcam2 : nullptr);

  for (size_t i = 0; i < n(); ++i) {
    out.cameraLandmarks[i].p +=
        v.segment<3>(VIOSensorState::CompDim + 3 * static_cast<int>(i));
  }

  if (H1) {
    H1->setZero(d, d);
    H1->block<6, 6>(0, 0).setIdentity();
    H1->block<6, 6>(6, 6) = Hpose1;
    H1->block<3, 3>(12, 12).setIdentity();
    H1->block<6, 6>(15, 15) = Hcam1;
    for (size_t i = 0; i < n(); ++i) {
      const int row = VIOSensorState::CompDim + 3 * static_cast<int>(i);
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
      const int row = VIOSensorState::CompDim + 3 * static_cast<int>(i);
      H2->block<3, 3>(row, row).setIdentity();
    }
  }

  return out;
}

VIOState::TangentVector VIOState::localCoordinates(const VIOState& other,
                                                   ChartJacobian H1,
                                                   ChartJacobian H2) const {
  const int d = dim();

  Matrix66 Hpose1, Hpose2, Hcam1, Hcam2;
  TangentVector out = Vector::Zero(d);

  out.segment<6>(0) = sensor.inputBias.localCoordinates(other.sensor.inputBias);
  out.segment<6>(6) = sensor.pose.localCoordinates(
      other.sensor.pose, H1 ? &Hpose1 : nullptr, H2 ? &Hpose2 : nullptr);
  out.segment<3>(12) = other.sensor.velocity - sensor.velocity;
  out.segment<6>(15) = sensor.cameraOffset.localCoordinates(
      other.sensor.cameraOffset, H1 ? &Hcam1 : nullptr, H2 ? &Hcam2 : nullptr);

  for (size_t i = 0; i < n(); ++i) {
    out.segment<3>(VIOSensorState::CompDim + 3 * static_cast<int>(i)) =
        other.cameraLandmarks[i].p - cameraLandmarks[i].p;
  }

  if (H1) {
    H1->setZero(d, d);
    H1->block<6, 6>(0, 0) = -Matrix66::Identity();
    H1->block<6, 6>(6, 6) = Hpose1;
    H1->block<3, 3>(12, 12) = -Matrix3::Identity();
    H1->block<6, 6>(15, 15) = Hcam1;
    for (size_t i = 0; i < n(); ++i) {
      const int row = VIOSensorState::CompDim + 3 * static_cast<int>(i);
      H1->block<3, 3>(row, row) = -Matrix3::Identity();
    }
  }

  if (H2) {
    H2->setZero(d, d);
    H2->block<6, 6>(0, 0).setIdentity();
    H2->block<6, 6>(6, 6) = Hpose2;
    H2->block<3, 3>(12, 12).setIdentity();
    H2->block<6, 6>(15, 15) = Hcam2;
    for (size_t i = 0; i < n(); ++i) {
      const int row = VIOSensorState::CompDim + 3 * static_cast<int>(i);
      H2->block<3, 3>(row, row).setIdentity();
    }
  }

  return out;
}

void VIOState::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << std::endl;
  std::cout << "VIOState(dim=" << dim() << ", n=" << n() << ")" << std::endl;
  sensor.print("  sensor");
  for (size_t i = 0; i < cameraLandmarks.size(); ++i) {
    cameraLandmarks[i].print("  landmark[" + std::to_string(i) + "]");
  }
}

bool VIOState::equals(const VIOState& other, double tol) const {
  if (cameraLandmarks.size() != other.cameraLandmarks.size()) return false;
  if (!sensor.equals(other.sensor, tol)) return false;
  for (size_t i = 0; i < cameraLandmarks.size(); ++i) {
    if (!cameraLandmarks[i].equals(other.cameraLandmarks[i], tol)) return false;
  }
  return true;
}

}  // namespace eqvio
}  // namespace gtsam
