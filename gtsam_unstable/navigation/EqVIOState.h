/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EqVIOState.h
 * @brief Dynamic EqVIO manifold state.
 * @author Rohan Bansal
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/dllexport.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>

#include <string>
#include <vector>

namespace gtsam {
namespace eqvio {

/// Dimension of the dynamic EqVIO state with `landmarkCount` landmarks.
inline int stateDim(size_t landmarkCount) {
  return 21 + 3 * static_cast<int>(landmarkCount);
}

/// Dynamic VIO state manifold with dimension 21 + 3n.
class GTSAM_UNSTABLE_EXPORT State {
 public:
  static constexpr int dimension = Eigen::Dynamic;

  using TangentVector = Vector;
  using Jacobian = Matrix;
  using ChartJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

  /// Body kinematics block stored as SE_2(3) in EqVIO `(R, v, p)` order.
  Se23 kinematics = Se23::Identity();
  /// IMU bias state.
  Bias bias = Bias::Identity();
  /// Camera pose relative to IMU/body frame.
  Pose3 cameraOffset = Pose3::Identity();
  /// Landmark states in euclidean coordinates. Inverse-depth is used for the error chart / linearization rather than the stored state.
  std::vector<Point3> cameraLandmarks;

  /// Construct default-initialized state.
  State() = default;
  /// Construct from explicit kinematics, bias, extrinsics, and landmarks.
  State(const Se23& kinematics_, const Bias& bias_, const Pose3& cameraOffset_,
        const std::vector<Point3>& lms);

  /// Number of landmarks.
  size_t n() const;
  int dim() const;
  /// Body/IMU pose in world frame.
  Pose3 pose() const;
  /// Body/IMU translational velocity in world frame.
  Vector3 velocity() const;
  /// Unit gravity direction expressed in the body frame.
  Vector3 gravityDir() const;

  /// Retract in the state chart.
  State retract(const TangentVector& v, ChartJacobian H1 = {},
                ChartJacobian H2 = {}) const;
  /// Local coordinates in the state chart.
  TangentVector localCoordinates(const State& other, ChartJacobian H1 = {},
                                 ChartJacobian H2 = {}) const;

  void print(const std::string& s = "") const;
  bool equals(const State& other, double tol = 1e-9) const;
};

}  // namespace eqvio

/**
 * @brief Traits specialization for State.
 *
 * This allows State to be used as a manifold type in GTSAM.
 */
template <>
struct traits<eqvio::State> {
  static constexpr int dimension = Eigen::Dynamic;
  using TangentVector = Vector;
  using ManifoldType = eqvio::State;
  using structure_category = manifold_tag;

  static int GetDimension(const eqvio::State& xi) { return xi.dim(); }

  static eqvio::State Retract(
      const eqvio::State& xi, const TangentVector& v,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = {}) {
    return xi.retract(v, H1, H2);
  }

  static TangentVector Local(
      const eqvio::State& xi, const eqvio::State& other,
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = {},
      OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = {}) {
    return xi.localCoordinates(other, H1, H2);
  }

  static void Print(const eqvio::State& xi, const std::string& s = "") {
    xi.print(s);
  }

  static bool Equals(const eqvio::State& xi1, const eqvio::State& xi2,
                     double tol = 1e-9) {
    return xi1.equals(xi2, tol);
  }
};

template <>
struct traits<const eqvio::State> : traits<eqvio::State> {};

}  // namespace gtsam
