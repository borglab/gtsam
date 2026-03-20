/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOState.h
/// @brief Dynamic EqVIO manifold state.
/// @author Rohan Bansal

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/navigation/EqVIOCommon.h>
#include <gtsam_unstable/dllexport.h>

#include <string>
#include <vector>

namespace gtsam {
namespace eqvio {

struct GTSAM_UNSTABLE_EXPORT Landmark {
  static constexpr int CompDim = 3;

  Point3 p = Point3::Zero();
  int id = -1;

  void print(const std::string& s = "") const;
  bool equals(const Landmark& other, double tol = 1e-9) const;
};

struct GTSAM_UNSTABLE_EXPORT SensorState {
  static constexpr int CompDim = 21;

  Bias inputBias = Bias::Identity();
  Pose3 pose = Pose3::Identity();
  Vector3 velocity = Vector3::Zero();
  Pose3 cameraOffset = Pose3::Identity();

  /// Unit gravity direction expressed in the body frame.
  Vector3 gravityDir() const;

  void print(const std::string& s = "") const;
  bool equals(const SensorState& other, double tol = 1e-9) const;
};

/// Dynamic VIO state manifold with dimension 21 + 3n.
class GTSAM_UNSTABLE_EXPORT State {
 public:
  static constexpr int dimension = Eigen::Dynamic;

  using TangentVector = Vector;
  using Jacobian = Matrix;
  using ChartJacobian = OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic>;

  SensorState sensor;
  std::vector<Landmark> cameraLandmarks;

  /// Construct default-initialized state.
  State() = default;
  /// Construct from explicit sensor and landmark blocks.
  State(const SensorState& sensor_, const std::vector<Landmark>& lms);

  /// Number of landmarks.
  size_t n() const;
  int dim() const;
  std::vector<int> ids() const;

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
