"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

A script demonstrating gravity estimation with the ImuFactorWithGravity family:
a stationary IMU in a navigation frame whose true gravity is tilted away from
the direction assumed by the preintegration parameters. The accelerometer
measures -R^T g_true, so optimizing the gravity variable recovers the tilt.

Two parametrizations are demonstrated:
- ImuFactorWithGravityDirection: Unit3 direction with a fixed, known magnitude.
- ImuFactorWithGravityVector: free Point3 vector, paired with a single
  VectorNormFactor3 as the magnitude pseudo-observation of
  [Lupton and Sukkarieh, TRO 2012].

Author: Nikhil Khedekar
"""

# pylint: disable=no-name-in-module,import-error

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, G, V, X


def stationary_pim(true_gravity: np.ndarray) -> gtsam.PreintegratedImuMeasurements:
    """Integrate one second of stationary IMU data under the given gravity."""
    # Nominal params believe gravity is straight down (z-up navigation frame);
    # they also provide the default magnitude for the Direction factor.
    params = gtsam.PreintegrationParams.MakeSharedU(9.81)
    params.setAccelerometerCovariance(1e-4 * np.eye(3))
    params.setGyroscopeCovariance(1e-6 * np.eye(3))
    params.setIntegrationCovariance(1e-8 * np.eye(3))

    pim = gtsam.PreintegratedImuMeasurements(params)
    for _ in range(10):
        # A stationary accelerometer measures the specific force -R^T g_true:
        pim.integrateMeasurement(-true_gravity, np.zeros(3), 0.1)
    return pim


def make_graph_and_values():
    """Tight priors anchor both states and the bias; gravity is left free."""
    graph = gtsam.NonlinearFactorGraph()
    tight_pose = gtsam.noiseModel.Isotropic.Sigma(6, 1e-6)
    tight_vec = gtsam.noiseModel.Isotropic.Sigma(3, 1e-6)
    tight_bias = gtsam.noiseModel.Isotropic.Sigma(6, 1e-6)
    graph.addPriorPose3(X(1), gtsam.Pose3(), tight_pose)
    graph.addPriorPose3(X(2), gtsam.Pose3(), tight_pose)
    graph.addPriorVector(V(1), np.zeros(3), tight_vec)
    graph.addPriorVector(V(2), np.zeros(3), tight_vec)
    graph.addPriorConstantBias(B(1), gtsam.imuBias.ConstantBias(), tight_bias)

    values = gtsam.Values()
    values.insert(X(1), gtsam.Pose3())
    values.insert(X(2), gtsam.Pose3())
    values.insert(V(1), np.zeros(3))
    values.insert(V(2), np.zeros(3))
    values.insert(B(1), gtsam.imuBias.ConstantBias())
    return graph, values


def main():
    # True gravity: tilted ~3.3 degrees away from straight down.
    true_gravity = gtsam.Rot3.Rodrigues(0.05, -0.03, 0.0).rotate(
        gtsam.Point3(0, 0, -9.81))
    print(f"true gravity:                      {np.round(true_gravity, 4)}")

    pim = stationary_pim(true_gravity)

    # Mode 1 (for reference): plain ImuFactor with gravity fixed by the params
    # would leave a large residual here, since the params assume (0, 0, -9.81).

    # Mode 2: direction on the sphere, magnitude fixed to 9.81 (from params).
    graph, values = make_graph_and_values()
    graph.add(gtsam.ImuFactorWithGravityDirection(
        X(1), V(1), X(2), V(2), B(1), G(0), pim))
    values.insert(G(0), gtsam.Unit3(np.array([0.0, 0.0, -1.0])))
    result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    recovered = result.atUnit3(G(0)).unitVector() * 9.81
    print(f"recovered (direction, |g| fixed):  {np.round(recovered, 4)}"
          f"   error {np.linalg.norm(recovered - true_gravity):.2e}")

    # Mode 3: free vector with Lupton's magnitude pseudo-observation, added
    # once on the gravity variable (never per IMU factor).
    graph, values = make_graph_and_values()
    graph.add(gtsam.ImuFactorWithGravityVector(
        X(1), V(1), X(2), V(2), B(1), G(0), pim))
    graph.add(gtsam.VectorNormFactor3(
        G(0), 9.81, gtsam.noiseModel.Isotropic.Sigma(1, 0.03)))
    values.insert(G(0), gtsam.Point3(0, 0, -9.0))  # never initialize at zero!
    result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()
    recovered = result.atPoint3(G(0))
    print(f"recovered (free vector + norm):    {np.round(recovered, 4)}"
          f"   error {np.linalg.norm(recovered - true_gravity):.2e}")


if __name__ == "__main__":
    main()
