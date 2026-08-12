"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for wrapped ISAM2 marginal query methods.
Author: Codex, prompted by Frank Dellaert
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import L, X
from gtsam.utils.test_case import GtsamTestCase


def create_planar_slam_problem():
    """Create a small nonlinear SLAM problem with pose and landmark variables."""
    graph = gtsam.NonlinearFactorGraph()

    x1 = X(1)
    x2 = X(2)
    x3 = X(3)
    l1 = L(1)
    l2 = L(2)

    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.2]))

    graph.addPriorPose2(x1, gtsam.Pose2(0.0, 0.0, 0.0), prior_noise)
    graph.add(
        gtsam.BetweenFactorPose2(x1, x2, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise)
    )
    graph.add(
        gtsam.BetweenFactorPose2(x2, x3, gtsam.Pose2(2.0, 0.0, 0.0), odometry_noise)
    )
    graph.add(
        gtsam.BearingRangeFactor2D(
            x1, l1, gtsam.Rot2.fromDegrees(45), np.sqrt(8.0), measurement_noise
        )
    )
    graph.add(
        gtsam.BearingRangeFactor2D(
            x2, l1, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise
        )
    )
    graph.add(
        gtsam.BearingRangeFactor2D(
            x3, l2, gtsam.Rot2.fromDegrees(90), 2.0, measurement_noise
        )
    )

    values = gtsam.Values()
    values.insert(x1, gtsam.Pose2(0.0, 0.0, 0.0))
    values.insert(x2, gtsam.Pose2(2.0, 0.0, 0.0))
    values.insert(x3, gtsam.Pose2(4.0, 0.0, 0.0))
    values.insert(l1, gtsam.Point2(2.0, 2.0))
    values.insert(l2, gtsam.Point2(4.0, 2.0))

    return graph, values, (x1, x2, x3, l1, l2)


class TestISAM2(GtsamTestCase):
    """Tests for ISAM2 wrapper marginal query methods."""

    def test_marginal_information_and_covariance(self):
        """ISAM2 single-key marginal queries should match batch Marginals."""
        graph, values, (_, x2, _, _, _) = create_planar_slam_problem()
        isam = gtsam.ISAM2()
        isam.update(graph, values)
        marginals = gtsam.Marginals(isam.getFactorsUnsafe(), isam.getLinearizationPoint())

        np.testing.assert_allclose(
            isam.marginalInformation(x2), marginals.marginalInformation(x2), atol=1e-8
        )
        np.testing.assert_allclose(
            isam.marginalCovariance(x2), marginals.marginalCovariance(x2), atol=1e-8
        )

    def test_joint_marginal_information_and_covariance(self):
        """ISAM2 joint marginal queries should match batch Marginals."""
        graph, values, (x1, _, x3, l1, _) = create_planar_slam_problem()
        isam = gtsam.ISAM2()
        isam.update(graph, values)
        marginals = gtsam.Marginals(isam.getFactorsUnsafe(), isam.getLinearizationPoint())
        query_keys = [x3, l1, x1]

        np.testing.assert_allclose(
            isam.jointMarginalCovariance(query_keys).fullMatrix(),
            marginals.jointMarginalCovariance(query_keys).fullMatrix(),
            atol=1e-8,
        )
        np.testing.assert_allclose(
            isam.jointMarginalInformation(query_keys).fullMatrix(),
            marginals.jointMarginalInformation(query_keys).fullMatrix(),
            atol=1e-8,
        )


if __name__ == "__main__":
    unittest.main()
