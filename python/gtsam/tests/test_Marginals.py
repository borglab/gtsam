"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for wrapped Marginals methods added for covariance recovery.
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


class TestMarginals(GtsamTestCase):
    """Tests for Marginals wrapper extensions."""

    def test_joint_marginal_blocks(self):
        """JointMarginal blocks should match the full joint covariance."""
        graph, values, (x1, _, x3, l1, l2) = create_planar_slam_problem()
        marginals = gtsam.Marginals(graph, values)

        union_keys = [x1, l1, l2, x3]
        joint = marginals.jointMarginalCovariance(union_keys)
        full = joint.fullMatrix()
        ordered_keys = sorted(set(union_keys))
        dims = values.dims()

        offsets = {}
        offset = 0
        for key in ordered_keys:
            offsets[key] = offset
            offset += dims[key]

        def block(key_i, key_j):
            row = offsets[key_i]
            col = offsets[key_j]
            return full[row : row + dims[key_i], col : col + dims[key_j]]

        np.testing.assert_allclose(joint.at(l2, l2), block(l2, l2), atol=1e-8)
        np.testing.assert_allclose(joint.at(x1, l2), block(x1, l2), atol=1e-8)
        np.testing.assert_allclose(joint.at(l1, x3), block(l1, x3), atol=1e-8)
        np.testing.assert_allclose(joint.at(x3, x3), block(x3, x3), atol=1e-8)


if __name__ == "__main__":
    unittest.main()
