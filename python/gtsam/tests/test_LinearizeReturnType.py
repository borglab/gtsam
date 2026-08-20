"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the Python type of factors produced by linearization.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

def pose_values():
    values = gtsam.Values()
    values.insert(X(1), gtsam.Pose3())
    values.insert(X(2), gtsam.Pose3(gtsam.Rot3(),
                                    gtsam.Point3(1.0, 0.0, 0.0)))
    return values


def between_factor():
    """A fixed-size-error factor: takes the FixedJacobianFactor path."""
    return gtsam.BetweenFactorPose3(
        X(1), X(2), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
        gtsam.noiseModel.Isotropic.Sigma(6, 0.1))


def prior_factor():
    """PriorFactor uses NoiseModelFactorN, whose error type is a dynamic
    Vector, so it retains the generic linearization path."""
    return gtsam.PriorFactorPose3(X(1), gtsam.Pose3(),
                                  gtsam.noiseModel.Isotropic.Sigma(6, 0.1))


class TestLinearizeReturnType(GtsamTestCase):
    """linearize() must yield a usable JacobianFactor in Python.

    NonlinearFactor::linearize is declared to return gtsam::GaussianFactor*.
    When the concrete type is not registered with pybind11, pybind falls back
    to that declared type rather than walking up to the nearest registered
    base, so the JacobianFactor interface is lost.
    """

    def test_between_factor_linearizes_to_jacobian_factor(self):
        linear = between_factor().linearize(pose_values())
        self.assertIsInstance(linear, gtsam.JacobianFactor)

    def test_jacobian_interface_is_reachable(self):
        """Type-checking is not enough: the methods must actually work."""
        linear = between_factor().linearize(pose_values())

        self.assertEqual(linear.rows(), 6)
        # Two Pose3 keys of dimension 6 each.
        self.assertEqual(linear.getA().shape, (6, 12))
        self.assertEqual(len(linear.getb()), 6)

    def test_prior_factor_linearizes_to_jacobian_factor(self):
        """Control: this path was never affected."""
        linear = prior_factor().linearize(pose_values())

        self.assertIsInstance(linear, gtsam.JacobianFactor)
        self.assertEqual(linear.rows(), 6)

    def test_graph_linearize_yields_jacobian_factors(self):
        """The same must hold when reached through a linearized graph.

        GaussianFactorGraph::at is also declared to return GaussianFactor*.
        """
        graph = gtsam.NonlinearFactorGraph()
        graph.add(prior_factor())
        graph.add(between_factor())

        linear_graph = graph.linearize(pose_values())
        self.assertEqual(linear_graph.size(), 2)
        for i in range(linear_graph.size()):
            self.assertIsInstance(linear_graph.at(i), gtsam.JacobianFactor)

    def test_registered_subclasses_keep_their_identity(self):
        """GaussianConditional derives from JacobianFactor and is registered.

        Reached through GaussianFactorGraph::at, which is declared to return
        GaussianFactor*, so this exercises the same resolution path. The
        conditional must not be flattened to a plain JacobianFactor.
        """
        graph = gtsam.NonlinearFactorGraph()
        graph.add(prior_factor())
        graph.add(between_factor())

        bayes_net = graph.linearize(pose_values()).eliminateSequential()
        as_graph = gtsam.GaussianFactorGraph(bayes_net)

        self.assertGreater(as_graph.size(), 0)
        self.assertIsInstance(as_graph.at(0), gtsam.GaussianConditional)


if __name__ == "__main__":
    unittest.main()
