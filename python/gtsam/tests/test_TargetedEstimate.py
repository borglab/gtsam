"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for targeted single-key calculateEstimate on ISAM2 and on both
fixed-lag smoothers.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, S, X
from gtsam.utils.test_case import GtsamTestCase


def scalar_problem():
    """A two-variable scalar chain: prior on s1, between s1 and s2."""
    graph = gtsam.NonlinearFactorGraph()
    noise = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)

    graph.add(gtsam.PriorFactorDouble(S(1), 1.0, noise))
    graph.add(gtsam.BetweenFactorDouble(S(1), S(2), 2.0, noise))

    values = gtsam.Values()
    values.insert(S(1), 0.9)
    values.insert(S(2), 3.2)
    return graph, values


def navstate_problem():
    """A single NavState constrained by a prior."""
    graph = gtsam.NonlinearFactorGraph()
    noise = gtsam.noiseModel.Isotropic.Sigma(9, 0.1)

    prior = gtsam.NavState(gtsam.Rot3.Ypr(0.1, 0.0, 0.0),
                           gtsam.Point3(1.0, 2.0, 3.0),
                           np.array([0.5, 0.0, 0.0]))
    graph.add(gtsam.PriorFactorNavState(X(1), prior, noise))

    values = gtsam.Values()
    values.insert(X(1), gtsam.NavState())
    return graph, values


def timestamps(*pairs):
    """Key-timestamp map for the smoothers, which take a plain dict."""
    return {key: time for key, time in pairs}


class TestISAM2TargetedEstimate(GtsamTestCase):
    """ISAM2 gains scalar and NavState extraction."""

    def test_scalar(self):
        graph, values = scalar_problem()
        isam = gtsam.ISAM2()
        isam.update(graph, values)

        expected = isam.calculateEstimate().atDouble(S(2))
        self.assertAlmostEqual(isam.calculateEstimateDouble(S(2)), expected)

    def test_navstate(self):
        graph, values = navstate_problem()
        isam = gtsam.ISAM2()
        isam.update(graph, values)

        expected = isam.calculateEstimate().atNavState(X(1))
        actual = isam.calculateEstimateNavState(X(1))
        self.gtsamAssertEquals(actual, expected, 1e-9)


class TestIncrementalFixedLagSmootherTargetedEstimate(GtsamTestCase):
    """The incremental smoother gains the templated accessor entirely.

    Previously the only way to read a single key from an
    IncrementalFixedLagSmoother was to go through getISAM2().
    """

    def test_scalar(self):
        graph, values = scalar_problem()
        smoother = gtsam.IncrementalFixedLagSmoother(10.0)
        smoother.update(graph, values, timestamps((S(1), 0.0), (S(2), 1.0)))

        expected = smoother.calculateEstimate().atDouble(S(2))
        self.assertAlmostEqual(smoother.calculateEstimateDouble(S(2)), expected)

    def test_navstate(self):
        graph, values = navstate_problem()
        smoother = gtsam.IncrementalFixedLagSmoother(10.0)
        smoother.update(graph, values, timestamps((X(1), 0.0)))

        expected = smoother.calculateEstimate().atNavState(X(1))
        actual = smoother.calculateEstimateNavState(X(1))
        self.gtsamAssertEquals(actual, expected, 1e-9)

    def test_matches_isam2_directly(self):
        """The forwarding accessor agrees with reaching through getISAM2()."""
        graph, values = scalar_problem()
        smoother = gtsam.IncrementalFixedLagSmoother(10.0)
        smoother.update(graph, values, timestamps((S(1), 0.0), (S(2), 1.0)))

        self.assertAlmostEqual(smoother.calculateEstimateDouble(S(2)),
                               smoother.getISAM2().calculateEstimateDouble(S(2)))


class TestBatchFixedLagSmootherTargetedEstimate(GtsamTestCase):
    """The batch smoother gains scalar, NavState and bias extraction."""

    def test_scalar(self):
        graph, values = scalar_problem()
        smoother = gtsam.BatchFixedLagSmoother(10.0)
        smoother.update(graph, values, timestamps((S(1), 0.0), (S(2), 1.0)))

        expected = smoother.calculateEstimate().atDouble(S(2))
        self.assertAlmostEqual(smoother.calculateEstimateDouble(S(2)), expected)

    def test_constant_bias(self):
        bias = gtsam.imuBias.ConstantBias(np.array([0.1, 0.2, 0.3]),
                                          np.array([0.01, 0.02, 0.03]))
        noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.PriorFactorConstantBias(B(1), bias, noise))

        values = gtsam.Values()
        values.insert(B(1), gtsam.imuBias.ConstantBias())

        smoother = gtsam.BatchFixedLagSmoother(10.0)
        smoother.update(graph, values, timestamps((B(1), 0.0)))

        expected = smoother.calculateEstimate().atConstantBias(B(1))
        actual = smoother.calculateEstimateConstantBias(B(1))
        self.gtsamAssertEquals(actual, expected, 1e-9)


if __name__ == "__main__":
    unittest.main()
