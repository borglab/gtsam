"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests confirming gtsam_unstable factors that derive from
NoiseModelFactor in C++ expose that interface in Python.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import P, Q, T, V, X
from gtsam.utils.test_case import GtsamTestCase


class TestUnstableFactorNoiseModel(GtsamTestCase):
    """These factors derive from NoiseModelFactorT / ExpressionFactor."""

    def test_pendulum_factor1(self):
        """Constructed with a Constrained noise model internally."""
        factor = gtsam_unstable.PendulumFactor1(Q(2), Q(1), V(1), 0.01)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertIsInstance(factor.noiseModel(),
                              gtsam.noiseModel.Constrained)

    def test_pendulum_factor2(self):
        factor = gtsam_unstable.PendulumFactor2(V(2), V(1), Q(1), 0.01, 1.0,
                                                9.81)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertIsInstance(factor.noiseModel(),
                              gtsam.noiseModel.Constrained)

    def test_pendulum_factor_pk(self):
        factor = gtsam_unstable.PendulumFactorPk(P(1), Q(1), Q(2), 0.01, 1.0,
                                                 1.0, 9.81, 0.0)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)

    def test_pendulum_factor_pk1(self):
        factor = gtsam_unstable.PendulumFactorPk1(P(2), Q(1), Q(2), 0.01, 1.0,
                                                  1.0, 9.81, 0.0)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)

    def test_velocity_constraint3(self):
        factor = gtsam_unstable.VelocityConstraint3(X(1), X(2), V(1), 1.0)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertIsInstance(factor.noiseModel(),
                              gtsam.noiseModel.Constrained)

    def test_toa_factor(self):
        """TOAFactor reaches NoiseModelFactor via ExpressionFactor<double>."""
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.05)
        factor = gtsam_unstable.TOAFactor(T(1), gtsam.Point3(1.0, 0.0, 0.0),
                                          0.003, model)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertAlmostEqual(factor.noiseModel().sigma(), 0.05)


if __name__ == "__main__":
    unittest.main()
