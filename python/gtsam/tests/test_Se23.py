"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Se23 wrapper unit tests.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestSe23(GtsamTestCase):
    """Test the Se23 alias and wrapped factor instantiations."""

    @staticmethod
    def make_state(xi):
        return gtsam.Se23.Expmap(np.array(xi))

    def test_alias(self):
        """Se23 is exposed as the K=2 ExtendedPose3 alias."""
        pose = gtsam.Se23()
        self.assertEqual(pose.k(), 2)
        self.gtsamAssertEquals(pose, gtsam.ExtendedPose32(), 1e-12)
        self.gtsamAssertEquals(gtsam.Se23.Identity(), gtsam.ExtendedPose32.Identity(), 1e-12)

    def test_prior_and_between_factors(self):
        """Test wrapped PriorFactorSe23 and BetweenFactorSe23."""
        state1 = self.make_state([0.1, -0.2, 0.3, 1.0, 2.0, 3.0, -0.4, 0.5, -0.6])
        state2 = self.make_state([-0.3, 0.1, 0.2, 1.5, 1.0, 2.5, 0.2, -0.1, 0.7])
        model = gtsam.noiseModel.Unit.Create(9)

        prior_factor = gtsam.PriorFactorSe23(0, state1, model)
        self.gtsamAssertEquals(prior_factor.prior(), state1, 1e-9)

        between_measurement = state1.between(state2)
        between_factor = gtsam.BetweenFactorSe23(0, 1, between_measurement, model)
        self.gtsamAssertEquals(between_factor.measured(), between_measurement, 1e-9)

        values = gtsam.Values()
        values.insert(0, state1)
        values.insert(1, state2)

        self.assertAlmostEqual(prior_factor.error(values), 0.0, places=9)
        self.assertAlmostEqual(between_factor.error(values), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
