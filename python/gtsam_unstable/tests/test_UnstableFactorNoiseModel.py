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
from gtsam.symbol_shorthand import T
from gtsam.utils.test_case import GtsamTestCase


class TestUnstableFactorNoiseModel(GtsamTestCase):
    """These factors derive from NoiseModelFactorT / ExpressionFactor."""

    def test_toa_factor(self):
        """TOAFactor reaches NoiseModelFactor via ExpressionFactor<double>."""
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.05)
        factor = gtsam_unstable.TOAFactor(T(1), gtsam.Point3(1.0, 0.0, 0.0),
                                          0.003, model)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertAlmostEqual(factor.noiseModel().sigma(), 0.05)


if __name__ == "__main__":
    unittest.main()
