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

import numpy as np

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

    def test_jacobian_vector_from_gtsam(self):
        """Unstable factors accept the globally registered JacobianVector."""
        model = gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        factor = gtsam_unstable.BetweenFactorEMPose2(
            0, 1, gtsam.Pose2(), model, model, 0.5, 0.5
        )
        values = gtsam.Values()
        values.insert(0, gtsam.Pose2())
        values.insert(1, gtsam.Pose2())
        jacobians = gtsam.JacobianVector(
            [np.empty((0, 0)), np.empty((0, 0))]
        )

        factor.whitenedError(values, jacobians)

        self.assertEqual(len(jacobians), 2)
        self.assertEqual(jacobians[0].shape, (6, 3))
        self.assertEqual(jacobians[1].shape, (6, 3))


if __name__ == "__main__":
    unittest.main()
