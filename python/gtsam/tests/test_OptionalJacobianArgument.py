"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for NoiseModelFactor's optional Jacobian argument.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


class TestUnwhitenedError(GtsamTestCase):
    """unwhitenedError takes gtsam::OptionalMatrixVecType H = nullptr."""

    @staticmethod
    def prior_problem():
        """A single Pose3 prior with a 1 m offset along z."""
        model = gtsam.noiseModel.Isotropic.Sigma(6, 1.0)
        factor = gtsam.PriorFactorPose3(X(1), gtsam.Pose3(), model)

        values = gtsam.Values()
        values.insert(X(1), gtsam.Pose3(gtsam.Rot3(),
                                        gtsam.Point3(0.0, 0.0, 1.0)))
        return factor, values

    def test_H_may_be_omitted(self):
        """The default must round-trip through None."""
        factor, values = self.prior_problem()

        error = factor.unwhitenedError(values)
        self.assertEqual(len(error), 6)
        # A pure 1 m offset with identity rotation, so the tangent vector has
        # unit norm regardless of sign or tangent-space ordering.
        self.assertAlmostEqual(np.linalg.norm(error), 1.0)

    def test_H_may_be_passed_explicitly_as_none(self):
        factor, values = self.prior_problem()

        error = factor.unwhitenedError(values, None)
        np.testing.assert_allclose(error, factor.unwhitenedError(values),
                                   atol=1e-12)

    def test_agrees_with_whitened_error(self):
        """Unit noise, so whitened and unwhitened coincide."""
        factor, values = self.prior_problem()

        np.testing.assert_allclose(factor.whitenedError(values),
                                   factor.unwhitenedError(values), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
