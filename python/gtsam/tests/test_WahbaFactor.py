"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for the chordal Wahba factor wrapper.
"""

import unittest

import gtsam
import numpy as np


class TestWahbaFactor(unittest.TestCase):
    """Verify the wrapped Unit3 direction and Rot3 frame conventions."""

    def test_exact_direction_correspondence(self):
        """The aRb rotation maps bDirection to measured_aDirection."""
        aRb = gtsam.Rot3.RzRyRx(0.2, -0.1, 0.3)
        bDirection = gtsam.Unit3(np.array([1.2, -0.7, 0.5]))
        measured_aDirection = aRb.rotate(bDirection)
        model = gtsam.noiseModel.Unit.Create(3)

        factor = gtsam.WahbaFactor(
            0, bDirection, measured_aDirection, model
        )
        np.testing.assert_allclose(
            factor.evaluateError(aRb), np.zeros(3), atol=1e-12
        )


if __name__ == "__main__":
    unittest.main()
