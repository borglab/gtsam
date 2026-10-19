"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for KnownLandmarkSquaredRangeFactor wrapper dimension validation.
"""

import unittest

import gtsam
import numpy as np


class TestKnownLandmarkSquaredRangeFactor(unittest.TestCase):
    """Verify valid and invalid landmark dimensions in the Python wrapper."""

    def test_valid_landmark_dimensions(self):
        """Both specializations accept correctly sized landmarks."""
        model = gtsam.noiseModel.Unit.Create(1)

        gtsam.KnownLandmarkSquaredRangeFactor2(
            0, np.zeros(2), 5.0, model
        )

        gtsam.KnownLandmarkSquaredRangeFactor3(
            0, np.zeros(3), 5.0, model
        )

    def test_invalid_landmark_dimensions(self):
        """Both specializations reject incorrectly sized landmarks."""
        model = gtsam.noiseModel.Unit.Create(1)

        cases = [
            (gtsam.KnownLandmarkSquaredRangeFactor2, 1),
            (gtsam.KnownLandmarkSquaredRangeFactor2, 3),
            (gtsam.KnownLandmarkSquaredRangeFactor3, 2),
            (gtsam.KnownLandmarkSquaredRangeFactor3, 4),
        ]

        for factor_type, size in cases:
            with self.subTest(size=size):
                with self.assertRaises(ValueError):
                    factor_type(0, np.zeros(size), 5.0, model)


if __name__ == "__main__":
    unittest.main()