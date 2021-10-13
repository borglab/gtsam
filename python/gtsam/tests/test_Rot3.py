"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information
Rot3 unit tests.
Author: John Lambert
"""
# pylint: disable=no-name-in-module

import unittest

import numpy as np

import gtsam
from gtsam import Rot3
from gtsam.utils.test_case import GtsamTestCase


class TestRot3(GtsamTestCase):
    """Test selected Rot3 methods."""

    def test_axisangle(self) -> None:
        """Test .axisAngle() method."""
        # fmt: off
        R = np.array(
          [
            [ -0.999957, 0.00922903, 0.00203116],
            [ 0.00926964, 0.999739, 0.0208927],
            [ -0.0018374, 0.0209105, -0.999781]
          ])
        # fmt: on
        
        # get back angle in radians
        _, actual_angle = Rot3(R).axisAngle()
        expected_angle = 3.1396582
        np.testing.assert_almost_equal(actual_angle, expected_angle, 1e-7)


if __name__ == "__main__":
    unittest.main()
