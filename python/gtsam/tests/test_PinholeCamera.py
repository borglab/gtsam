"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

PinholeCamera unit tests.
Author: Fan Jiang
"""
import unittest
from math import pi

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestPinholeCamera(GtsamTestCase):
    """
    Tests if we can correctly get the camera Jacobians in Python
    """
    def test_jacobian(self):
        cam1 = gtsam.PinholeCameraCal3Bundler()

        # order is important because Eigen is column major!
        Dpose = np.zeros((2, 6), order='F')
        Dpoint = np.zeros((2, 3), order='F')
        Dcal = np.zeros((2, 3), order='F')
        cam1.project(np.array([1, 1, 1]), Dpose, Dpoint, Dcal)

        self.gtsamAssertEquals(Dpoint, np.array([[1, 0, -1], [0, 1, -1]]))

        self.gtsamAssertEquals(
            Dpose,
            np.array([
                [1., -2., 1., -1., 0., 1.],  #
                [2., -1., -1., 0., -1., 1.]
            ]))

        self.gtsamAssertEquals(Dcal, np.array([[1., 2., 4.], [1., 2., 4.]]))


if __name__ == "__main__":
    unittest.main()
