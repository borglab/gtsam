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
    def test_jacobian(self):
        cam1 = gtsam.PinholeCameraCal3Bundler()
        cam1.deserialize(
            '22 serialization::archive 19 1 0\n0 0 0 0 0 1 0\n1 1 0\n2 9.99727427959442139e-01 6.30191620439291000e-03 2.24814359098672867e-02 5.97546668723225594e-03 -9.99876141548156738e-01 1.45585928112268448e-02 2.25703977048397064e-02 -1.44202867522835732e-02 -9.99641239643096924e-01 0 0 -5.81446531765312802e-02 -3.64078342172925590e-02 -5.63949743097517997e-01 1 0\n3 0 0 5.18692016601562500e+02 5.18692016601562500e+02 0.00000000000000000e+00 0.00000000000000000e+00 0.00000000000000000e+00 -1.14570140838623047e-01 -3.44798192381858826e-02 1.00000000000000008e-05\n'
        )

        # order is important because Eigen is column major!
        Dpose = np.zeros((2, 6), order='F')
        Dpoint = np.zeros((2, 3), order='F')
        Dcal = np.zeros((2, 3), order='F')
        cam1.project(np.array([0.43350768, 0.0305741, -1.93050155]), Dpose,
                     Dpoint, Dcal)

        self.gtsamAssertEquals(
            Dpoint,
            np.array([[358.22984814, -0.58837638, 128.85360812],
                      [3.58714683, -371.03278204, -16.89571148]]))

        Dpose_expected = np.array([[
                -3.97279895, -553.22184659, -16.40213844, -361.03694491,
                -0.98773192, 120.76242884
            ],
            [
                512.13105406, 3.97279895, -171.21912901,
                -0.98773192, -371.25308927, -11.56857932
            ]])

        self.gtsamAssertEquals(
            Dpose,
            Dpose_expected)

        self.gtsamAssertEquals(
            Dcal,
            np.array([[0.33009787, 19.60464375, 2.214697],
                      [-0.03162211, -1.87804997, -0.21215951]]))

if __name__ == "__main__":
    unittest.main()
