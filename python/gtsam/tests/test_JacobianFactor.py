"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

JacobianFactor unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestJacobianFactor(GtsamTestCase):

    def test_eliminate(self):
        # Recommended way to specify a matrix (see python/README)
        Ax2 = np.array(
           [[-5., 0.],
            [+0., -5.],
            [10., 0.],
            [+0., 10.]], order='F')

        # This is good too
        Al1 = np.array(
           [[5, 0],
            [0, 5],
            [0, 0],
            [0, 0]], dtype=float, order = 'F')

        # Not recommended for performance reasons, but should still work
        # as the wrapper should convert it to the correct type and storage order
        Ax1 = np.array(
           [[0,  0],  # f4
            [0,  0],  # f4
            [-10,  0],  # f2
            [0, -10]])  # f2

        x2 = 1
        l1 = 2
        x1 = 3

        # the RHS
        b2 = np.array([-1., 1.5, 2., -1.])
        sigmas = np.array([1., 1., 1., 1.])
        model4 = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        combined = gtsam.JacobianFactor(x2, Ax2, l1, Al1, x1, Ax1, b2, model4)

        # eliminate the first variable (x2) in the combined factor, destructive
        # !
        ord = gtsam.Ordering()
        ord.push_back(x2)
        actualCG, lf = combined.eliminate(ord)

        # create expected Conditional Gaussian
        R11 = np.array([[11.1803,  0.00],
                        [0.00, 11.1803]])
        S12 = np.array([[-2.23607, 0.00],
                        [+0.00, -2.23607]])
        S13 = np.array([[-8.94427, 0.00],
                        [+0.00, -8.94427]])
        d = np.array([2.23607, -1.56525])
        expectedCG = gtsam.GaussianConditional(
            x2, d, R11, l1, S12, x1, S13, gtsam.noiseModel.Unit.Create(2))
        # check if the result matches
        self.gtsamAssertEquals(actualCG, expectedCG, 1e-4)

        # the expected linear factor
        Bl1 = np.array([[4.47214, 0.00],
                        [0.00, 4.47214]])

        Bx1 = np.array(
            # x1
            [[-4.47214,  0.00],
             [+0.00, -4.47214]])

        # the RHS
        b1 = np.array([0.0, 0.894427])

        model2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([1., 1.]))
        expectedLF = gtsam.JacobianFactor(l1, Bl1, x1, Bx1, b1, model2)

        # check if the result matches the combined (reduced) factor
        self.gtsamAssertEquals(lf, expectedLF, 1e-4)

if __name__ == "__main__":
    unittest.main()
