"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Simple unit test for custom robust noise model.
Author: Fan Jiang
"""
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestRobust(GtsamTestCase):

    def test_scalar_objective(self):
        """Vector loss and factor error sum scalar losses after whitening."""
        model = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(
                1.0, gtsam.noiseModel.mEstimator.Base.ReweightScheme.Scalar),
            gtsam.noiseModel.Diagonal.Sigmas(np.array([2.0, 3.0])))
        point = np.array([4.0, 6.0])
        factor = gtsam.PriorFactorPoint2(0, np.zeros(2), model)
        values = gtsam.Values()
        values.insert(0, point)
        self.assertAlmostEqual(model.loss(point), 3.0)
        self.assertAlmostEqual(factor.error(values), 3.0)
        # The scalar overload remains the loss of a squared distance.
        self.assertAlmostEqual(model.loss(8.0), np.sqrt(8.0) - 0.5)

    def test_RobustLossAndWeight(self):
        k = 10.0

        def custom_weight(e):
            abs_e = abs(e)
            return 1.0 if abs_e <= k else k / abs_e

        def custom_loss(e):
            abs_e = abs(e)
            return abs_e * abs_e / 2.0 if abs_e <= k else k * abs_e - k * k / 2.0

        custom_robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Custom(custom_weight, custom_loss,
                                               gtsam.noiseModel.mEstimator.Base.ReweightScheme.Scalar,
                                               "huber"),
            gtsam.noiseModel.Isotropic.Sigma(1, 2.0))
        f = gtsam.PriorFactorDouble(0, 1.0, custom_robust)
        v = gtsam.Values()
        v.insert(0, 0.0)

        self.assertAlmostEqual(f.error(v), 0.125)

if __name__ == "__main__":
    unittest.main()
