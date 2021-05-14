"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

CustomFactor unit tests.
Author: Fan Jiang
"""
from typing import List
import unittest
from gtsam import Values, Pose2, CustomFactor

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestCustomFactor(GtsamTestCase):

    def test_new(self):
        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            return np.array([1, 0, 0])
        
        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0]), error_func)

    def test_call(self):

        expected_pose = Pose2(1, 1, 0)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]) -> np.ndarray:
            key0 = this.keys()[0]
            error = -v.atPose2(key0).localCoordinates(expected_pose)
            return error
        
        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0]), error_func)
        v = Values()
        v.insert(0, Pose2(1, 0, 0))
        e = cf.error(v)
        
        self.assertEqual(e, 0.5)
    
    def test_jacobian(self):
        """Tests if the factor result matches the GTSAM Pose2 unit test"""

        gT1 = Pose2(1, 2, np.pi/2)
        gT2 = Pose2(-1, 4, np.pi)

        expected = Pose2(2, 2, np.pi/2)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            # print(f"{this = },\n{v = },\n{len(H) = }")

            key0 = this.keys()[0]
            key1 = this.keys()[1]
            gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
            error = Pose2(0, 0, 0).localCoordinates(gT1.between(gT2))
            
            if len(H) > 0:
                result = gT1.between(gT2)
                H[0] = -result.inverse().AdjointMap()
                H[1] = np.eye(3)
            return error
        
        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0, 1]), error_func)
        v = Values()
        v.insert(0, gT1)
        v.insert(1, gT2)
        
        bf = gtsam.BetweenFactorPose2(0, 1, Pose2(0, 0, 0), noise_model)

        gf = cf.linearize(v)
        gf_b = bf.linearize(v)

        J_cf, b_cf = gf.jacobian()
        J_bf, b_bf = gf_b.jacobian()
        np.testing.assert_allclose(J_cf, J_bf)
        np.testing.assert_allclose(b_cf, b_bf)

if __name__ == "__main__":
    unittest.main()
