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
        """Test the creation of a new CustomFactor"""

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            """Minimal error function stub"""
            return np.array([1, 0, 0])

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0]), error_func)

    def test_new_keylist(self):
        """Test the creation of a new CustomFactor"""

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            """Minimal error function stub"""
            return np.array([1, 0, 0])

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, [0], error_func)

    def test_call(self):
        """Test if calling the factor works (only error)"""
        expected_pose = Pose2(1, 1, 0)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]) -> np.ndarray:
            """Minimal error function with no Jacobian"""
            key0 = this.keys()[0]
            error = -v.atPose2(key0).localCoordinates(expected_pose)
            return error

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, [0], error_func)
        v = Values()
        v.insert(0, Pose2(1, 0, 0))
        e = cf.error(v)

        self.assertEqual(e, 0.5)

    def test_jacobian(self):
        """Tests if the factor result matches the GTSAM Pose2 unit test"""

        gT1 = Pose2(1, 2, np.pi / 2)
        gT2 = Pose2(-1, 4, np.pi)

        expected = Pose2(2, 2, np.pi / 2)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            """
            the custom error function. One can freely use variables captured
            from the outside scope. Or the variables can be acquired by indexing `v`.
            Jacobian is passed by modifying the H array of numpy matrices.
            """

            key0 = this.keys()[0]
            key1 = this.keys()[1]
            gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
            error = expected.localCoordinates(gT1.between(gT2))

            if H is not None:
                result = gT1.between(gT2)
                H[0] = -result.inverse().AdjointMap()
                H[1] = np.eye(3)
            return error

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0, 1]), error_func)
        v = Values()
        v.insert(0, gT1)
        v.insert(1, gT2)

        bf = gtsam.BetweenFactorPose2(0, 1, expected, noise_model)

        gf = cf.linearize(v)
        gf_b = bf.linearize(v)

        J_cf, b_cf = gf.jacobian()
        J_bf, b_bf = gf_b.jacobian()
        np.testing.assert_allclose(J_cf, J_bf)
        np.testing.assert_allclose(b_cf, b_bf)

    def test_printing(self):
        """Tests if the factor result matches the GTSAM Pose2 unit test"""
        gT1 = Pose2(1, 2, np.pi / 2)
        gT2 = Pose2(-1, 4, np.pi)

        def error_func(this: CustomFactor, v: gtsam.Values, _: List[np.ndarray]):
            """Minimal error function stub"""
            return np.array([1, 0, 0])

        noise_model = gtsam.noiseModel.Unit.Create(3)
        from gtsam.symbol_shorthand import X
        cf = CustomFactor(noise_model, [X(0), X(1)], error_func)

        cf_string = """CustomFactor on x0, x1
  noise model: unit (3) 
"""
        self.assertEqual(cf_string, repr(cf))

    def test_no_jacobian(self):
        """Tests that we will not calculate the Jacobian if not requested"""

        gT1 = Pose2(1, 2, np.pi / 2)
        gT2 = Pose2(-1, 4, np.pi)

        expected = Pose2(2, 2, np.pi / 2)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            """
            Error function that mimics a BetweenFactor
            :param this: reference to the current CustomFactor being evaluated
            :param v: Values object
            :param H: list of references to the Jacobian arrays
            :return: the non-linear error
            """
            key0 = this.keys()[0]
            key1 = this.keys()[1]
            gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
            error = expected.localCoordinates(gT1.between(gT2))

            self.assertTrue(H is None)  # Should be true if we only request the error

            if H is not None:
                result = gT1.between(gT2)
                H[0] = -result.inverse().AdjointMap()
                H[1] = np.eye(3)
            return error

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, gtsam.KeyVector([0, 1]), error_func)
        v = Values()
        v.insert(0, gT1)
        v.insert(1, gT2)

        bf = gtsam.BetweenFactorPose2(0, 1, expected, noise_model)

        e_cf = cf.error(v)
        e_bf = bf.error(v)
        np.testing.assert_allclose(e_cf, e_bf)

    def test_optimization(self):
        """Tests if a factor graph with a CustomFactor can be properly optimized"""
        gT1 = Pose2(1, 2, np.pi / 2)
        gT2 = Pose2(-1, 4, np.pi)

        expected = Pose2(2, 2, np.pi / 2)

        def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
            """
            Error function that mimics a BetweenFactor
            :param this: reference to the current CustomFactor being evaluated
            :param v: Values object
            :param H: list of references to the Jacobian arrays
            :return: the non-linear error
            """
            key0 = this.keys()[0]
            key1 = this.keys()[1]
            gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
            error = expected.localCoordinates(gT1.between(gT2))

            if H is not None:
                result = gT1.between(gT2)
                H[0] = -result.inverse().AdjointMap()
                H[1] = np.eye(3)
            return error

        noise_model = gtsam.noiseModel.Unit.Create(3)
        cf = CustomFactor(noise_model, [0, 1], error_func)

        fg = gtsam.NonlinearFactorGraph()
        fg.add(cf)
        fg.add(gtsam.PriorFactorPose2(0, gT1, noise_model))

        v = Values()
        v.insert(0, Pose2(0, 0, 0))
        v.insert(1, Pose2(0, 0, 0))

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(fg, v, params)
        result = optimizer.optimize()

        self.gtsamAssertEquals(result.atPose2(0), gT1, tol=1e-5)
        self.gtsamAssertEquals(result.atPose2(1), gT2, tol=1e-5)


if __name__ == "__main__":
    unittest.main()
