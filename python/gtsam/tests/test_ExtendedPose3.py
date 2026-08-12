"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

ExtendedPose3 wrapper unit tests.
"""

import unittest

import numpy as np

import gtsam
from gtsam import Rot3
from gtsam.utils.test_case import GtsamTestCase


class TestExtendedPose3(GtsamTestCase):
    """Test wrapped ExtendedPose3<K> static instantiations."""

    @staticmethod
    def class_for_k(k: int):
        return getattr(gtsam, f"ExtendedPose3{k}")

    @staticmethod
    def make_dynamic_state(xi):
        return gtsam.ExtendedPose3d.Expmap(np.array(xi))

    def test_constructors_static_k(self):
        """All requested static-K classes are wrapped and default-constructible."""
        for k in (2, 3, 4, 6):
            cls = self.class_for_k(k)
            pose = cls()
            self.assertEqual(pose.k(), k)
            self.gtsamAssertEquals(pose, cls.Identity(), 1e-12)

    def test_k6_lie_group_and_matrix_lie_group(self):
        """Test selected MatrixLieGroup operations on K=6."""
        cls = self.class_for_k(6)

        xi = np.array([
            0.11, -0.07, 0.05,
            0.30, -0.40, 0.10,
            -0.20, 0.60, -0.50,
            0.70, -0.10, 0.20,
            -0.40, 0.30, 0.80,
            0.50, -0.20, -0.30,
            0.10, 0.20, -0.60,
        ])
        yi = np.array([
            -0.06, 0.04, 0.02,
            0.10, -0.15, 0.05,
            -0.12, 0.18, 0.07,
            0.22, -0.09, 0.03,
            -0.14, 0.11, 0.08,
            0.06, 0.13, -0.17,
            -0.05, 0.09, 0.04,
        ])

        hat = cls.Hat(xi)
        np.testing.assert_allclose(cls.Vee(hat), xi, atol=1e-9)

        p = cls.Expmap(xi)
        q = cls.Expmap(yi)
        np.testing.assert_allclose(cls.Logmap(p), xi, atol=1e-9)

        composed = p * q
        np.testing.assert_allclose(composed.matrix(), p.matrix() @ q.matrix(), atol=1e-9)
        self.gtsamAssertEquals(composed, p.compose(q), 1e-9)
        self.gtsamAssertEquals(p.between(q), p.inverse().compose(q), 1e-9)
        self.gtsamAssertEquals(p.inverse() * p, cls.Identity(), 1e-9)

        self.assertEqual(cls.Dim(), 21)
        self.assertEqual(p.dim(), 21)
        self.assertEqual(p.k(), 6)
        self.assertEqual(p.vec().shape[0], 81)

    def test_k6_constructor_with_components(self):
        """Construct K=6 from (Rot3, x) and access x(i)/xMatrix()."""
        cls = self.class_for_k(6)
        x = np.array([
            [1.0, 4.0, -1.0, 0.5, 2.1, -0.3],
            [2.0, 5.0, 0.5, -1.2, 3.3, 0.8],
            [3.0, 6.0, 2.0, 1.4, -2.7, 0.6],
        ])
        pose = cls(Rot3(), x)

        np.testing.assert_allclose(pose.xMatrix(), x, atol=1e-12)
        for i in range(6):
            np.testing.assert_allclose(pose.x(i), x[:, i], atol=1e-12)

    def test_dynamic_alias(self):
        """Dynamic-k alias is wrapped as ExtendedPose3d."""
        cls = gtsam.ExtendedPose3d
        pose = cls(5)
        self.assertEqual(pose.k(), 5)
        self.assertEqual(pose.dim(), 18)
        self.gtsamAssertEquals(pose, cls.Identity(5), 1e-12)

        x = np.array([
            [1.0, 0.1, -0.2, 0.3, 0.4],
            [0.5, -0.6, 0.7, -0.8, 0.9],
            [1.1, 1.2, -1.3, 1.4, -1.5],
        ])
        expected = cls(Rot3(), x)
        np.testing.assert_allclose(expected.xMatrix(), x, atol=1e-12)
        self.gtsamAssertEquals(expected, cls(expected.matrix()), 1e-12)

    def test_dynamic_prior_and_between_factors(self):
        """Dynamic-k type supports wrapped PriorFactor and BetweenFactor."""
        state1 = self.make_dynamic_state([
            0.1, -0.2, 0.3,
            1.0, 2.0, 3.0,
            -0.4, 0.5, -0.6,
            0.7, -0.8, 0.9,
        ])
        state2 = self.make_dynamic_state([
            -0.3, 0.1, 0.2,
            1.5, 1.0, 2.5,
            0.2, -0.1, 0.7,
            -0.6, 0.4, 0.3,
        ])
        model = gtsam.noiseModel.Unit.Create(12)

        prior_factor = gtsam.PriorFactorExtendedPose3d(0, state1, model)
        self.gtsamAssertEquals(prior_factor.prior(), state1, 1e-9)

        between_measurement = state1.between(state2)
        between_factor = gtsam.BetweenFactorExtendedPose3d(
            0, 1, between_measurement, model
        )
        self.gtsamAssertEquals(between_factor.measured(), between_measurement, 1e-9)

        values = gtsam.Values()
        values.insert(0, state1)
        values.insert(1, state2)

        self.assertAlmostEqual(prior_factor.error(values), 0.0, places=9)
        self.assertAlmostEqual(between_factor.error(values), 0.0, places=9)

    @unittest.skipUnless(hasattr(gtsam.ExtendedPose36, "serialize"), "Serialization not enabled")
    def test_serialization_k6(self):
        """Serialization works when boost serialization is enabled."""
        cls = self.class_for_k(6)
        expected = cls.Expmap(np.array([
            0.01, -0.02, 0.03,
            0.04, -0.05, 0.06,
            0.07, -0.08, 0.09,
            -0.10, 0.11, -0.12,
            0.13, -0.14, 0.15,
            -0.16, 0.17, -0.18,
            0.19, -0.20, 0.21,
        ]))
        actual = cls()
        serialized = expected.serialize()
        actual.deserialize(serialized)
        self.gtsamAssertEquals(expected, actual, 1e-10)


if __name__ == "__main__":
    unittest.main()
