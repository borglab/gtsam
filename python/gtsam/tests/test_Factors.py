"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for various factors.

Author: Varun Agrawal
"""
import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


class TestNonlinearEquality2Factor(GtsamTestCase):
    """
    Test various instantiations of NonlinearEquality2.
    """

    def test_point3(self):
        """Test for Point3 version."""
        factor = gtsam.NonlinearEquality2Point3(0, 1)
        error = factor.evaluateError(gtsam.Point3(0, 0, 0),
                                     gtsam.Point3(0, 0, 0))

        np.testing.assert_allclose(error, np.zeros(3))


class TestJacobianFactor(GtsamTestCase):
    """Test JacobianFactor"""

    def test_gaussian_factor_graph(self):
        """Test construction from GaussianFactorGraph."""
        gfg = gtsam.GaussianFactorGraph()
        jf = gtsam.JacobianFactor(gfg)
        self.assertIsInstance(jf, gtsam.JacobianFactor)

        nfg = gtsam.NonlinearFactorGraph()
        nfg.push_back(gtsam.PriorFactorDouble(1, 0.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0)))
        values = gtsam.Values()
        values.insert(1, 0.0)
        gfg = nfg.linearize(values)
        jf = gtsam.JacobianFactor(gfg)
        self.assertIsInstance(jf, gtsam.JacobianFactor)

class TestHessianFactor(GtsamTestCase):
    """Test HessianFactor"""

    def test_gaussian_factor_graph(self):
        """Test construction from GaussianFactorGraph."""
        gfg = gtsam.GaussianFactorGraph()
        hf = gtsam.HessianFactor(gfg)
        self.assertIsInstance(hf, gtsam.HessianFactor)

        nfg = gtsam.NonlinearFactorGraph()
        nfg.push_back(gtsam.PriorFactorDouble(1, 0.0, gtsam.noiseModel.Isotropic.Sigma(1, 1.0)))
        values = gtsam.Values()
        values.insert(1, 0.0)
        gfg = nfg.linearize(values)
        hf = gtsam.HessianFactor(gfg)
        self.assertIsInstance(hf, gtsam.HessianFactor)

    def test_hessian_nary_factor(self):
        """Test construction from n-ary factor."""
        n = 4 # number of edges on factor
        rng = np.random.default_rng(42)
        A = rng.random((n*2, n*2))
        Gfull = A @ A.T # Symmetric PD matrix
        gfull = rng.random(n*2)
        G11 = Gfull[:2,:2]
        G12 = Gfull[:2,2:4]
        G13 = Gfull[:2,4:6]
        G14 = Gfull[:2,6:8]
        G22 = Gfull[2:4,2:4]
        G23 = Gfull[2:4,4:6]
        G24 = Gfull[2:4,6:8]
        G33 = Gfull[4:6,4:6]
        G34 = Gfull[4:6,6:8]
        G44 = Gfull[6:8,6:8]
        g1 = gfull[:2]
        g2 = gfull[2:4]
        g3 = gfull[4:6]
        g4 = gfull[6:8]
        f = 1.0

        # Unary Factors
        hf_unary = gtsam.HessianFactor(X(0), G11, g1, f)

        self.gtsamAssertEquals(hf_unary.augmentedInformation()[:-1,:-1], G11)
        self.gtsamAssertEquals(hf_unary.augmentedInformation()[:-1,-1], g1)
        self.assertEqual(float(hf_unary.augmentedInformation()[-1,-1]), f)

        # Binary Factors
        hf_binary = gtsam.HessianFactor(X(0), X(1), G11, G12, g1, G22, g2, f)
        G_binary = np.block([
            [G11, G12],
            [G12.T, G22]
        ])
        self.gtsamAssertEquals(hf_binary.augmentedInformation()[:-1,:-1], G_binary)
        self.gtsamAssertEquals(hf_binary.augmentedInformation()[:-1,-1], np.block([g1, g2]))
        self.assertEqual(float(hf_binary.augmentedInformation()[-1,-1]), f)

        # Ternary Factors
        hf_ternary1 = gtsam.HessianFactor(X(0), X(1), X(2), G11, G12, G13, g1, G22, G23, g2, G33, g3, f)
        G_ternary = np.block([
            [G11, G12, G13],
            [G12.T, G22, G23],
            [G13.T, G23.T, G33]
        ])
        g_ternary = np.block([g1, g2, g3])
        self.gtsamAssertEquals(hf_ternary1.augmentedInformation()[:-1,:-1], G_ternary)
        self.gtsamAssertEquals(hf_ternary1.augmentedInformation()[:-1,-1], g_ternary)
        self.assertEqual(float(hf_ternary1.augmentedInformation()[-1,-1]), f)

        aug_info_ternary = np.zeros((3 * 2 + 1, 3 * 2 + 1))
        aug_info_ternary[:-1, :-1] = G_ternary
        aug_info_ternary[:-1, -1] = g_ternary
        aug_info_ternary[-1, :-1] = g_ternary
        aug_info_ternary[-1, -1] = f
        hf_ternary2 = gtsam.HessianFactor(
            [X(0), X(1), X(2)],
            gtsam.SymmetricBlockMatrix([2, 2, 2, 1], aug_info_ternary)
        )
        self.gtsamAssertEquals(hf_ternary1, hf_ternary2)

        # Quaternary Factors
        aug_info_quaternary = np.zeros((n * 2 + 1, n * 2 + 1))
        aug_info_quaternary[:-1, :-1] = Gfull
        aug_info_quaternary[:-1, -1] = gfull
        aug_info_quaternary[-1, :-1] = gfull
        aug_info_quaternary[-1, -1] = f
        hf_quaternary = gtsam.HessianFactor(
            [X(0), X(1), X(2), X(3)],
            gtsam.SymmetricBlockMatrix([2, 2, 2, 2, 1], aug_info_quaternary),
        )
        self.gtsamAssertEquals(hf_quaternary.augmentedInformation()[:-1,:-1], Gfull)
        self.gtsamAssertEquals(hf_quaternary.augmentedInformation()[:-1,-1], gfull)
        self.assertEqual(float(hf_quaternary.augmentedInformation()[-1,-1]), f)

if __name__ == "__main__":
    unittest.main()
