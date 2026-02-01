"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

FrobeniusFactor unit tests.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, import-error, invalid-name
import unittest

import numpy as np

from gtsam import (
    SL4,
    SO3,
    SO4,
    FrobeniusBetweenFactorGal3,
    FrobeniusBetweenFactorNLSimilarity2,
    FrobeniusBetweenFactorNLSimilarity3,
    FrobeniusBetweenFactorNLSL4,
    FrobeniusBetweenFactorSO4,
    FrobeniusFactorGal3,
    FrobeniusFactorSL4,
    FrobeniusFactorSimilarity2,
    FrobeniusFactorSimilarity3,
    FrobeniusFactorSO4,
    Gal3,
    Rot3,
    ShonanFactor3,
    Similarity2,
    Similarity3,
    SOn,
)

id = SO4()
v1 = np.array([0, 0, 0, 0.1, 0, 0])
Q1 = SO4.Expmap(v1)
v2 = np.array([0, 0, 0, 0.01, 0.02, 0.03])
Q2 = SO4.Expmap(v2)

P1_sim2 = Similarity2.Expmap(np.array([0.1, 0.2, 0.3, 0.4]))
P2_sim2 = Similarity2.Expmap(np.array([0.2, 0.3, 0.4, 0.5]))

P1_sim3 = Similarity3.Expmap(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
P2_sim3 = Similarity3.Expmap(np.array([0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]))

G1_gal3 = Gal3(Rot3.Rz(0.1), np.array([0.2, 0.3, 0.4]), np.array([0.5, 0.6, 0.7]), 0.8)
G2_gal3 = Gal3(Rot3.Rz(0.2), np.array([0.3, 0.4, 0.5]), np.array([0.6, 0.7, 0.8]), 0.9)

# Define SL4 transformations

id_sl4 = SL4()
T_matrix1 = np.array([[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]])
G1_sl4 = SL4(T_matrix1)

T_matrix2 = np.array([[1, 0, 0, 4], [0, 1, 0, 5], [0, 0, 1, 6], [0, 0, 0, 1]])
G2_sl4 = SL4(T_matrix2)


class TestFrobeniusFactorSO4(unittest.TestCase):
    """Test FrobeniusFactor factors."""

    def test_frobenius_factor(self):
        """Test creation of a factor that calculates the Frobenius norm."""
        factor = FrobeniusFactorSO4(1, 2)
        actual = factor.evaluateError(Q1, Q2)
        expected = (Q2.matrix() - Q1.matrix()).transpose().reshape((16,))
        np.testing.assert_array_equal(actual, expected)

    def test_frobenius_between_factor(self):
        """Test creation of a Frobenius BetweenFactor."""
        factor = FrobeniusBetweenFactorSO4(1, 2, Q1.between(Q2))
        actual = factor.evaluateError(Q1, Q2)
        expected = np.zeros((16,))
        np.testing.assert_array_almost_equal(actual, expected)

    def test_frobenius_wormhole_factor(self):
        """Test creation of a factor that calculates Shonan error."""
        R1 = SO3.Expmap(v1[3:])
        R2 = SO3.Expmap(v2[3:])
        factor = ShonanFactor3(1, 2, Rot3(R1.between(R2).matrix()), p=4)
        I4 = SOn(4)
        Q1 = I4.retract(v1)
        Q2 = I4.retract(v2)
        actual = factor.evaluateError(Q1, Q2)
        expected = np.zeros((12,))
        np.testing.assert_array_almost_equal(actual, expected, decimal=4)


class TestFrobeniusFactorSimilarity2(unittest.TestCase):
    def test_frobenius_factor(self):
        factor = FrobeniusFactorSimilarity2(1, 2)
        actual = factor.evaluateError(P1_sim2, P2_sim2)
        expected = (P2_sim2.matrix() - P1_sim2.matrix()).transpose().reshape((9,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_frobenius_between_factor(self):
        factor = FrobeniusBetweenFactorNLSimilarity2(1, 2, P1_sim2.between(P2_sim2))
        actual = factor.evaluateError(P1_sim2, P2_sim2)
        expected = np.zeros((9,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)


class TestFrobeniusFactorSimilarity3(unittest.TestCase):
    def test_frobenius_factor(self):
        factor = FrobeniusFactorSimilarity3(1, 2)
        actual = factor.evaluateError(P1_sim3, P2_sim3)
        expected = (P2_sim3.matrix() - P1_sim3.matrix()).transpose().reshape((16,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_frobenius_between_factor(self):
        factor = FrobeniusBetweenFactorNLSimilarity3(1, 2, P1_sim3.between(P2_sim3))
        actual = factor.evaluateError(P1_sim3, P2_sim3)
        expected = np.zeros((16,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)


class TestFrobeniusFactorGal3(unittest.TestCase):
    def test_frobenius_factor(self):
        factor = FrobeniusFactorGal3(1, 2)
        actual = factor.evaluateError(G1_gal3, G2_gal3)
        expected = (G2_gal3.matrix() - G1_gal3.matrix()).transpose().reshape((25,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_frobenius_between_factor(self):
        factor = FrobeniusBetweenFactorGal3(1, 2, G1_gal3.between(G2_gal3))
        actual = factor.evaluateError(G1_gal3, G2_gal3)
        expected = np.zeros((25,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)


class TestFrobeniusFactorSL4(unittest.TestCase):
    def test_frobenius_factor(self):
        """Test Frobenius factor for SL4."""
        factor = FrobeniusFactorSL4(1, 2)  # Replace with appropriate SL4 factor class
        actual = factor.evaluateError(G1_sl4, G2_sl4)
        expected = (G2_sl4.matrix() - G1_sl4.matrix()).transpose().reshape((16,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_frobenius_between_factor(self):
        """Test Frobenius BetweenFactor for SL4."""
        factor = FrobeniusBetweenFactorNLSL4(
            1, 2, G1_sl4.between(G2_sl4)
        )  # Replace with appropriate SL4 factor class
        actual = factor.evaluateError(G1_sl4, G2_sl4)
        expected = np.zeros((16,))
        np.testing.assert_allclose(actual, expected, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
