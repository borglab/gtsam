"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Chebyshev2 Basis using the GTSAM Python wrapper.
Converted from the C++ tests.
"""

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import Chebyshev2


# Define test functions f and fprime:
def f(x):
    return 3.0 * (x**3) - 2.0 * (x**2) + 5.0 * x - 11.0


def fprime(x):
    return 9.0 * (x**2) - 4.0 * x + 5.0


def Chebyshev2_vector(f, N, a=-1.0, b=1.0):
    points = Chebyshev2.Points(N, a, b)
    return np.array([f(x) for x in points])


class TestChebyshev2(GtsamTestCase):

    def test_Point(self):
        """Test that Chebyshev points are correctly calculated and symmetrical."""
        N = 5
        points = Chebyshev2.Points(N)
        expected = np.array([-1.0, -np.sqrt(2.0) / 2.0, 0.0, np.sqrt(2.0) / 2.0, 1.0])
        tol = 1e-15
        np.testing.assert_allclose(points, expected, rtol=0, atol=tol)

        # Check symmetry:
        p0 = Chebyshev2.Point(N, 0)
        p4 = Chebyshev2.Point(N, 4)
        p1 = Chebyshev2.Point(N, 1)
        p3 = Chebyshev2.Point(N, 3)
        self.assertAlmostEqual(p0, -p4, delta=tol)
        self.assertAlmostEqual(p1, -p3, delta=tol)

    def test_PointInInterval(self):
        """Test that Chebyshev points map correctly to arbitrary intervals [a,b]."""
        N = 5
        points = Chebyshev2.Points(N, 0, 20)
        expected = (
            np.array(
                [0.0, 1.0 - np.sqrt(2.0) / 2.0, 1.0, 1.0 + np.sqrt(2.0) / 2.0, 2.0]
            )
            * 10.0
        )
        tol = 1e-15
        np.testing.assert_allclose(points, expected, rtol=0, atol=tol)
        # Also check all-at-once:
        actual = Chebyshev2.Points(N, 0, 20)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=tol)

    def test_Decomposition(self):
        """Test fitting a linear function with Chebyshev basis."""
        # Create a sequence: dictionary mapping x -> y.
        sequence = {}
        for i in range(16):
            x_val = (1.0 / 16) * i - 0.99
            sequence[x_val] = x_val
        fit = gtsam.FitBasisChebyshev2(sequence, None, 3)
        params = fit.parameters()
        expected = np.array([-1.0, 0.0, 1.0])
        np.testing.assert_allclose(params, expected, rtol=0, atol=1e-4)

    def test_DifferentiationMatrix3(self):
        """Test the 3×3 differentiation matrix against known values."""
        N = 3
        # Expected differentiation matrix (from chebfun) then multiplied by -1.
        expected = np.array([[1.5, -2.0, 0.5], [0.5, -0.0, -0.5], [-0.5, 2.0, -1.5]])
        expected = -expected
        actual = Chebyshev2.DifferentiationMatrix(N)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-4)

    def test_DerivativeMatrix6(self):
        """Test the 6×6 differentiation matrix against known values."""
        N = 6
        expected = np.array(
            [
                [8.5000, -10.4721, 2.8944, -1.5279, 1.1056, -0.5000],
                [2.6180, -1.1708, -2.0000, 0.8944, -0.6180, 0.2764],
                [-0.7236, 2.0000, -0.1708, -1.6180, 0.8944, -0.3820],
                [0.3820, -0.8944, 1.6180, 0.1708, -2.0000, 0.7236],
                [-0.2764, 0.6180, -0.8944, 2.0000, 1.1708, -2.6180],
                [0.5000, -1.1056, 1.5279, -2.8944, 10.4721, -8.5000],
            ]
        )
        expected = -expected
        actual = Chebyshev2.DifferentiationMatrix(N)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-4)

    def test_CalculateWeights(self):
        """Test interpolation weights for a cubic function at arbitrary points."""
        N = 32
        fvals = Chebyshev2_vector(f, N)
        x1, x2 = 0.7, -0.376
        w1 = Chebyshev2.CalculateWeights(N, x1)
        w2 = Chebyshev2.CalculateWeights(N, x2)
        self.assertAlmostEqual(w1.dot(fvals), f(x1), delta=1e-8)
        self.assertAlmostEqual(w2.dot(fvals), f(x2), delta=1e-8)

    def test_CalculateWeights2(self):
        """Test interpolation weights in arbitrary interval [a,b]."""
        N = 32
        a, b = 0.0, 10.0
        x1, x2 = 7.0, 4.12
        fvals = Chebyshev2_vector(f, N, a, b)
        w1 = Chebyshev2.CalculateWeights(N, x1, a, b)
        self.assertAlmostEqual(w1.dot(fvals), f(x1), delta=1e-8)
        w2 = Chebyshev2.CalculateWeights(N, x2, a, b)
        self.assertAlmostEqual(w2.dot(fvals), f(x2), delta=1e-8)

    def test_CalculateWeights_CoincidingPoint(self):
        """Test that weights are correctly computed when x coincides with a Chebyshev point."""
        N = 5
        coincidingPoint = Chebyshev2.Point(N, 1)
        w = Chebyshev2.CalculateWeights(N, coincidingPoint)
        tol = 1e-9
        for j in range(N):
            expected = 1.0 if j == 1 else 0.0
            self.assertAlmostEqual(w[j], expected, delta=tol)

    def test_DerivativeWeights(self):
        """Test derivative weights for polynomial function at arbitrary points."""
        N = 32
        fvals = Chebyshev2_vector(f, N)
        for x in [0.7, -0.376, 0.0]:
            dw = Chebyshev2.DerivativeWeights(N, x)
            self.assertAlmostEqual(dw.dot(fvals), fprime(x), delta=1e-9)
        x4 = Chebyshev2.Point(N, 3)
        dw4 = Chebyshev2.DerivativeWeights(N, x4)
        self.assertAlmostEqual(dw4.dot(fvals), fprime(x4), delta=1e-9)

    def test_DerivativeWeights2(self):
        """Test derivative weights in arbitrary interval [a,b]."""
        N = 32
        a, b = 0.0, 10.0
        x1, x2 = 5.0, 4.12
        fvals = Chebyshev2_vector(f, N, a, b)
        dw1 = Chebyshev2.DerivativeWeights(N, x1, a, b)
        self.assertAlmostEqual(dw1.dot(fvals), fprime(x1), delta=1e-8)
        dw2 = Chebyshev2.DerivativeWeights(N, x2, a, b)
        self.assertAlmostEqual(dw2.dot(fvals), fprime(x2), delta=1e-8)
        x3 = Chebyshev2.Point(N, 3, a, b)
        dw3 = Chebyshev2.DerivativeWeights(N, x3, a, b)
        self.assertAlmostEqual(dw3.dot(fvals), fprime(x3), delta=1e-8)

    def test_DerivativeWeightsDifferentiationMatrix(self):
        """Test that derivative weights match multiplication by differentiation matrix."""
        N6 = 6
        x1 = 0.311
        D6 = Chebyshev2.DifferentiationMatrix(N6)
        expected = Chebyshev2.CalculateWeights(N6, x1).dot(D6)
        actual = Chebyshev2.DerivativeWeights(N6, x1)
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)

        a, b, x2 = -3.0, 8.0, 5.05
        D6_2 = Chebyshev2.DifferentiationMatrix(N6, a, b)
        expected1 = Chebyshev2.CalculateWeights(N6, x2, a, b).dot(D6_2)
        actual1 = Chebyshev2.DerivativeWeights(N6, x2, a, b)
        np.testing.assert_allclose(actual1, expected1, rtol=0, atol=1e-12)

    def test_DerivativeWeights6(self):
        """Test that differentiating the identity function gives a constant."""
        N6 = 6
        D6 = Chebyshev2.DifferentiationMatrix(N6)
        x = Chebyshev2.Points(N6)  # ramp with slope 1
        ones = np.ones(N6)
        np.testing.assert_allclose(D6.dot(x), ones, rtol=0, atol=1e-9)

    def test_DerivativeWeights7(self):
        """Test that differentiating the identity function gives a constant (N=7)."""
        N7 = 7
        D7 = Chebyshev2.DifferentiationMatrix(N7)
        x = Chebyshev2.Points(N7)
        ones = np.ones(N7)
        np.testing.assert_allclose(D7.dot(x), ones, rtol=0, atol=1e-9)

    def test_IntegrationMatrix(self):
        """Test integration matrix properties and accuracy on polynomial functions."""
        N = 10
        a, b = 0.0, 10.0
        P = Chebyshev2.IntegrationMatrix(N, a, b)
        F = P.dot(np.ones(N))
        self.assertAlmostEqual(F[0], 0.0, delta=1e-9)
        points = Chebyshev2.Points(N, a, b)
        ramp = points - a
        np.testing.assert_allclose(F, ramp, rtol=0, atol=1e-9)
        fp = Chebyshev2_vector(fprime, N, a, b)
        F_est = P.dot(fp)
        self.assertAlmostEqual(F_est[0], 0.0, delta=1e-9)
        F_est += f(a)
        F_true = Chebyshev2_vector(f, N, a, b)
        np.testing.assert_allclose(F_est, F_true, rtol=0, atol=1e-9)
        D = Chebyshev2.DifferentiationMatrix(N, a, b)
        ff_est = D.dot(F_est)
        np.testing.assert_allclose(ff_est, fp, rtol=0, atol=1e-9)

    def test_IntegrationWeights7(self):
        """Test integration weights against known values for N=7."""
        N = 7
        actual = Chebyshev2.IntegrationWeights(N, -1, 1)
        expected = np.array(
            [
                0.0285714285714286,
                0.253968253968254,
                0.457142857142857,
                0.520634920634921,
                0.457142857142857,
                0.253968253968254,
                0.0285714285714286,
            ]
        )
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-9)
        self.assertAlmostEqual(np.sum(actual), 2.0, delta=1e-9)
        fp = Chebyshev2_vector(fprime, N)
        expectedF = f(1) - f(-1)
        self.assertAlmostEqual(actual.dot(fp), expectedF, delta=1e-9)
        P = Chebyshev2.IntegrationMatrix(N)
        p7 = P[-1, :]
        self.assertAlmostEqual(p7.dot(fp), expectedF, delta=1e-9)
        fvals = Chebyshev2_vector(f, N)
        self.assertAlmostEqual(p7.dot(fvals), actual.dot(fvals), delta=1e-9)

    def test_IntegrationWeights8(self):
        """Test integration weights against known values for N=8."""
        N = 8
        actual = Chebyshev2.IntegrationWeights(N, -1, 1)
        expected = np.array(
            [
                0.0204081632653061,
                0.190141007218208,
                0.352242423718159,
                0.437208405798326,
                0.437208405798326,
                0.352242423718159,
                0.190141007218208,
                0.0204081632653061,
            ]
        )
        np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-9)
        self.assertAlmostEqual(np.sum(actual), 2.0, delta=1e-9)

    def test_DoubleIntegrationWeights(self):
        """Test double integration weights for constant function (N=7)."""
        N = 7
        a, b = 0.0, 10.0
        P = Chebyshev2.IntegrationMatrix(N, a, b)
        ones = np.ones(N)
        w = Chebyshev2.DoubleIntegrationWeights(N, a, b)
        self.assertAlmostEqual(w.dot(ones), b * b / 2.0, delta=1e-9)

    def test_DoubleIntegrationWeights2(self):
        """Test double integration weights for constant function (N=8)."""
        N = 8
        a, b = 0.0, 3.0
        P = Chebyshev2.IntegrationMatrix(N, a, b)
        ones = np.ones(N)
        w = Chebyshev2.DoubleIntegrationWeights(N, a, b)
        self.assertAlmostEqual(w.dot(ones), b * b / 2.0, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
