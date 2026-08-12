"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Basis unit tests.
Author: Frank Dellaert & Gerry Chen (Python)
"""
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestBasis(GtsamTestCase):
    """
    Tests FitBasis python binding for FourierBasis, Chebyshev1Basis, Chebyshev2Basis, and Chebyshev2.
    
    It tests FitBasis by fitting to a ground-truth function that can be represented exactly in
    the basis, then checking that the regression (fit result) matches the function.  For the
    Chebyshev bases, the line y=x is used to generate the data while for Fourier, 0.7*cos(x) is
    used.
    """

    def setUp(self):
        self.N = 2
        self.x = [0., 0.5, 0.75]
        self.interpx = np.linspace(0., 1., 10)
        self.noise = gtsam.noiseModel.Unit.Create(1)

    def evaluate(self, basis, fitparams, x):
        """
        Until wrapper for Basis functors are ready,
        this is how to evaluate a basis function.
        """
        return basis.WeightMatrix(self.N, x) @ fitparams

    def fit_basis_helper(self, fitter, basis, f=lambda x: x):
        """Helper method to fit data to a specified fitter using a specified basis."""
        data = {x: f(x) for x in self.x}
        fit = fitter(data, self.noise, self.N)
        coeff = fit.parameters()
        interpy = self.evaluate(basis, coeff, self.interpx)
        return interpy

    def test_fit_basis_fourier(self):
        """Fit a Fourier basis."""

        f = lambda x: 0.7 * np.cos(x)
        interpy = self.fit_basis_helper(gtsam.FitBasisFourierBasis,
                                        gtsam.FourierBasis, f)
        # test a basis by checking that the fit result matches the function at x-values interpx.
        np.testing.assert_almost_equal(interpy,
                                       np.array([f(x) for x in self.interpx]),
                                       decimal=7)

    def test_fit_basis_chebyshev1basis(self):
        """Fit a Chebyshev1 basis."""

        f = lambda x: x
        interpy = self.fit_basis_helper(gtsam.FitBasisChebyshev1Basis,
                                        gtsam.Chebyshev1Basis, f)
        # test a basis by checking that the fit result matches the function at x-values interpx.
        np.testing.assert_almost_equal(interpy,
                                       np.array([f(x) for x in self.interpx]),
                                       decimal=7)

    def test_fit_basis_chebyshev2basis(self):
        """Fit a Chebyshev2 basis."""

        f = lambda x: x
        interpy = self.fit_basis_helper(gtsam.FitBasisChebyshev2Basis,
                                        gtsam.Chebyshev2Basis)
        # test a basis by checking that the fit result matches the function at x-values interpx.
        np.testing.assert_almost_equal(interpy,
                                       np.array([f(x) for x in self.interpx]),
                                       decimal=7)

    def test_fit_basis_chebyshev2(self):
        """Fit a Chebyshev2 pseudospectral basis."""

        f = lambda x: x
        interpy = self.fit_basis_helper(gtsam.FitBasisChebyshev2,
                                        gtsam.Chebyshev2)
        # test a basis by checking that the fit result matches the function at x-values interpx.
        np.testing.assert_almost_equal(interpy,
                                       np.array([f(x) for x in self.interpx]),
                                       decimal=7)


if __name__ == "__main__":
    unittest.main()
