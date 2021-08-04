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

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import B

class TestBasis(GtsamTestCase):
    def test_fit_basis(self):
        f = lambda x: x  # line y = x
        N = 2
        datax = [0., 0.5, 0.75]
        interpx = np.linspace(0., 1., 10)
        noise = gtsam.noiseModel.Unit.Create(1)
        def testBasis(fitter, basis, f=f):
            data = {x: f(x) for x in datax}
            fit = fitter(N, data, noise)
            coeff = fit.parameters()
            interpy = basis.WeightMatrix(N, interpx) @ coeff
            np.testing.assert_almost_equal(interpy, np.array([f(x) for x in interpx]), decimal=7)
        testBasis(gtsam.FitBasisFourierBasis, gtsam.FourierBasis, f=lambda x: 0.7*np.cos(x))
        testBasis(gtsam.FitBasisChebyshev1Basis, gtsam.Chebyshev1Basis)
        testBasis(gtsam.FitBasisChebyshev2Basis, gtsam.Chebyshev2Basis)
        testBasis(gtsam.FitBasisChebyshev2, gtsam.Chebyshev2)

if __name__ == "__main__":
    unittest.main()
