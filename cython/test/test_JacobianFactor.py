import unittest
from gtsam import *
from math import *
import numpy as np
from gtsam_utils import Matrix, Vector

class TestJacobianFactor(unittest.TestCase):

    def test_eliminate(self):
        Ax2 = Matrix([
            [-5., 0.],
            [+0., -5.],
            [10., 0.],
            [+0., 10.]])

        Al1 = Matrix([
            [5., 0.],
            [0., 5.],
            [0., 0.],
            [0., 0.]])

        Ax1 = Matrix([
            [0.00,  0.],  # f4
            [0.00,  0.],  # f4
            [-10.,  0.],  # f2
            [0.00, -10.]])  # f2

        x2 = 1
        l1 = 2
        x1 = 3

        # the RHS
        b2 = Vector([-1., 1.5, 2., -1.])
        sigmas = Vector([1., 1., 1., 1.])
        model4 = noiseModel_Diagonal.Sigmas(sigmas)
        combined = JacobianFactor(x2, np.transpose(
            Ax2), l1, Al1, x1, Ax1, b2, model4)

        # eliminate the first variable (x2) in the combined factor, destructive
        # !
        ord = Ordering()
        ord.push_back(x2)
        actualCG, lf = combined.eliminate(ord)

        # create expected Conditional Gaussian
        R11 = Matrix([[11.1803,  0.00],
                      [0.00, 11.1803]])
        S12 = Matrix([[-2.23607, 0.00],
                      [+0.00, -2.23607] ])
        S13 = Matrix([[-8.94427, 0.00],
                      [+0.00, -8.94427]])
        d = Vector([2.23607, -1.56525])
        expectedCG = GaussianConditional(
            x2, d, R11, l1, S12, x1, S13, noiseModel_Unit.Create(2))
        # check if the result matches
        self.assertTrue(actualCG.equals(expectedCG, 1e-4))

        # the expected linear factor
        Bl1 = Matrix([
            [4.47214, 0.00],
            [0.00, 4.47214]])

        Bx1 = Matrix([
            # x1
            [-4.47214,  0.00],
            [+0.00, -4.47214]])

        # the RHS
        b1 = Vector([0.0, 0.894427])

        model2 = noiseModel_Diagonal.Sigmas(np.array([1., 1.]))
        expectedLF = JacobianFactor(l1, Bl1, x1, Bx1, b1, model2)

        # check if the result matches the combined (reduced) factor
        self.assertTrue(lf.equals(expectedLF, 1e-4))

if __name__ == "__main__":
    unittest.main()
