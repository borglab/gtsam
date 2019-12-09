import unittest
import numpy as np

import gtsam


class TestJacobianFactor(unittest.TestCase):
    def test_eliminate(self):
        Ax2 = np.array([[-5.0, 0.0], [+0.0, -5.0], [10.0, 0.0], [+0.0, 10.0]])

        Al1 = np.array([[5, 0], [0, 5], [0, 0], [0, 0]])
        Ax1 = np.array([[0, 0], [0, 0], [-10, 0], [0, -10]])

        x2 = 1
        l1 = 2
        x1 = 3

        # the RHS
        b2 = np.array([-1.0, 1.5, 2.0, -1.0])
        sigmas = np.array([1.0, 1.0, 1.0, 1.0])
        model4 = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        combined = gtsam.JacobianFactor(x2, Ax2, l1, Al1, x1, Ax1, b2, model4)

        # eliminate the first variable (x2) in the combined factor, destructive
        # !
        ord = gtsam.Ordering()
        ord.push_back(x2)
        actualCG, lf = combined.eliminate(ord)

        # create expected Conditional Gaussian
        R11 = np.array([[11.1803, 0.00], [0.00, 11.1803]])
        S12 = np.array([[-2.23607, 0.00], [+0.00, -2.23607]])
        S13 = np.array([[-8.94427, 0.00], [+0.00, -8.94427]])
        d = np.array([2.23607, -1.56525])
        expectedCG = gtsam.GaussianConditional(
            x2, d, R11, l1, S12, x1, S13, gtsam.noiseModel.Unit.Create(2)
        )
        # check if the result matches
        self.assertTrue(actualCG.equals(expectedCG, 1e-4))

        # the expected linear factor
        Bl1 = np.array([[4.47214, 0.00], [0.00, 4.47214]])

        Bx1 = np.array(
            # x1
            [[-4.47214, 0.00], [+0.00, -4.47214]]
        )

        # the RHS
        b1 = np.array([0.0, 0.894427])

        model2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0, 1.0]))
        expectedLF = gtsam.JacobianFactor(l1, Bl1, x1, Bx1, b1, model2)

        # check if the result matches the combined (reduced) factor
        self.assertTrue(lf.equals(expectedLF, 1e-4))


if __name__ == "__main__":
    unittest.main()
