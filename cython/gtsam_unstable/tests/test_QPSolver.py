import unittest
import gtsam
import numpy as np

class TestQPSolver(unittest.TestCase):
    def test_QPSolver1(self):
        qp = gtsam.QP()
        qp.add_cost(gtsam.HessianFactor(1, 2, np.array([[2.0]]), np.array([[-1.]]), np.array([3.]), np.array([[2.]]), np.zeros(1), 10.0))
        qp.add_inequality(1, np.array([1.]), 2, np.array([1.]), 2.0)
        qp.add_inequality(1, np.array([-1.]), 0.0)
        qp.add_inequality(2, np.array([-1.]), 0.0)
        qp.add_inequality(1, np.array([1.]), 1.5)

        solver = gtsam.QPSolver(qp)
        values, duals = solver.optimize()

        expectedValues = gtsam.VectorValues()
        expectedValues.insert(1, np.array([1.5]))
        expectedValues.insert(2, np.array([0.5]))

        self.assertTrue(values.equals(expectedValues, 1e-5))

    def test_QPSolver2(self):
        qp = gtsam.QP()
        qp.add_cost(gtsam.HessianFactor(1, 2, np.array([[1.0]]), np.array([[-1.]]), np.array([2.]), np.array([[2.]]), np.array([6.]), 1000.0))
        qp.add_inequality(1, np.array([1.]), 2, np.array([1.]), 2.0)
        qp.add_inequality(1, np.array([-1.]), 2, np.array([2.]), 2.0)
        qp.add_inequality(1, np.array([2.]), 2, np.array([1.]), 3.0)
        qp.add_inequality(1, np.array([-1.]), 0.0)
        qp.add_inequality(2, np.array([-1.]), 0.0)

        solver = gtsam.QPSolver(qp)
        values, duals = solver.optimize()

        expectedValues = gtsam.VectorValues()
        expectedValues.insert(1, np.array([2.0/3.0]))
        expectedValues.insert(2, np.array([4.0/3.0]))

        self.assertTrue(values.equals(expectedValues, 1e-5))

if __name__ == "__main__":
    unittest.main()


