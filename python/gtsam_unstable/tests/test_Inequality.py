"""Tests for the legacy linear inequality wrappers."""

import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase


class TestLinearInequality(GtsamTestCase):
    def test_factor_and_graph(self):
        factor = gtsam_unstable.LinearInequality(
            0, np.array([[1.0]]), 1.0, 100
        )
        values = gtsam.VectorValues()
        values.insert(0, np.array([0.5]))

        self.assertEqual(factor.dualKey(), 100)
        self.assertTrue(factor.active())
        self.assertAlmostEqual(factor.error(values), -0.5)

        factor.inactivate()
        self.assertFalse(factor.active())
        factor.activate()

        graph = gtsam_unstable.InequalityFactorGraph()
        graph.add(0, np.array([[1.0]]), 1.0, 100)
        self.assertEqual(graph.size(), 1)
        self.assertEqual(graph.nrFactors(), 1)
        self.assertTrue(graph.exists(0))
        self.assertAlmostEqual(graph.error(values), 0.0)

        violating_values = gtsam.VectorValues()
        violating_values.insert(0, np.array([1.5]))
        self.assertTrue(np.isinf(graph.error(violating_values)))


if __name__ == "__main__":
    unittest.main()
