"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Values.
Author: Shangjie Xue, Varun Agrawal, Frank Dellaert
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.symbol_shorthand import D, V, M
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestHybridValues(GtsamTestCase):
    """Unit tests for HybridValues."""

    def setUp(self):
        """Set up common objects for tests."""
        self.vector_values = gtsam.VectorValues()
        self.vector_values.insert(V(0), np.array([1.0, 2.0]))
        self.vector_values.insert(V(1), np.array([3.0]))

        self.discrete_values = gtsam.DiscreteValues()
        self.discrete_values[D(0)] = 1
        self.discrete_values[D(1)] = 0

        self.nonlinear_values = gtsam.Values()
        self.nonlinear_values.insert(M(5), gtsam.Pose2(1, 2, 0.3))

    def test_constructors(self):
        """Test various constructors."""
        hv_empty = gtsam.HybridValues()
        self.assertEqual(hv_empty.continuous().size(), 0)
        self.assertEqual(len(hv_empty.discrete()), 0)
        self.assertEqual(hv_empty.nonlinear().size(), 0)

        hv_vd = gtsam.HybridValues(self.vector_values, self.discrete_values)
        self.assertEqual(hv_vd.continuous().size(), 2)
        self.assertEqual(len(hv_vd.discrete()), 2)
        self.assertEqual(hv_vd.nonlinear().size(), 0)
        self.assertTrue(hv_vd.continuous().equals(self.vector_values, 1e-9))
        # DiscreteValues comparison needs to be element-wise or via string
        self.assertEqual(hv_vd.discrete()[D(0)], self.discrete_values[D(0)])

        hv_all = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )
        self.assertEqual(hv_all.continuous().size(), 2)
        self.assertEqual(len(hv_all.discrete()), 2)
        self.assertEqual(hv_all.nonlinear().size(), 1)
        self.assertTrue(hv_all.nonlinear().equals(self.nonlinear_values, 1e-9))

    def test_accessors(self):
        """Test accessing underlying containers."""
        hv_all = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )

        self.assertTrue(hv_all.continuous().equals(self.vector_values, 1e-9))
        # Compare DiscreteValues content
        retrieved_dv = hv_all.discrete()
        self.assertEqual(len(retrieved_dv), len(self.discrete_values))
        for k, v in self.discrete_values.items():
            self.assertEqual(retrieved_dv[k], v)
        self.assertTrue(hv_all.nonlinear().equals(self.nonlinear_values, 1e-9))

        # Test at methods
        self.gtsamAssertEquals(hv_all.at(V(0)), self.vector_values.at(V(0)), 1e-9)
        self.assertEqual(hv_all.atDiscrete(D(0)), self.discrete_values[D(0)])
        # For nonlinear, access via nonlinear().atTYPE()
        self.assertTrue(
            hv_all.nonlinear()
            .atPose2(M(5))
            .equals(self.nonlinear_values.atPose2(M(5)), 1e-9)
        )

    def test_exists(self):
        """Test existence checks."""
        hv_all = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )

        self.assertTrue(hv_all.existsVector(V(0)))
        self.assertFalse(hv_all.existsVector(D(0)))  # Key for discrete
        self.assertFalse(hv_all.existsVector(M(5)))  # Key for nonlinear

        self.assertTrue(hv_all.existsDiscrete(D(1)))
        self.assertFalse(hv_all.existsDiscrete(V(0)))  # Key for vector

        self.assertTrue(hv_all.existsNonlinear(M(5)))
        self.assertFalse(hv_all.existsNonlinear(V(0)))  # Key for vector

        # General exists (checks nonlinear, then vector, then discrete)
        self.assertTrue(hv_all.exists(V(0)))  # Vector
        self.assertTrue(hv_all.exists(D(0)))  # Discrete
        self.assertTrue(hv_all.exists(M(5)))  # Nonlinear
        self.assertFalse(hv_all.exists(D(7)))  # Non-existent key

    def test_equals(self):
        """Test equals method."""
        hv1 = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )
        hv2 = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )

        self.assertTrue(hv1.equals(hv2, 1e-9))

    def test_insert_individual(self):
        """Test inserting individual values."""
        hv = gtsam.HybridValues()
        hv.insert(V(10), np.array([1.0]))
        hv.insert(D(10), 1)
        hv.insertNonlinear(M(11), gtsam.Pose2())

        self.assertTrue(hv.existsVector(V(10)))
        self.gtsamAssertEquals(hv.at(V(10)), np.array([1.0]))
        self.assertTrue(hv.existsDiscrete(D(10)))
        self.assertEqual(hv.atDiscrete(D(10)), 1)

    def test_insert_containers(self):
        """Test inserting from other Values containers."""
        hv = gtsam.HybridValues()

        hv.insert(self.vector_values)
        self.assertEqual(hv.continuous().size(), 2)
        self.assertTrue(hv.continuous().equals(self.vector_values, 1e-9))

        hv.insert(self.discrete_values)
        self.assertEqual(len(hv.discrete()), 2)
        # Check discrete values equality
        retrieved_dv = hv.discrete()
        for k, v in self.discrete_values.items():
            self.assertEqual(retrieved_dv[k], v)

        hv.insert(self.nonlinear_values)
        self.assertEqual(hv.nonlinear().size(), 1)
        self.assertTrue(hv.nonlinear().equals(self.nonlinear_values, 1e-9))

        hv_copy = gtsam.HybridValues()
        hv_copy.insert(hv)  # Test insert(HybridValues)
        self.assertTrue(hv_copy.equals(hv, 1e-9))

    def test_insert_or_assign(self):
        """Test insert_or_assign method."""
        hv = gtsam.HybridValues()
        hv.insert(V(0), np.array([1.0]))
        hv.insert(D(0), 0)

        # Test insert_or_assign for vector
        hv.insert_or_assign(V(0), np.array([2.0]))  # Update existing
        self.gtsamAssertEquals(hv.at(V(0)), np.array([2.0]))
        hv.insert_or_assign(V(1), np.array([3.0]))  # Insert new
        self.gtsamAssertEquals(hv.at(V(1)), np.array([3.0]))

        # Test insert_or_assign for discrete
        hv.insert_or_assign(D(0), 1)  # Update existing
        self.assertEqual(hv.atDiscrete(D(0)), 1)
        hv.insert_or_assign(D(1), 2)  # Insert new
        self.assertEqual(hv.atDiscrete(D(1)), 2)

    def test_update(self):
        """Test update methods."""
        hv = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )

        # Update VectorValues
        vv_update = gtsam.VectorValues()
        vv_update.insert(V(0), np.array([10.0, 20.0]))  # Existing key
        hv.update(vv_update)
        self.gtsamAssertEquals(hv.at(V(0)), np.array([10.0, 20.0]))
        self.assertEqual(hv.continuous().size(), 2)  # X0, X1

        # Update DiscreteValues
        dv_update = gtsam.DiscreteValues()
        dv_update[D(0)] = 5  # Existing key
        hv.update(dv_update)
        self.assertEqual(hv.atDiscrete(D(0)), 5)
        self.assertEqual(len(hv.discrete()), 2)  # C0, C1

        # Update NonlinearValues
        nv_update = gtsam.Values()
        nv_update.insert(M(5), gtsam.Pose2(3, 4, 0.5))
        hv.update(nv_update)
        self.assertTrue(
            hv.nonlinear().atPose2(M(5)).equals(gtsam.Pose2(3, 4, 0.5), 1e-9)
        )
        self.assertEqual(hv.nonlinear().size(), 1)

        # Update HybridValues (only continuous and discrete parts for now)
        hv_update_source = gtsam.HybridValues()
        vv_source = gtsam.VectorValues()
        vv_source.insert(V(0), np.array([-1.0, -2.0]))
        dv_source = gtsam.DiscreteValues()
        dv_source[D(0)] = 100
        hv_update_source.insert(vv_source)
        hv_update_source.insert(dv_source)

        hv.update(hv_update_source)
        self.gtsamAssertEquals(hv.at(V(0)), np.array([-1.0, -2.0]))
        self.assertEqual(hv.atDiscrete(D(0)), 100)

    def test_retract(self):
        """Test retract method."""
        hv = gtsam.HybridValues(
            self.vector_values, self.discrete_values, self.nonlinear_values
        )

        delta = gtsam.VectorValues()
        deltaM5 = np.array([0.5, 0.5, -1.0])
        delta.insert(M(5), deltaM5)

        hv_retracted = hv.retract(delta)

        # Original should be unchanged
        self.assertTrue(hv.continuous().equals(self.vector_values, 1e-9))

        # Check retracted values
        expected_M5 = hv.nonlinear().atPose2(M(5)).retract(deltaM5)
        self.gtsamAssertEquals(hv_retracted.nonlinear().atPose2(M(5)), expected_M5)

        # Check that discrete and continuous parts are copied
        self.assertEqual(len(hv_retracted.discrete()), len(self.discrete_values))
        self.assertEqual(hv_retracted.continuous().size(), self.vector_values.size())


if __name__ == "__main__":
    unittest.main()
