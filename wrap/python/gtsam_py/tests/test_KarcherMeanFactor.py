"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

KarcherMeanFactor unit tests.
Author: Frank Dellaert
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase

KEY = 0
MODEL = gtsam.noiseModel.Unit.Create(3)


def find_Karcher_mean_Rot3(rotations):
    """Find the Karcher mean of given values."""
    # Cost function C(R) = \sum PriorFactor(R_i)::error(R)
    # No closed form solution.
    graph = gtsam.NonlinearFactorGraph()
    for R in rotations:
        graph.add(gtsam.PriorFactorRot3(KEY, R, MODEL))
    initial = gtsam.Values()
    initial.insert(KEY, gtsam.Rot3())
    result = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
    return result.atRot3(KEY)


# Rot3 version
R = gtsam.Rot3.Expmap(np.array([0.1, 0, 0]))


class TestKarcherMean(GtsamTestCase):

    def test_find(self):
        # Check that optimizing for Karcher mean (which minimizes Between distance)
        # gets correct result.
        rotations = {R, R.inverse()}
        expected = gtsam.Rot3()
        actual = find_Karcher_mean_Rot3(rotations)
        self.gtsamAssertEquals(expected, actual)

    def test_factor(self):
        """Check that the InnerConstraint factor leaves the mean unchanged."""
        # Make a graph with two variables, one between, and one InnerConstraint
        # The optimal result should satisfy the between, while moving the other
        # variable to make the mean the same as before.
        # Mean of R and R' is identity. Let's make a BetweenFactor making R21 =
        # R*R*R, i.e. geodesic length is 3 rather than 2.
        graph = gtsam.NonlinearFactorGraph()
        R12 = R.compose(R.compose(R))
        graph.add(gtsam.BetweenFactorRot3(1, 2, R12, MODEL))
        keys = gtsam.KeyVector()
        keys.append(1)
        keys.append(2)
        graph.add(gtsam.KarcherMeanFactorRot3(keys))

        initial = gtsam.Values()
        initial.insert(1, R.inverse())
        initial.insert(2, R)
        expected = find_Karcher_mean_Rot3([R, R.inverse()])

        result = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
        actual = find_Karcher_mean_Rot3(
            [result.atRot3(1), result.atRot3(2)])
        self.gtsamAssertEquals(expected, actual)
        self.gtsamAssertEquals(
            R12, result.atRot3(1).between(result.atRot3(2)))


if __name__ == "__main__":
    unittest.main()
