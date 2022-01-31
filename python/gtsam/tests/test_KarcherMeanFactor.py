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
from gtsam import Rot3
from gtsam.utils.test_case import GtsamTestCase

KEY = 0
MODEL = gtsam.noiseModel.Unit.Create(3)


# Rot3 version
R = Rot3.Expmap(np.array([0.1, 0, 0]))


class TestKarcherMean(GtsamTestCase):

    def test_find(self):
        # Check that optimizing for Karcher mean (which minimizes Between distance)
        # gets correct result.
        rotations = [R, R.inverse()]
        expected = Rot3()
        actual = gtsam.FindKarcherMean(rotations)
        self.gtsamAssertEquals(expected, actual)

    def test_find_karcher_mean_identity(self):
        """Averaging 3 identity rotations should yield the identity."""
        a1Rb1 = Rot3()
        a2Rb2 = Rot3()
        a3Rb3 = Rot3()

        aRb_list = [a1Rb1, a2Rb2, a3Rb3]
        aRb_expected = Rot3()

        aRb = gtsam.FindKarcherMean(aRb_list)
        self.gtsamAssertEquals(aRb, aRb_expected)

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
        keys = [1, 2]
        graph.add(gtsam.KarcherMeanFactorRot3(keys))

        initial = gtsam.Values()
        initial.insert(1, R.inverse())
        initial.insert(2, R)
        expected = Rot3()

        result = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
        actual = gtsam.FindKarcherMean([result.atRot3(1), result.atRot3(2)])
        self.gtsamAssertEquals(expected, actual)
        self.gtsamAssertEquals(
            R12, result.atRot3(1).between(result.atRot3(2)))


if __name__ == "__main__":
    unittest.main()
