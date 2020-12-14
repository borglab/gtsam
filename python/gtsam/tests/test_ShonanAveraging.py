"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Shonan Rotation Averaging.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
from gtsam import ShonanAveraging3, ShonanAveragingParameters3
from gtsam.utils.test_case import GtsamTestCase

DEFAULT_PARAMS = ShonanAveragingParameters3(
    gtsam.LevenbergMarquardtParams.CeresDefaults())


def fromExampleName(name: str, parameters=DEFAULT_PARAMS):
    g2oFile = gtsam.findExampleDataFile(name)
    return ShonanAveraging3(g2oFile, parameters)


class TestShonanAveraging(GtsamTestCase):
    """Tests for Shonan Rotation Averaging."""

    def setUp(self):
        """Set up common variables."""
        self.shonan = fromExampleName("toyExample.g2o")

    def test_checkConstructor(self):
        self.assertEqual(5, self.shonan.nrUnknowns())

        D = self.shonan.denseD()
        self.assertEqual((15, 15), D.shape)

        Q = self.shonan.denseQ()
        self.assertEqual((15, 15), Q.shape)

        L = self.shonan.denseL()
        self.assertEqual((15, 15), L.shape)

    def test_buildGraphAt(self):
        graph = self.shonan.buildGraphAt(5)
        self.assertEqual(7, graph.size())

    def test_checkOptimality(self):
        random = self.shonan.initializeRandomlyAt(4)
        lambdaMin = self.shonan.computeMinEigenValue(random)
        self.assertAlmostEqual(-414.87376657555996,
                               lambdaMin, places=3)  # Regression test
        self.assertFalse(self.shonan.checkOptimality(random))

    def test_tryOptimizingAt3(self):
        initial = self.shonan.initializeRandomlyAt(3)
        self.assertFalse(self.shonan.checkOptimality(initial))
        result = self.shonan.tryOptimizingAt(3, initial)
        self.assertTrue(self.shonan.checkOptimality(result))
        lambdaMin = self.shonan.computeMinEigenValue(result)
        self.assertAlmostEqual(-5.427688831332745e-07,
                               lambdaMin, places=3)  # Regression test
        self.assertAlmostEqual(0, self.shonan.costAt(3, result), places=3)
        SO3Values = self.shonan.roundSolution(result)
        self.assertAlmostEqual(0, self.shonan.cost(SO3Values), places=3)

    def test_tryOptimizingAt4(self):
        random = self.shonan.initializeRandomlyAt(4)
        result = self.shonan.tryOptimizingAt(4, random)
        self.assertTrue(self.shonan.checkOptimality(result))
        self.assertAlmostEqual(0, self.shonan.costAt(4, result), places=2)
        lambdaMin = self.shonan.computeMinEigenValue(result)
        self.assertAlmostEqual(-5.427688831332745e-07,
                               lambdaMin, places=3)  # Regression test
        SO3Values = self.shonan.roundSolution(result)
        self.assertAlmostEqual(0, self.shonan.cost(SO3Values), places=3)

    def test_initializeWithDescent(self):
        random = self.shonan.initializeRandomlyAt(3)
        Qstar3 = self.shonan.tryOptimizingAt(3, random)
        lambdaMin, minEigenVector = self.shonan.computeMinEigenVector(Qstar3)
        initialQ4 = self.shonan.initializeWithDescent(
            4, Qstar3, minEigenVector, lambdaMin)
        self.assertAlmostEqual(5, initialQ4.size())

    def test_run(self):
        initial = self.shonan.initializeRandomly()
        result, lambdaMin = self.shonan.run(initial, 5, 10)
        self.assertAlmostEqual(0, self.shonan.cost(result), places=2)
        self.assertAlmostEqual(-5.427688831332745e-07,
                               lambdaMin, places=3)  # Regression test

    def test_runKlausKarcher(self):
        # Load 2D toy example
        lmParams = gtsam.LevenbergMarquardtParams.CeresDefaults()
        # lmParams.setVerbosityLM("SUMMARY")
        g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt")
        parameters = gtsam.ShonanAveragingParameters2(lmParams)
        shonan = gtsam.ShonanAveraging2(g2oFile, parameters)
        self.assertAlmostEqual(4, shonan.nrUnknowns())

        # Check graph building
        graph = shonan.buildGraphAt(2)
        self.assertAlmostEqual(6, graph.size())
        initial = shonan.initializeRandomly()
        result, lambdaMin = shonan.run(initial, 2, 10)
        self.assertAlmostEqual(0.0008211, shonan.cost(result), places=5)
        self.assertAlmostEqual(0, lambdaMin, places=9)  # certificate!

    # Test alpha/beta/gamma prior weighting.
    def test_PriorWeights(self):
        lmParams = gtsam.LevenbergMarquardtParams.CeresDefaults()
        params = ShonanAveragingParameters3(lmParams)
        self.assertAlmostEqual(0, params.getAnchorWeight(), 1e-9)
        self.assertAlmostEqual(1, params.getKarcherWeight(), 1e-9)
        self.assertAlmostEqual(0, params.getGaugesWeight(), 1e-9)
        alpha, beta, gamma = 100.0, 200.0, 300.0
        params.setAnchorWeight(alpha)
        params.setKarcherWeight(beta)
        params.setGaugesWeight(gamma)
        self.assertAlmostEqual(alpha, params.getAnchorWeight(), 1e-9)
        self.assertAlmostEqual(beta, params.getKarcherWeight(), 1e-9)
        self.assertAlmostEqual(gamma, params.getGaugesWeight(), 1e-9)
        params.setKarcherWeight(0)
        shonan = fromExampleName("Klaus3.g2o", params)

        initial = gtsam.Values()
        for i in range(3):
            initial.insert(i, gtsam.Rot3())
        self.assertAlmostEqual(3.0756, shonan.cost(initial), places=3)
        result, _lambdaMin = shonan.run(initial, 3, 3)
        self.assertAlmostEqual(0.0015, shonan.cost(result), places=3)


if __name__ == '__main__':
    unittest.main()
