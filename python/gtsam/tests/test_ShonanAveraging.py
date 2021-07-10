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
from gtsam import (
    BetweenFactorPose2,
    LevenbergMarquardtParams,
    Rot2,
    Pose2,
    ShonanAveraging2,
    ShonanAveragingParameters2,
    ShonanAveraging3,
    ShonanAveragingParameters3
)
from gtsam.utils.test_case import GtsamTestCase

DEFAULT_PARAMS = ShonanAveragingParameters3(
    gtsam.LevenbergMarquardtParams.CeresDefaults())


def fromExampleName(
    name: str, parameters: ShonanAveragingParameters3 = DEFAULT_PARAMS
) -> ShonanAveraging3:
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
    
    def test_constructorBetweenFactorPose2s(self) -> None:
        """Check if ShonanAveraging2 constructor works when not initialized from g2o file."""
        # map (i1,i2) pair to theta in degrees that parameterizes Rot2 object i2Ri1
        i2Ri1_dict = {
            (1, 2):  -43.86342,
            (1, 5):  -135.06351,
            (2, 4):  72.64527,
            (3, 5):  117.75967,
            (6, 7):  -31.73934,
            (7, 10): 177.45901,
            (6, 9):  -133.58713,
            (7, 8):  -90.58668,
            (0, 3):  127.02978,
            (8, 10): -92.16361,
            (4, 8):  51.63781,
            (4, 6):  173.96384,
            (4, 10): 139.59445,
            (2, 3):  151.04022,
            (3, 4):  -78.39495,
            (1, 4):  28.78185,
            (6, 8):  -122.32602,
            (0, 2):  -24.01045,
            (5, 7):  -53.93014,
            (4, 5):  -163.84535,
            (2, 5):  -91.20009,
            (1, 3):  107.17680,
            (7, 9):  -102.35615,
            (0, 1):  19.85297,
            (5, 8):  -144.51682,
            (5, 6):  -22.19079,
            (5, 10): -56.56016,
        }
        i2Ri1_dict = {(i1,i2): Rot2.fromDegrees(theta_deg) for (i1,i2), theta_deg in i2Ri1_dict.items()}
        lm_params = LevenbergMarquardtParams.CeresDefaults()
        shonan_params = ShonanAveragingParameters2(lm_params)
        shonan_params.setUseHuber(False)
        shonan_params.setCertifyOptimality(True)
        
        noise_model = gtsam.noiseModel.Unit.Create(3)
        between_factors = gtsam.BetweenFactorPose2s()
        for (i1, i2), i2Ri1 in i2Ri1_dict.items():
            i2Ti1 = Pose2(i2Ri1, np.zeros(2))
            between_factors.append(BetweenFactorPose2(i2, i1, i2Ti1, noise_model))
        
        obj = ShonanAveraging2(between_factors, shonan_params)
        initial = obj.initializeRandomly()
        result_values, _ = obj.run(initial, p_min, self._p_max)
        
        for i in range(11):
            wRi = result_values.atRot2(i)


if __name__ == '__main__':
    unittest.main()
