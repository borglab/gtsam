"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Shonan Rotation Averaging.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import math
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (BetweenFactorPose2, BetweenFactorPose3,
                   BinaryMeasurementRot3, LevenbergMarquardtParams, Pose2,
                   Pose3, Rot2, Rot3, ShonanAveraging2, ShonanAveraging3,
                   ShonanAveragingParameters2, ShonanAveragingParameters3)

DEFAULT_PARAMS = ShonanAveragingParameters3(
    gtsam.LevenbergMarquardtParams.CeresDefaults()
)


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
        """Check if ShonanAveraging2 constructor works when not initialized from g2o file.

        GT pose graph:

           | cam 1 = (0,4)
         --o
           | .
           .   .
           .     .
           |       |
           o-- ... o--
        cam 0       cam 2 = (4,0)
          (0,0)
        """
        num_images = 3

        wTi_list = [
            Pose2(Rot2.fromDegrees(0), np.array([0, 0])),
            Pose2(Rot2.fromDegrees(90), np.array([0, 4])),
            Pose2(Rot2.fromDegrees(0), np.array([4, 0])),
        ]

        edges = [(0, 1), (1, 2), (0, 2)]
        i2Ri1_dict = {
            (i1, i2): wTi_list[i2].inverse().compose(wTi_list[i1]).rotation()
            for (i1, i2) in edges
        }

        lm_params = LevenbergMarquardtParams.CeresDefaults()
        shonan_params = ShonanAveragingParameters2(lm_params)
        shonan_params.setUseHuber(False)
        shonan_params.setCertifyOptimality(True)

        noise_model = gtsam.noiseModel.Unit.Create(3)
        between_factors = []
        for (i1, i2), i2Ri1 in i2Ri1_dict.items():
            i2Ti1 = Pose2(i2Ri1, np.zeros(2))
            between_factors.append(
                BetweenFactorPose2(i2, i1, i2Ti1, noise_model)
            )

        obj = ShonanAveraging2(between_factors, shonan_params)
        initial = obj.initializeRandomly()
        result_values, _ = obj.run(initial, min_p=2, max_p=100)

        wRi_list = [result_values.atRot2(i) for i in range(num_images)]
        thetas_deg = np.array([wRi.degrees() for wRi in wRi_list])

        # map all angles to [0,360)
        thetas_deg = thetas_deg % 360
        thetas_deg -= thetas_deg[0]

        expected_thetas_deg = np.array([0.0, 90.0, 0.0])
        np.testing.assert_allclose(thetas_deg, expected_thetas_deg, atol=0.1)

    def test_measurements3(self):
        """Create from Measurements."""
        measurements = []
        unit3 = gtsam.noiseModel.Unit.Create(3)
        m01 = BinaryMeasurementRot3(0, 1, Rot3.Yaw(math.radians(90)), unit3)
        m12 = BinaryMeasurementRot3(1, 2, Rot3.Yaw(math.radians(90)), unit3)
        measurements.append(m01)
        measurements.append(m12)
        obj = ShonanAveraging3(measurements)
        self.assertIsInstance(obj, ShonanAveraging3)
        initial = obj.initializeRandomly()
        _, cost = obj.run(initial, min_p=3, max_p=5)
        self.assertAlmostEqual(cost, 0)

if __name__ == "__main__":
    unittest.main()
