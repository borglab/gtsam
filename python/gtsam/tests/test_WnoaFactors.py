"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for White-Noise-on-Acceleration, Continuous-time, Gaussian-process factors.

Author: Connor Holmes
"""

import unittest
from dataclasses import dataclass

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase

from gtsam import WnoaInterpFactorPose3, WnoaMotionFactorPose3
from gtsam.symbol_shorthand import X, V


@dataclass
class Se3FixtureData:
    """Data container for SE3 fixture."""

    timeStep: float
    qPsdDiag: np.ndarray
    values: gtsam.Values


@dataclass
class Se3InterpGraphData:
    """Data container for SE3 interpolation graph."""

    timeStep: float
    qPsdDiag: np.ndarray
    values: gtsam.Values
    newGraph: gtsam.NonlinearFactorGraph
    estimatedStates: set
    interpolatedStates: set


class TestStateData(GtsamTestCase):
    """Test StateData class."""

    def testConstruction(self):
        """Test construction of StateData."""
        stateData = gtsam.StateData()
        self.assertIsInstance(stateData, gtsam.StateData)

        stateData = gtsam.StateData(X(0), V(0), 0.0)
        self.assertEqual(stateData.pose, X(0))
        self.assertEqual(stateData.velocity, V(0))
        self.assertEqual(stateData.time, 0.0)


class TestWnoaMotionFactor(GtsamTestCase):
    """Test WnoaMotionFactor class.
    Tests are based on the more extensive C++ tests in gtsam/nonlinear/testWnoaFactor.cpp.
    """

    def testConstructionAndEval(self):
        """Test construction of WnoaMotionFactor."""
        stateData0 = gtsam.StateData(X(0), V(0), 0.0)
        stateData1 = gtsam.StateData(X(1), V(1), 1.0)
        qPsdDiag = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        factor = WnoaMotionFactorPose3(stateData0, stateData1, qPsdDiag)
        self.assertIsInstance(factor, WnoaMotionFactorPose3)

    def testEvaluateError(self):
        """Test evaluateError without Jacobians."""
        stateData0 = gtsam.StateData(X(0), V(0), 0.0)
        stateData1 = gtsam.StateData(X(1), V(1), 0.1)

        qPsdDiag = np.ones(6)
        factor = WnoaMotionFactorPose3(stateData0, stateData1, qPsdDiag)

        p0 = gtsam.Pose3.Expmap(np.array([0.5, 0.0, 0.2, 1.0, 0.0, 0.0]))
        v0 = np.array([0.1, 0.0, 0.0, 1.0, 0.0, 2.0])
        p1 = p0.retract(0.1 * v0)
        v1 = 2.0 * v0

        error = factor.evaluateError(p0, v0, p1, v1)
        expected = np.hstack([np.zeros(6), v0])

        np.testing.assert_allclose(error, expected, rtol=1e-4, atol=1e-6)


class TestWnoaInterpFactorPose3(GtsamTestCase):
    """Test WnoaInterpFactorPose3 class."""

    def _makeFactorAndValues(self):
        timeStep = 0.1
        qPsdDiag = np.ones(6)
        p0 = gtsam.Pose3.Expmap(np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]))
        v0 = np.array([1.0, 0.0, 0.5, 0.1, 0.0, 0.0])

        estimatedStates = {
            gtsam.StateData(X(2), V(2), 2.0 * timeStep),
            gtsam.StateData(X(0), V(0), 0.0),
        }
        interpolatedStates = {gtsam.StateData(X(1), V(1), timeStep)}

        p1 = p0.retract(timeStep * v0)
        p2 = p0.retract(2.0 * timeStep * v0)

        model = gtsam.noiseModel.Isotropic.Sigma(6, 1.0)
        prior = gtsam.PriorFactorPose3(X(1), p1, model)
        factor = WnoaInterpFactorPose3(
            prior, estimatedStates, interpolatedStates, qPsdDiag
        )

        values = gtsam.Values()
        values.insert(X(0), p0)
        values.insert(X(2), p2)
        values.insert(V(0), v0)
        values.insert(V(2), v0)

        return factor, values

    def testConstruction(self):
        """Test WnoaInterpFactorPose3 construction."""
        factor, _ = self._makeFactorAndValues()
        self.assertIsInstance(factor, WnoaInterpFactorPose3)

    def testPrint(self):
        """Test WnoaInterpFactorPose3 print."""
        factor, _ = self._makeFactorAndValues()
        factor.print()

    def testEquals(self):
        """Test WnoaInterpFactorPose3 equals."""
        factor1, _ = self._makeFactorAndValues()
        factor2, _ = self._makeFactorAndValues()
        self.assertTrue(factor1.equals(factor2, 1e-9))

    def testError(self):
        """Test WnoaInterpFactorPose3 error."""
        factor, values = self._makeFactorAndValues()
        error = factor.error(values)
        self.assertAlmostEqual(error, 0.0, places=9)


class TestWnoaFactorGraphPose3(GtsamTestCase):
    """Test interpolation factor graphs for Pose3."""

    def _makeSe3Fixture(self):
        timeStep = 0.1
        qPsdDiag = np.ones(6)

        p0 = gtsam.Pose3.Expmap(np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0]))
        v0 = np.array([1.0, 0.0, 0.5, 0.1, 0.0, 0.0])

        values = gtsam.Values()
        values.insert(X(0), p0)
        values.insert(X(1), p0.retract(timeStep * v0))
        values.insert(X(2), p0.retract(2.0 * timeStep * v0))
        values.insert(X(3), p0.retract(3.0 * timeStep * v0))
        values.insert(X(4), p0.retract(4.0 * timeStep * v0))
        values.insert(V(0), v0)
        values.insert(V(1), v0)
        values.insert(V(2), v0)
        values.insert(V(3), v0)
        values.insert(V(4), v0)

        return Se3FixtureData(
            timeStep=timeStep,
            qPsdDiag=qPsdDiag,
            values=values,
        )

    def _makeSe3InterpGraph(self, useWnoaGraph=False):
        fixture = self._makeSe3Fixture()
        v0 = fixture.values.atVector6(V(0))

        model = gtsam.noiseModel.Isotropic.Sigma(6, 1.0)
        p1 = fixture.values.atPose3(X(1))
        p3 = fixture.values.atPose3(X(3))
        betweenFactor = gtsam.BetweenFactorPose3(
            X(1),
            X(3),
            p1.between(p3),
            model,
        )
        priorPoseFactor = gtsam.PriorFactorPose3(X(1), p1, model)
        priorVelFactor = gtsam.PriorFactorVector6(V(1), v0, model)

        graph = gtsam.NonlinearFactorGraph()
        graph.add(betweenFactor)
        graph.add(priorPoseFactor)
        graph.add(priorVelFactor)

        estimatedStates = {
            gtsam.StateData(X(0), V(0), 0.0),
            gtsam.StateData(X(4), V(4), 4.0 * fixture.timeStep),
            gtsam.StateData(X(2), V(2), 2.0 * fixture.timeStep),
        }
        interpolatedStates = {
            gtsam.StateData(X(3), V(3), 3.0 * fixture.timeStep),
            gtsam.StateData(X(1), V(1), fixture.timeStep),
        }

        if useWnoaGraph:
            newGraph = gtsam.interpolateWnoaFactorGraphPose3(
                graph, estimatedStates, interpolatedStates, fixture.qPsdDiag
            )
        else:
            newGraph = gtsam.interpolateFactorGraphPose3(
                graph, estimatedStates, interpolatedStates, fixture.qPsdDiag
            )

        # Create values with only positions 0, 2, 4 and velocities
        values = gtsam.Values()
        values.insert(X(0), fixture.values.atPose3(X(0)))
        values.insert(X(2), fixture.values.atPose3(X(2)))
        values.insert(X(4), fixture.values.atPose3(X(4)))
        values.insert(V(0), v0)
        values.insert(V(2), v0)
        values.insert(V(4), v0)

        return Se3InterpGraphData(
            timeStep=fixture.timeStep,
            qPsdDiag=fixture.qPsdDiag,
            values=values,
            newGraph=newGraph,
            estimatedStates=estimatedStates,
            interpolatedStates=interpolatedStates,
        )

    def _assertCovarianceMap(self, covarianceMap):
        def isPositiveSemidefinite(covariance):
            symmetric = 0.5 * (covariance + covariance.T)
            minEigenvalue = np.min(np.linalg.eigvalsh(symmetric))
            return minEigenvalue >= -1e-9

        self.assertIn(X(1), covarianceMap)
        self.assertIn(V(1), covarianceMap)
        self.assertIn(X(3), covarianceMap)
        self.assertIn(V(3), covarianceMap)

        for key in (X(1), V(1), X(3), V(3)):
            covariance = covarianceMap[key]
            self.assertEqual(covariance.shape, (6, 6))
            self.assertTrue(isPositiveSemidefinite(covariance))

    def testSe3InterpGraph(self):
        """Test interpolateFactorGraphPose3 for SE3 graphs."""
        data = self._makeSe3InterpGraph()

        optimizer = gtsam.GaussNewtonOptimizer(data.newGraph, data.values)
        self.assertAlmostEqual(optimizer.error(), 0.0, places=9)
        optimizer.iterate()
        self.assertAlmostEqual(optimizer.error(), 0.0, places=9)
        optimizer.optimize()
        self.assertAlmostEqual(optimizer.error(), 0.0, places=6)

        perturb = np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1])
        valuesPert = gtsam.Values()
        valuesPert.insert(X(0), data.values.atPose3(X(0)).retract(perturb))
        valuesPert.insert(X(2), data.values.atPose3(X(2)).retract(perturb))
        valuesPert.insert(X(4), data.values.atPose3(X(4)).retract(perturb))
        v0 = data.values.atVector6(V(0))
        vPert = v0 + 0.1 * np.ones(6)
        valuesPert.insert(V(0), vPert)
        valuesPert.insert(V(2), vPert)
        valuesPert.insert(V(4), vPert)

        optimizer2 = gtsam.GaussNewtonOptimizer(data.newGraph, valuesPert)
        optimizer2.optimize()
        self.assertAlmostEqual(optimizer2.error(), 0.0, places=4)

    def testSe3InterpWnoaGraph(self):
        """Test interpolateWnoaFactorGraphPose3 for SE3 graphs."""
        data = self._makeSe3InterpGraph(useWnoaGraph=True)

        optimizer = gtsam.LevenbergMarquardtOptimizer(data.newGraph, data.values)
        self.assertAlmostEqual(optimizer.error(), 0.0, places=9)
        optimizer.iterate()
        self.assertAlmostEqual(optimizer.error(), 0.0, places=9)
        optimizer.optimize()
        self.assertAlmostEqual(optimizer.error(), 0.0, places=6)

        perturb = np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1])
        valuesPert = gtsam.Values()
        valuesPert.insert(X(0), data.values.atPose3(X(0)).retract(perturb))
        valuesPert.insert(X(2), data.values.atPose3(X(2)).retract(perturb))
        valuesPert.insert(X(4), data.values.atPose3(X(4)).retract(perturb))
        v0 = data.values.atVector6(V(0))
        vPert = v0 + 0.1 * np.ones(6)
        valuesPert.insert(V(0), vPert)
        valuesPert.insert(V(2), vPert)
        valuesPert.insert(V(4), vPert)

        optimizer2 = gtsam.GaussNewtonOptimizer(data.newGraph, valuesPert)
        optimizer2.optimize()
        self.assertAlmostEqual(optimizer2.error(), 0.0, places=4)

    def testSe3UpdateInterpValues(self):
        """Test updateInterpValuesPose3 for SE3 graphs."""
        data = self._makeSe3InterpGraph()

        resultInterp = gtsam.updateInterpValuesPose3(
            data.newGraph,
            data.values,
            data.estimatedStates,
            data.interpolatedStates,
            data.qPsdDiag,
        )

        p3Est = resultInterp.atPose3(X(3))
        p1Est = resultInterp.atPose3(X(1))
        v3Est = resultInterp.atVector6(V(3))
        v1Est = resultInterp.atVector6(V(1))

        # Ground truth fixture
        data_gt = self._makeSe3Fixture()
        v0 = data_gt.values.atVector6(V(0))
        p3 = data_gt.values.atPose3(X(3))
        p1 = data_gt.values.atPose3(X(1))

        self.gtsamAssertEquals(p3, p3Est, 1e-6)
        self.gtsamAssertEquals(p1, p1Est, 1e-6)
        np.testing.assert_allclose(v0, v3Est, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(v0, v1Est, rtol=1e-12, atol=1e-12)

    def testSe3UpdateInterpValuesWithCovariance(self):
        """Test updateInterpValuesWithCovariancePose3 for SE3 graphs."""
        data = self._makeSe3InterpGraph()

        resultInterp, covarianceMap = gtsam.updateInterpValuesWithCovariancePose3(
            data.newGraph,
            data.values,
            data.estimatedStates,
            data.interpolatedStates,
            data.qPsdDiag,
        )

        p3Est = resultInterp.atPose3(X(3))
        p1Est = resultInterp.atPose3(X(1))
        v3Est = resultInterp.atVector6(V(3))
        v1Est = resultInterp.atVector6(V(1))

        # Ground truth fixture
        data_gt = self._makeSe3Fixture()
        v0 = data_gt.values.atVector6(V(0))
        p3 = data_gt.values.atPose3(X(3))
        p1 = data_gt.values.atPose3(X(1))

        self.gtsamAssertEquals(p3, p3Est, 1e-6)
        self.gtsamAssertEquals(p1, p1Est, 1e-6)
        np.testing.assert_allclose(v0, v3Est, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(v0, v1Est, rtol=1e-12, atol=1e-12)
        self._assertCovarianceMap(covarianceMap)


if __name__ == "__main__":
    unittest.main()
