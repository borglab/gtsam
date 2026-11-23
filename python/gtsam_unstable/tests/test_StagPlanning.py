"""STAG rollout types and Python graph callbacks.

Authors: Frank Dellaert and Akash Ratheesh. See LICENSE for details.
"""

import threading
import unittest

import numpy as np

import gtsam
import gtsam_unstable as gu
from gtsam.symbol_shorthand import U, X


class TestStagPlanning(unittest.TestCase):
    def test_native_models_share_the_planner(self):
        unit = gtsam.noiseModel.Unit.Create(2)
        noise = gtsam.noiseModel.Constrained.All
        models = [
            (gu.PointMass(0.1), np.array([0., 0., 1., 0.]),
             gtsam.SamplingPlannerVector4Point2, gu.PointMassDynamicsFactor, noise(4)),
            (gu.Unicycle(0.1), gtsam.Pose2(), gtsam.SamplingPlannerPose2Point2,
             gu.UnicycleDynamicsFactor, noise(3)),
        ]
        for model, start, planner_type, factor, model_noise in models:
            with self.subTest(model=type(model).__name__):
                planner = planner_type([X(0), X(1)], [U(0)], 0.1)
                initial = gtsam.Values()
                initial.insertPoint2(U(0), np.zeros(2))
                point_mass = isinstance(model, gu.PointMass)
                if point_mass:
                    initial.insertVector4(X(0), start)
                    dynamics = gu.pointMassVectorField()
                else:
                    initial.insert(X(0), start)
                    dynamics = gu.unicycleTransition(model)
                graph = gtsam.NonlinearFactorGraph()
                graph.add(gtsam.PriorFactorPoint2(U(0), np.array([1., 0.]), unit))
                graph.add(factor(X(0), U(0), X(1), model, model_noise))
                proposal = gtsam.GaussianProposal(np.eye(2), 32, 17)
                nominal = planner.rollout(initial, np.zeros((2, 1)), dynamics)
                result = planner.optimize(graph, initial, dynamics, proposal, gtsam.MPPI(), 2)
                self.assertLess(graph.error(result), graph.error(nominal))
                predicted = model.predict(start, result.atPoint2(U(0)))
                if point_mass:
                    np.testing.assert_allclose(result.atVector4(X(1)), predicted)
                else:
                    self.assertTrue(result.atPose2(X(1)).equals(predicted, 1e-12))

    def test_fixed_size_state_and_custom_cost(self):
        graph = gtsam.NonlinearFactorGraph()
        model = gu.PointMass(0.1)
        graph.add(gu.PointMassDynamicsFactor(
            X(0), U(0), X(1), model, gtsam.noiseModel.Constrained.All(4)))

        caller = threading.get_ident()

        def effort(factor, values, jacobians):
            self.assertEqual(threading.get_ident(), caller)
            return values.atPoint2(U(0))

        graph.add(gtsam.CustomFactor(gtsam.noiseModel.Unit.Create(2), [U(0)], effort))
        params = gtsam.VanillaMPPIParams()
        params.controlCovariance = np.eye(2)
        params.numSamples = 16
        self.assertFalse(params.parallelCandidateScoring)
        params.parallelCandidateScoring = True
        controller = gu.VanillaMPPIControllerPointMassDynamicsFactor(params)
        start = np.array([0.0, 0.0, 1.0, 0.0])
        initial = controller.initialValues(graph, [U(0)], start, np.zeros((2, 1)))
        result = controller.optimize(graph, initial, [U(0)])
        np.testing.assert_allclose(result.atVector4(X(1)),
                                   model.predict(start, result.atPoint2(U(0))))
        self.assertTrue(np.isfinite(graph.error(result)))


if __name__ == "__main__":
    unittest.main()
