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
             gtsam.SamplingPlannerVectorVector, gu.PointMassDynamicsFactor, noise(4)),
            (gu.Unicycle(0.1), gtsam.Pose2(), gtsam.SamplingPlannerPose2Vector,
             gu.UnicycleDynamicsFactor, noise(3)),
        ]
        for model, start, planner_type, factor, model_noise in models:
            with self.subTest(model=type(model).__name__):
                planner = planner_type([X(0), X(1)], [U(0)], 0.1)
                point_mass = isinstance(model, gu.PointMass)
                if point_mass:
                    dynamics = gu.pointMassVectorField()
                    callback = model.dynamics
                else:
                    dynamics = gu.unicycleTransition(model)
                    callback = gtsam.discreteTransition(model.predict)
                graph = gtsam.NonlinearFactorGraph()
                graph.add(gtsam.PriorFactorVector(U(0), np.array([1., 0.]), unit))
                graph.add(factor(X(0), U(0), X(1), model, model_noise))
                controls = np.zeros((2, 1))
                seed = gtsam.Values()
                seed.insert(X(0), start)
                initial = planner.rollout(seed, controls, dynamics)
                np.testing.assert_array_equal(planner.nominalControls(initial), controls)
                proposal = gtsam.GaussianProposal(np.eye(2), 32, 17)
                nominal = planner.rollout(initial, np.zeros((2, 1)), dynamics)
                result = planner.optimize(graph, initial, dynamics, proposal, gtsam.MPPI(), 2)
                # The native model and the Python callback must produce the same
                # seeded update, including for a discrete step returning Pose2.
                replay = gtsam.GaussianProposal(np.eye(2), 32, 17)
                search = replay.initialize(controls)
                for _ in range(2):
                    planner.iterate(graph, initial, callback, replay, gtsam.MPPI(), search)
                np.testing.assert_allclose(search.mean, planner.nominalControls(result))
                self.assertLess(graph.error(result), graph.error(nominal))
                predicted = model.predict(start, result.atVector(U(0)))
                if point_mass:
                    np.testing.assert_allclose(result.atVector(X(1)), predicted)
                else:
                    self.assertTrue(result.atPose2(X(1)).equals(predicted, 1e-12))

    def test_point_mass_and_custom_cost(self):
        graph = gtsam.NonlinearFactorGraph()
        model = gu.PointMass(0.1)
        graph.add(gu.PointMassDynamicsFactor(
            X(0), U(0), X(1), model, gtsam.noiseModel.Constrained.All(4)))

        caller = threading.get_ident()

        def effort(factor, values, jacobians):
            self.assertEqual(threading.get_ident(), caller)
            return values.atVector(U(0))

        graph.add(gtsam.CustomFactor(gtsam.noiseModel.Unit.Create(2), [U(0)], effort))
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1)], [U(0)], 0.1, True)
        start = np.array([0., 0., 1., 0.])
        seed = gtsam.Values()
        seed.insert(X(0), start)
        dynamics = gu.pointMassVectorField()
        initial = planner.rollout(seed, np.zeros((2, 1)), dynamics)
        result = planner.optimize(graph, initial, dynamics,
                                  gtsam.GaussianProposal(np.eye(2), 16, 24), gtsam.MPPI(), 1)
        np.testing.assert_allclose(result.atVector(X(1)),
                                   model.predict(start, result.atVector(U(0))))
        self.assertTrue(np.isfinite(graph.error(result)))


if __name__ == "__main__":
    unittest.main()
