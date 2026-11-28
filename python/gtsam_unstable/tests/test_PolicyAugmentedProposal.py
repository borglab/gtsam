"""Python dynamics, policy proposals and factor callbacks in a native sampling batch.

Authors: Frank Dellaert and Akash Ratheesh. See LICENSE for details.
"""

import gtsam_unstable as unstable
import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import U, X


class TestSamplingPlanner(unittest.TestCase):
    def test_policy_candidate_with_python_cost(self):
        """Predictive sampling should select the policy candidate with zero error."""
        initial = gtsam.Values()
        initial.insert(X(0), np.zeros(1))
        initial.insert(U(0), np.zeros(1))
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1)], [U(0)], 1.0)

        def terminal_error(factor, values, jacobians):
            return values.atVector(X(1)) - 2.0

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.CustomFactor(
            gtsam.noiseModel.Unit.Create(1), [X(1)], terminal_error))
        dynamics = lambda x, u: u
        proposal = unstable.PolicyAugmentedProposal(
            np.eye(1), 8, 1, lambda context, nominal: np.full_like(nominal, 2.0), 42)
        result = planner.optimize(graph, initial, dynamics, proposal,
                                  gtsam.PredictiveSampling(), 1)
        np.testing.assert_allclose(result.atVector(U(0)), [2.0])
        np.testing.assert_allclose(result.atVector(X(1)), [2.0])
        self.assertEqual(graph.error(result), 0.0)


    def test_seeded_tracking_proposal(self):
        """Tube and robust search replay the policy innovations on paired trajectories."""
        states, controls = [X(0), X(1)], [U(0)]
        planner = gtsam.SamplingPlannerVectorVector(states, controls, 1.0)
        tracking = gtsam.MPPITracking(np.eye(1), 8, 1.0)
        nominal = gtsam.Values()
        nominal.insert(X(0), np.zeros(1))
        nominal.insert(U(0), np.zeros(1))
        dynamics = lambda x, u: u
        nominal = planner.rollout(nominal, np.zeros((1, 1)), dynamics)
        actual = gtsam.Values(nominal)
        actual.update(X(0), np.ones(1))
        graph = gtsam.NonlinearFactorGraph()
        graph.addPriorVector(X(1), np.zeros(1), gtsam.noiseModel.Unit.Create(1))
        feedback = lambda k, x, ref, u: ref - x
        draws = []

        def generator(initial, mean, seed):
            draws.append((initial.atVector(X(0)).item(), seed))
            return mean + np.random.default_rng(seed).standard_normal(mean.shape)

        proposal = unstable.PolicyAugmentedProposal.withSeededGenerator(np.eye(1), 3, 1, generator)
        tube = tracking.tube(planner, graph, nominal, actual, dynamics, feedback, proposal, 0.0, 1)
        self.assertEqual(len(draws), 2)
        self.assertEqual(draws[0][1], draws[1][1])
        self.assertNotEqual(draws[0][0], draws[1][0])
        robust = tracking.robust(planner, graph, nominal, actual, dynamics, feedback,
                                 proposal, 3.0, 3, 0, 1, np.eye(1))
        for plan in (tube, robust):
            np.testing.assert_array_equal(plan.actual.atVector(X(0)), [1.0])
            self.assertTrue(np.isfinite(graph.error(plan.actual)))


if __name__ == "__main__":
    unittest.main()
