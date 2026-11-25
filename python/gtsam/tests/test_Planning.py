"""MPPI proposals, rollout dynamics and graph scoring through the bindings.

Authors: Akash Ratheesh and Frank Dellaert. See LICENSE for details.
"""

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import U, X


class TestPlanning(unittest.TestCase):
    def test_graph_scores_and_mppi_update(self):
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1), X(2)], [U(0), U(1)], 0.25)
        initial = gtsam.Values()
        initial.insert(X(0), np.array([1.0]))
        initial.insert(U(0), np.array([0.4]))
        initial.insert(U(1), np.array([-0.1]))
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.PriorFactorVector(X(2), np.zeros(1), gtsam.noiseModel.Unit.Create(1)))
        field = lambda x, u: u
        proposal = gtsam.GaussianProposal(np.eye(1), 8, 17)
        rule = gtsam.MPPI(2)
        search = proposal.initialize(planner.nominalControls(initial))
        batch = proposal.sample(search, initial, 0.25)
        plans = batch.controls.reshape(8, 2)
        expected = 0.5 * (1 + 0.25 * np.sum(plans, axis=1)) ** 2
        scores = planner.rolloutAndScore(graph, initial, batch.controls, field)
        np.testing.assert_allclose(scores, expected)
        rule.apply(proposal, search, batch, scores)
        np.testing.assert_allclose(search.mean, [rule.weights(expected) @ plans])

        result = planner.optimize(graph, initial, field, gtsam.GaussianProposal(np.eye(1), 8, 17),
                                  rule, 1)
        np.testing.assert_allclose(planner.nominalControls(result), search.mean)
        np.testing.assert_allclose(result.atVector(X(2)), 1 + 0.25 * np.sum(search.mean))

    def test_discrete_step_is_not_integrated_again(self):
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1)], [U(0)], 0.01)
        initial = gtsam.Values()
        initial.insert(X(0), np.array([2.0]))
        controls = np.array([[3.0]])
        function = lambda x, u: x + u
        field = planner.rollout(initial, controls, function)
        step = planner.rollout(initial, controls, gtsam.discreteTransition(function))
        np.testing.assert_allclose(field.atVector(X(1)), [2.05])
        np.testing.assert_allclose(step.atVector(X(1)), [5.0])

    def test_draws_own_their_buffers(self):
        sampler = gtsam.GaussianNoiseSampler(np.eye(2), 4, 21)
        first = sampler.sampleControls(np.zeros((2, 3)))
        saved = first.copy()
        sampler.sampleControls(np.ones((2, 3)))
        np.testing.assert_array_equal(first, saved)


if __name__ == "__main__":
    unittest.main()
