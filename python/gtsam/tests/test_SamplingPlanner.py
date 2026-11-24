"""Compare search rules on the same Python discrete dynamics and cost graph.

Authors: Frank Dellaert and Akash Ratheesh. See LICENSE for details.
"""

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import U, X


class TestSamplingPlanner(unittest.TestCase):
    def test_discrete_search_rules(self):
        """Each rule scores the same explicit discrete transition."""
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1)], [U(0)], 0.1)
        step = gtsam.discreteTransition(lambda x, u: x + u)
        seed = gtsam.Values()
        seed.insert(X(0), np.array([1.]))
        initial = planner.rollout(seed, np.zeros((1, 1)), step)
        graph = gtsam.NonlinearFactorGraph()
        graph.addPriorVector(X(1), np.array([2.]), gtsam.noiseModel.Unit.Create(1))
        for rule in (gtsam.MPPI(), gtsam.CEM(4), gtsam.PredictiveSampling()):
            with self.subTest(rule=type(rule).__name__):
                proposal = gtsam.GaussianProposal(np.eye(1), 32, 17)
                plan = planner.optimize(graph, initial, step, proposal, rule, 2)
                np.testing.assert_allclose(plan.atVector(X(1)), 1. + plan.atVector(U(0)))
                self.assertLess(graph.error(plan), graph.error(initial))


if __name__ == "__main__":
    unittest.main()
