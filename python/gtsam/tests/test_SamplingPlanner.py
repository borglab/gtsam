"""Python dynamics, policy proposals and factor callbacks in a native sampling batch.

Authors: Frank Dellaert and Akash Ratheesh. See LICENSE for details.
"""

import threading
import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import U, X


def _zero_values(keys):
    """Build a Values container with 1-D zero vectors for every key."""
    values = gtsam.Values()
    for key in keys:
        values.insert(key, np.zeros(1))
    return values


class TestSamplingPlanner(unittest.TestCase):
    def test_sampler_buffer_and_seed(self):
        """A new horizon may resize native storage while a previous draw is live."""
        seed = 2**60 + 37
        sampler = gtsam.GaussianNoiseSampler(np.eye(2), 4, seed)

        # Drawing a larger horizon must not overwrite an earlier returned array.
        first = sampler.sampleNoise(3)
        expected = first.copy()
        sampler.sampleNoise(8)
        sampler.sampleControls(np.zeros((2, 3)))

        self.assertEqual(sampler.seed(), seed)
        self.assertEqual(sampler.stepCounter(), 3)
        np.testing.assert_array_equal(first, expected)

    def test_custom_factor_scored_on_calling_thread(self):
        """Python graph callbacks keep their thread affinity with either scoring option."""
        caller = threading.get_ident()
        initial = _zero_values([X(0), U(0)])
        threads = []

        def terminal_error(factor, values, jacobians):
            threads.append(threading.get_ident())
            return values.atVector(X(1))

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.CustomFactor(
            gtsam.noiseModel.Unit.Create(1), [X(1)], terminal_error))

        for options in ({}, {"parallelCandidateScoring": True}):
            with self.subTest(options=options):
                threads.clear()
                planner = gtsam.SamplingPlannerVectorVector(
                    [X(0), X(1)], [U(0)], 1.0, **options)
                planner.optimize(graph, initial, lambda x, u: u,
                                 gtsam.GaussianProposal(np.eye(1), 64, 13),
                                 gtsam.MPPI(), 1)
                self.assertEqual(threads, [caller] * 64)

    def test_discrete_search_rules(self):
        """Each SamplingAlgorithm rule scores the same explicit discrete transition."""
        planner = gtsam.SamplingPlannerVectorVector([X(0), X(1)], [U(0)], 0.1)
        step = gtsam.discreteTransition(lambda x, u: x + u)
        seed = gtsam.Values()
        seed.insert(X(0), np.array([1.]))
        initial = planner.rollout(seed, np.zeros((1, 1)), step)

        graph = gtsam.NonlinearFactorGraph()
        graph.addPriorVector(X(1), np.array([2.]), gtsam.noiseModel.Unit.Create(1))

        rules = (
            gtsam.MPPI(),
            gtsam.TsallisMPPI(10.0, 2.0, 1.0),
            gtsam.CEM(4),
            gtsam.EliteMPPI(4),
            gtsam.PredictiveSampling(),
        )
        for rule in rules:
            with self.subTest(rule=type(rule).__name__):
                proposal = gtsam.GaussianProposal(np.eye(1), 32, 17)
                plan = planner.optimize(graph, initial, step, proposal, rule, 2)
                np.testing.assert_allclose(plan.atVector(X(1)), 1. + plan.atVector(U(0)))
                self.assertLess(graph.error(plan), graph.error(initial))

    def test_pose_checkpoint_and_resume(self):
        """A caller-owned CEM fit retains its mean and spread between iterations."""
        initial = gtsam.Values()
        initial.insert(X(0), gtsam.Pose2())
        initial.insert(U(0), np.zeros(2))
        planner = gtsam.SamplingPlannerPose2Vector([X(0), X(1)], [U(0)], 0.25)
        dynamics = lambda pose, u: np.array([u[0], 0.0, u[1]])

        # Roll out a constant-velocity policy to define the target pose.
        result = planner.rolloutPolicy(initial, lambda k, pose: np.array([2.0, 0.0]), dynamics)
        self.assertAlmostEqual(result.atPose2(X(1)).x(), 0.5)

        # Advance a caller-owned search state across two CEM iterations.
        proposal = gtsam.ColoredNoiseProposal(np.eye(2), 12, 1.0, 22)
        search = proposal.initialize(planner.nominalControls(initial))
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.PriorFactorPose2(
            X(1), result.atPose2(X(1)), gtsam.noiseModel.Unit.Create(3)))
        for _ in range(2):
            planner.iterate(graph, initial, dynamics, proposal, gtsam.CEM(3), search)

        self.assertEqual(search.iteration, 2)
        self.assertEqual(search.mean.shape, (2, 1))
        self.assertTrue(np.all(search.stddev > 0))

    def test_tracking_proposals(self):
        """Both controllers accept Gaussian, colored, derivative-space and piecewise-linear draws."""
        states, controls = [X(0), X(1), X(2)], [U(0), U(1)]
        nominal = _zero_values(states + controls)
        actual = gtsam.Values(nominal)
        actual.update(X(0), np.ones(1))

        graph = gtsam.NonlinearFactorGraph()
        graph.addPriorVector(X(2), np.zeros(1), gtsam.noiseModel.Unit.Create(1))
        planner = gtsam.SamplingPlannerVectorVector(states, controls, 1.)
        tracking = gtsam.MPPITracking(np.eye(1), 16, 1.)
        dynamics = lambda x, u: u
        feedback = lambda k, x, ref, u: ref - x

        proposals = (
            gtsam.GaussianProposal(np.eye(1), 4),
            gtsam.ColoredNoiseProposal(np.eye(1), 4, 2.),
            gtsam.NormalLognormalProposal(np.eye(1), 4, 0.5),
            gtsam.SmoothControlProposal(np.eye(1), 4),
            gtsam.PiecewiseLinearProposal(np.eye(1), 4, 1),
        )
        for proposal in proposals:
            with self.subTest(proposal=type(proposal).__name__):
                tube = tracking.tube(planner, graph, nominal, actual, dynamics, feedback,
                                     proposal, threshold=0., iterations=1)
                robust = tracking.robust(planner, graph, nominal, actual, dynamics, feedback,
                                         proposal, threshold=3., candidates=3, stride=1,
                                         iterations=1, feedbackPrecision=np.eye(1))
                for plan in (tube, robust):
                    np.testing.assert_array_equal(plan.actual.atVector(X(0)), [1.])
                    self.assertTrue(np.isfinite(graph.error(plan.actual)))


if __name__ == "__main__":
    unittest.main()
