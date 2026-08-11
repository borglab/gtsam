"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Search.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

from dfg_utils import generate_observation_cpt, generate_transition_cpt, make_key
from gtsam.utils.test_case import GtsamTestCase

from gtsam import (
    DiscreteConditional,
    DiscreteFactorGraph,
    DiscreteSearch,
    Ordering,
    DefaultKeyFormatter,
)

OrderingType = Ordering.OrderingType


class TestDiscreteSearch(GtsamTestCase):
    """Tests for Discrete Factor Graphs."""

    def test_MPE_chain(self):
        """
        Test for numerical underflow in EliminateMPE on long chains.
        Adapted from the toy problem of @pcl15423
        Ref: https://github.com/borglab/gtsam/issues/1448
        """
        num_states = 3
        num_obs = 200
        desired_state = 1
        states = list(range(num_states))

        X = {index: make_key("X", index, len(states)) for index in range(num_obs)}
        Z = {index: make_key("Z", index, num_obs + 1) for index in range(num_obs)}
        graph = DiscreteFactorGraph()

        transition_cpt = generate_transition_cpt(num_states)
        for i in reversed(range(1, num_obs)):
            transition_conditional = DiscreteConditional(
                X[i], [X[i - 1]], transition_cpt
            )
            graph.push_back(transition_conditional)

        # Contrived example such that the desired state gives measurements [0, num_obs) with equal
        # probability but all other states always give measurement num_obs
        obs_cpt = generate_observation_cpt(num_states, num_obs, desired_state)
        # Contrived example where each measurement is its own index
        for i in range(num_obs):
            obs_conditional = DiscreteConditional(Z[i], [X[i]], obs_cpt)
            factor = obs_conditional.likelihood(i)
            graph.push_back(factor)

        # Check MPE
        mpe = graph.optimize()
        vals = [mpe[X[i][0]] for i in range(num_obs)]
        self.assertEqual(vals, [desired_state] * num_obs)

        # Create an ordering:
        ordering = Ordering()
        for i in reversed(range(num_obs)):
            ordering.push_back(X[i][0])

        # Now do Search
        search = DiscreteSearch.FromFactorGraph(graph, ordering)
        solutions = search.run(K=1)
        mpe2 = solutions[0].assignment
        # print({DefaultKeyFormatter(key): value for key, value in mpe2.items()})
        vals = [mpe2[X[i][0]] for i in range(num_obs)]
        self.assertEqual(vals, [desired_state] * num_obs)


if __name__ == "__main__":
    unittest.main()
