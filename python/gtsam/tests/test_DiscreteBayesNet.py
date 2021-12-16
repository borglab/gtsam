"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Bayes Nets.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

from gtsam import (DiscreteBayesNet, DiscreteConditional, DiscreteFactorGraph,
                   DiscreteKeys, DiscreteValues, Ordering)
from gtsam.utils.test_case import GtsamTestCase


class TestDiscreteBayesNet(GtsamTestCase):
    """Tests for Discrete Bayes Nets."""

    def test_constructor(self):
        """Test constructing a Bayes net."""

        bayesNet = DiscreteBayesNet()
        Parent, Child = (0, 2), (1, 2)
        empty = DiscreteKeys()
        prior = DiscreteConditional(Parent, empty, "6/4")
        bayesNet.add(prior)

        parents = DiscreteKeys()
        parents.push_back(Parent)
        conditional = DiscreteConditional(Child, parents, "7/3 8/2")
        bayesNet.add(conditional)

        # Check conversion to factor graph:
        fg = DiscreteFactorGraph(bayesNet)
        self.assertEqual(fg.size(), 2)
        self.assertEqual(fg.at(1).size(), 2)

    def test_Asia(self):
        """Test full Asia example."""

        Asia = (0, 2)
        Smoking = (4, 2)
        Tuberculosis = (3, 2)
        LungCancer = (6, 2)

        Bronchitis = (7, 2)
        Either = (5, 2)
        XRay = (2, 2)
        Dyspnea = (1, 2)

        def P(keys):
            dks = DiscreteKeys()
            for key in keys:
                dks.push_back(key)
            return dks

        asia = DiscreteBayesNet()
        asia.add(Asia, P([]), "99/1")
        asia.add(Smoking, P([]), "50/50")

        asia.add(Tuberculosis, P([Asia]), "99/1 95/5")
        asia.add(LungCancer, P([Smoking]), "99/1 90/10")
        asia.add(Bronchitis, P([Smoking]), "70/30 40/60")

        asia.add(Either, P([Tuberculosis, LungCancer]), "F T T T")

        asia.add(XRay, P([Either]), "95/5 2/98")
        asia.add(Dyspnea, P([Either, Bronchitis]), "9/1 2/8 3/7 1/9")

        # Convert to factor graph
        fg = DiscreteFactorGraph(asia)

        # Create solver and eliminate
        ordering = Ordering()
        for j in range(8):
            ordering.push_back(j)
        chordal = fg.eliminateSequential(ordering)
        expected2 = DiscreteConditional(Bronchitis, P([]), "11/9")
        self.gtsamAssertEquals(chordal.at(7), expected2)

        # solve
        actualMPE = chordal.optimize()
        expectedMPE = DiscreteValues()
        for key in [Asia, Dyspnea, XRay, Tuberculosis, Smoking, Either, LungCancer, Bronchitis]:
            expectedMPE[key[0]] = 0
        self.assertEqual(list(actualMPE.items()),
                         list(expectedMPE.items()))

        # Check value for MPE is the same
        self.assertAlmostEqual(asia(actualMPE), fg(actualMPE))

        # add evidence, we were in Asia and we have dyspnea
        fg.add(Asia, "0 1")
        fg.add(Dyspnea, "0 1")

        # solve again, now with evidence
        chordal2 = fg.eliminateSequential(ordering)
        actualMPE2 = chordal2.optimize()
        expectedMPE2 = DiscreteValues()
        for key in [XRay, Tuberculosis, Either, LungCancer]:
            expectedMPE2[key[0]] = 0
        for key in [Asia, Dyspnea, Smoking, Bronchitis]:
            expectedMPE2[key[0]] = 1
        self.assertEqual(list(actualMPE2.items()),
                         list(expectedMPE2.items()))

        # now sample from it
        actualSample = chordal2.sample()
        self.assertEqual(len(actualSample), 8)


if __name__ == "__main__":
    unittest.main()
