"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Bayes Nets.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import math
import textwrap
import unittest

from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DiscreteBayesNet, DiscreteConditional, DiscreteDistribution,
                   DiscreteFactorGraph, DiscreteKeys, DiscreteValues, Ordering)

# Some keys:
Asia = (0, 2)
Smoking = (4, 2)
Tuberculosis = (3, 2)
LungCancer = (6, 2)

Bronchitis = (7, 2)
Either = (5, 2)
XRay = (2, 2)
Dyspnea = (1, 2)


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

        asia = DiscreteBayesNet()
        asia.add(Asia, "99/1")
        asia.add(Smoking, "50/50")

        asia.add(Tuberculosis, [Asia], "99/1 95/5")
        asia.add(LungCancer, [Smoking], "99/1 90/10")
        asia.add(Bronchitis, [Smoking], "70/30 40/60")

        asia.add(Either, [Tuberculosis, LungCancer], "F T T T")

        asia.add(XRay, [Either], "95/5 2/98")
        asia.add(Dyspnea, [Either, Bronchitis], "9/1 2/8 3/7 1/9")

        # Convert to factor graph
        fg = DiscreteFactorGraph(asia)

        # Create solver and eliminate
        ordering = Ordering()
        for j in range(8):
            ordering.push_back(j)
        chordal = fg.eliminateSequential(ordering)
        expected2 = DiscreteDistribution(Bronchitis, "11/9")
        self.gtsamAssertEquals(chordal.at(7), expected2)

        # solve
        actualMPE = fg.optimize()
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
        actualMPE2 = fg.optimize()
        expectedMPE2 = DiscreteValues()
        for key in [XRay, Tuberculosis, Either, LungCancer]:
            expectedMPE2[key[0]] = 0
        for key in [Asia, Dyspnea, Smoking, Bronchitis]:
            expectedMPE2[key[0]] = 1
        self.assertEqual(list(actualMPE2.items()),
                         list(expectedMPE2.items()))

        # now sample from it
        chordal2 = fg.eliminateSequential(ordering)
        actualSample = chordal2.sample()
        # TODO(kartikarcot): Resolve the len function issue. Probably
        # due to a use of initializer list which is not supported in CPP17
        # self.assertEqual(len(actualSample), 8)

    def test_fragment(self):
        """Test evaluate/sampling/optimizing for Asia fragment."""

        # Create a reverse-topologically sorted fragment:
        fragment = DiscreteBayesNet()
        fragment.add(Either, [Tuberculosis, LungCancer], "F T T T")
        fragment.add(Tuberculosis, [Asia], "99/1 95/5")
        fragment.add(LungCancer, [Smoking], "99/1 90/10")

        # Create assignment with missing values:
        given = DiscreteValues()
        for key in [Asia, Smoking]:
            given[key[0]] = 0

        # Now sample from fragment:
        values = fragment.sample(given)
        # TODO(kartikarcot): Resolve the len function issue. Probably
        # due to a use of initializer list which is not supported in CPP17
        # self.assertEqual(len(values), 5)

        for i in [0, 1, 2]:
            self.assertAlmostEqual(fragment.at(i).logProbability(values),
                                   math.log(fragment.at(i).evaluate(values)))
        self.assertAlmostEqual(fragment.logProbability(values),
                               math.log(fragment.evaluate(values)))
        actual = fragment.sample(given)
        # TODO(kartikarcot): Resolve the len function issue. Probably
        # due to a use of initializer list which is not supported in CPP17
        # self.assertEqual(len(actual), 5)

    def test_dot(self):
        """Check that dot works with position hints."""
        fragment = DiscreteBayesNet()
        fragment.add(Either, [Tuberculosis, LungCancer], "F T T T")
        MyAsia = gtsam.symbol('a', 0), 2  # use a symbol!
        fragment.add(Tuberculosis, [MyAsia], "99/1 95/5")
        fragment.add(LungCancer, [Smoking], "99/1 90/10")

        # Make sure we can *update* position hints
        writer = gtsam.DotWriter()
        ph: dict = writer.positionHints
        ph['a'] = 2  # hint at symbol position
        writer.positionHints = ph

        # Check the output of dot
        actual = fragment.dot(writer=writer)
        expected_result = """\
            digraph {
              size="5,5";

              var3[label="3"];
              var4[label="4"];
              var5[label="5"];
              var6[label="6"];
              var6989586621679009792[label="a0", pos="0,2!"];

              var4->var6
              var6989586621679009792->var3
              var3->var5
              var6->var5
            }"""
        self.assertEqual(actual, textwrap.dedent(expected_result))


if __name__ == "__main__":
    unittest.main()
