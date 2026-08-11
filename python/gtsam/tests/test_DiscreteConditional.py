"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Conditionals.
Author: Varun Agrawal
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

from gtsam.utils.test_case import GtsamTestCase

from gtsam import DecisionTreeFactor, DiscreteConditional, DiscreteKeys

# Some DiscreteKeys for binary variables:
A = 0, 2
B = 1, 2
C = 2, 2
D = 4, 2
E = 3, 2


class TestDiscreteConditional(GtsamTestCase):
    """Tests for Discrete Conditionals."""

    def test_single_value_versions(self):
        X = (0, 2)
        Y = (1, 3)
        conditional = DiscreteConditional(X, [Y], "2/8 4/6 5/5")

        actual0 = conditional.likelihood(0)
        expected0 = DecisionTreeFactor(Y, "0.2 0.4 0.5")
        self.gtsamAssertEquals(actual0, expected0, 1e-9)

        actual1 = conditional.likelihood(1)
        expected1 = DecisionTreeFactor(Y, "0.8 0.6 0.5")
        self.gtsamAssertEquals(actual1, expected1, 1e-9)

        actual = conditional.sample(2)
        self.assertIsInstance(actual, int)

    def test_multiply(self):
        """Check calculation of joint P(A,B)"""
        conditional = DiscreteConditional(A, [B], "1/2 2/1")
        prior = DiscreteConditional(B, "1/2")

        # P(A,B) = P(A|B) * P(B) = P(B) * P(A|B)
        for actual in [prior * conditional, conditional * prior]:
            self.assertEqual(2, actual.nrFrontals())
            for v, value in actual.enumerate():
                self.assertAlmostEqual(actual(v), conditional(v) * prior(v))

    def test_multiply2(self):
        """Check calculation of conditional joint P(A,B|C)"""
        A_given_B = DiscreteConditional(A, [B], "1/3 3/1")
        B_given_C = DiscreteConditional(B, [C], "1/3 3/1")

        # P(A,B|C) = P(A|B)P(B|C) = P(B|C)P(A|B)
        for actual in [A_given_B * B_given_C, B_given_C * A_given_B]:
            self.assertEqual(2, actual.nrFrontals())
            self.assertEqual(1, actual.nrParents())
            for v, value in actual.enumerate():
                self.assertAlmostEqual(actual(v), A_given_B(v) * B_given_C(v))

    def test_multiply4(self):
        """Check calculation of joint P(A,B,C|D,E) = P(A,B|D) P(C|D,E)"""
        A_given_B = DiscreteConditional(A, [B], "1/3 3/1")
        B_given_D = DiscreteConditional(B, [D], "1/3 3/1")
        AB_given_D = A_given_B * B_given_D
        C_given_DE = DiscreteConditional(C, [D,  E], "4/1 1/1 1/1 1/4")

        # P(A,B,C|D,E) = P(A,B|D) P(C|D,E) = P(C|D,E) P(A,B|D)
        for actual in [AB_given_D * C_given_DE, C_given_DE * AB_given_D]:
            self.assertEqual(3, actual.nrFrontals())
            self.assertEqual(2, actual.nrParents())
            for v, value in actual.enumerate():
                self.assertAlmostEqual(
                    actual(v), AB_given_D(v) * C_given_DE(v))

    def test_marginals(self):
        conditional = DiscreteConditional(A, [B], "1/2 2/1")
        prior = DiscreteConditional(B, "1/2")
        pAB = prior * conditional
        self.gtsamAssertEquals(prior, pAB.marginal(B[0]))

        pA = DiscreteConditional(A, "5/4")
        self.gtsamAssertEquals(pA, pAB.marginal(A[0]))

    def test_markdown(self):
        """Test whether the _repr_markdown_ method."""

        A = (2, 2)
        B = (1, 2)
        C = (0, 3)
        parents = DiscreteKeys()
        parents.push_back(B)
        parents.push_back(C)

        conditional = DiscreteConditional(A, parents,
                                          "0/1 1/3  1/1 3/1  0/1 1/0")
        expected = " *P(A|B,C):*\n\n" \
            "|*B*|*C*|0|1|\n" \
            "|:-:|:-:|:-:|:-:|\n" \
            "|0|0|0|1|\n" \
            "|0|1|0.25|0.75|\n" \
            "|0|2|0.5|0.5|\n" \
            "|1|0|0.75|0.25|\n" \
            "|1|1|0|1|\n" \
            "|1|2|1|0|\n"

        def formatter(x: int):
            names = ["C", "B", "A"]
            return names[x]

        actual = conditional._repr_markdown_(formatter)
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
