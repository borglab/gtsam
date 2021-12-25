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

from gtsam import DiscreteConditional, DiscreteKeys
from gtsam.utils.test_case import GtsamTestCase


class TestDiscreteConditional(GtsamTestCase):
    """Tests for Discrete Conditionals."""
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
        expected = \
            " $P(A|B,C)$:\n" \
            "|B|C|0|1|\n" \
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
