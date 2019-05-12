"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

TestCase class with GTSAM assert utils.
Author: Frank Dellaert
"""
import unittest


class GtsamTestCase(unittest.TestCase):
    """TestCase class with GTSAM assert utils."""

    def gtsamAssertEquals(self, actual, expected, tol=1e-9):
        """ AssertEqual function that prints out actual and expected if not equal.
            Usage:
                self.gtsamAssertEqual(actual,expected)
            Keyword Arguments:
                tol {float} -- tolerance passed to 'equals', default 1e-9
        """
        equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Values are not equal:\n{}!={}".format(actual, expected))
