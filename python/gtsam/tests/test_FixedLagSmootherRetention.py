"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for retaining and releasing keys in the fixed-lag smoothers.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

import gtsam

NOISE = gtsam.noiseModel.Isotropic.Sigma(2, 0.1)
LAG = 2.0


def add_state(smoother, i, *retain_and_release):
    """Add X(i) at time i, with a prior for X(0) or odometry from X(i - 1)."""
    factors = gtsam.NonlinearFactorGraph()
    if i == 0:
        factors.push_back(
            gtsam.PriorFactorPoint2(X(0), gtsam.Point2(0, 0), NOISE))
    else:
        factors.push_back(
            gtsam.BetweenFactorPoint2(X(i - 1), X(i), gtsam.Point2(1, 0),
                                      NOISE))
    values = gtsam.Values()
    values.insert(X(i), gtsam.Point2(i, 0))
    return smoother.update(factors, values, {X(i): float(i)}, [],
                           *retain_and_release)


class TestFixedLagSmootherRetention(GtsamTestCase):
    """Both concrete smoothers retain and release keys through KeySet."""

    def smoothers(self):
        return [gtsam.BatchFixedLagSmoother(LAG),
                gtsam.IncrementalFixedLagSmoother(LAG)]

    def test_retain_and_release(self):
        """A retained 64-bit symbol key outlives the lag until released."""
        for smoother in self.smoothers():
            with self.subTest(smoother=type(smoother).__name__):
                add_state(smoother, 0)
                add_state(smoother, 1, gtsam.KeySet([X(1)]))

                # Later four-argument updates keep the persistent retained set.
                for i in range(2, 8):
                    add_state(smoother, i)

                for _ in range(2):  # Repeated retrieval returns the same set.
                    retained = smoother.retainedKeys()
                    self.assertIsInstance(retained, gtsam.KeySet)
                    self.gtsamAssertEquals(retained, gtsam.KeySet([X(1)]))
                self.assertGreater(X(1), 2**32)
                self.assertIn(X(1), smoother.timestamps())
                self.assertFalse(smoother.getLinearizationPoint().exists(X(0)))
                np.testing.assert_allclose(
                    smoother.calculateEstimatePoint2(X(1)), [1, 0], atol=1e-6)

                # X(1) is outside the lag, so releasing it marginalizes it now.
                smoother.update(gtsam.NonlinearFactorGraph(), gtsam.Values(),
                                {}, [], gtsam.KeySet(), gtsam.KeySet([X(1)]))
                self.assertTrue(smoother.retainedKeys().empty())
                self.assertNotIn(X(1), smoother.timestamps())
                self.assertFalse(smoother.getLinearizationPoint().exists(X(1)))
                self.assertTrue(smoother.getLinearizationPoint().exists(X(7)))

    def test_release_keyword_argument(self):
        """The retain and release sets can be passed by keyword."""
        for smoother in self.smoothers():
            with self.subTest(smoother=type(smoother).__name__):
                add_state(smoother, 0, gtsam.KeySet([X(0)]))
                for i in range(1, 4):
                    add_state(smoother, i)
                self.assertTrue(smoother.getLinearizationPoint().exists(X(0)))

                smoother.update(gtsam.NonlinearFactorGraph(), gtsam.Values(),
                                {}, [], keysToRetain=gtsam.KeySet(),
                                keysToRelease=gtsam.KeySet([X(0)]))
                self.assertTrue(smoother.retainedKeys().empty())
                self.assertFalse(smoother.getLinearizationPoint().exists(X(0)))

    def test_rejects_retaining_unknown_key(self):
        """Retaining a key absent from the smoother and newTheta raises."""
        for smoother in self.smoothers():
            with self.subTest(smoother=type(smoother).__name__):
                add_state(smoother, 0)
                with self.assertRaises(ValueError):
                    add_state(smoother, 1, gtsam.KeySet([X(2)]))
                self.assertTrue(smoother.retainedKeys().empty())
                self.assertFalse(smoother.getLinearizationPoint().exists(X(1)))


if __name__ == "__main__":
    unittest.main()
