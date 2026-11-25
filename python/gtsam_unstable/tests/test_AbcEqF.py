"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the Attitude-Bias-Calibration equivariant filter wrapper.

The filter is implemented as gtsam::abc::AbcEqF in
gtsam_unstable/geometry/AbcEqF.h. The gtsam wrapper still exposes it as
gtsam.abc.AbcEquivariantFilter{1,2,3} through the forwarding header
gtsam_unstable/geometry/ABCEquivariantFilter.h, which these tests guard.
Author: Rohan Bansal
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase

FILTERS = {
    1: gtsam.abc.AbcEquivariantFilter1,
    2: gtsam.abc.AbcEquivariantFilter2,
    3: gtsam.abc.AbcEquivariantFilter3,
}


class TestAbcEqF(GtsamTestCase):
    """ABC filter construction, prediction, and direction updates."""

    def test_default_construction(self):
        for n, Filter in FILTERS.items():
            with self.subTest(n=n):
                abc_filter = Filter()
                self.gtsamAssertEquals(abc_filter.attitude(), gtsam.Rot3())
                np.testing.assert_allclose(abc_filter.bias(), np.zeros(3))
                for i in range(n):
                    self.gtsamAssertEquals(abc_filter.calibration(i),
                                           gtsam.Rot3())

    def test_predict_integrates_angular_velocity(self):
        for n, Filter in FILTERS.items():
            with self.subTest(n=n):
                abc_filter = Filter(np.eye(6 + 3 * n))
                omega = np.array([0.1, -0.2, 0.3])
                for _ in range(100):
                    abc_filter.predict(omega, 1e-4 * np.eye(6), 0.01)
                self.gtsamAssertEquals(abc_filter.attitude(),
                                       gtsam.Rot3.Expmap(omega), 1e-6)

    def test_consistent_update_keeps_identity(self):
        for n, Filter in FILTERS.items():
            with self.subTest(n=n):
                abc_filter = Filter()
                direction = gtsam.Unit3(np.array([0.0, 0.0, 1.0]))
                for i in range(n):
                    abc_filter.update(direction, direction, 1e-2 * np.eye(3), i)
                self.gtsamAssertEquals(abc_filter.attitude(), gtsam.Rot3(),
                                       1e-9)
                np.testing.assert_allclose(abc_filter.bias(), np.zeros(3),
                                           atol=1e-9)


if __name__ == "__main__":
    unittest.main()
