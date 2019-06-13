import unittest
from gtsam_py import gtsam
import numpy as np


class TestCal3Unified(unittest.TestCase):
    def test_Cal3Unified(self):
        K = gtsam.Cal3Unified()
        self.assertEqual(K.fx(), 1.0)
        self.assertEqual(K.fx(), 1.0)

    def test_retract(self):
        expected = gtsam.Cal3Unified(
            fx=100 + 2,
            fy=105 + 3,
            s=0.0 + 4,
            u0=320 + 5,
            v0=240 + 6,
            k1=1e-3 + 7,
            k2=2.0 * 1e-3 + 8,
            p1=3.0 * 1e-3 + 9,
            p2=4.0 * 1e-3 + 10,
            xi=0.1 + 1,
        )
        K = gtsam.Cal3Unified(
            fx=100,
            fy=105,
            s=0.0,
            u0=320,
            v0=240,
            k1=1e-3,
            k2=2.0 * 1e-3,
            p1=3.0 * 1e-3,
            p2=4.0 * 1e-3,
            xi=0.1,
        )
        d = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 1], order='F')
        actual = K.retract(d)
        self.assertTrue(actual.equals(expected, 1e-9))
        np.testing.assert_allclose(d, K.localCoordinates(actual))


if __name__ == "__main__":
    unittest.main()
