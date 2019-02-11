"""Pose2 unit tests."""
import unittest

import numpy as np

from gtsam import Pose2


class TestPose2(unittest.TestCase):
    """Test selected Pose2 methods."""

    def test_adjoint(self):
        """Test adjoint method."""
        xi = np.array([1, 2, 3])
        expected = np.dot(Pose2.adjointMap_(xi), xi)
        actual = Pose2.adjoint_(xi, xi)
        np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
