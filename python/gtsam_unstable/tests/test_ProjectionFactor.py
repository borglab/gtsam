"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

ProjectionFactorPPP unit tests.
Author: Varun Agrawal
"""
import unittest

import gtsam
import gtsam_unstable
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestProjectionFactorPPP(GtsamTestCase):
    """Test the ProjectionFactorPPP factor."""
    def test_serialization(self):
        """Test if ProjectionFactorPPP serialization works correctly."""
        factor = gtsam_unstable.ProjectionFactorPPPCal3_S2(
            np.ones(2), gtsam.noiseModel.Isotropic.Sigmas(np.ones(2)), 0, 1, 2,
            gtsam.Cal3_S2())
        serialized = factor.serialize()

        # Initialize differently on purpose
        actual = gtsam_unstable.ProjectionFactorPPPCal3_S2(
            np.ones(2), gtsam.noiseModel.Isotropic.Sigmas(np.zeros(2)), 0, 0,
            0, gtsam.Cal3_S2())
        actual.deserialize(serialized)

        self.gtsamAssertEquals(factor, actual, 1e-10)


if __name__ == '__main__':
    unittest.main()
