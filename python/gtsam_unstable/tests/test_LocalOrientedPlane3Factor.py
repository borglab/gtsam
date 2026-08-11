import unittest
import numpy as np
import gtsam
from gtsam.utils.test_case import GtsamTestCase
import gtsam_unstable

class TestLocalOrientedPlane3Factor(GtsamTestCase):
    def test_factor_error(self):
        # 1. Create the factor
        z = np.array([0.0, 0.0, 1.0, 1.5])
        noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        
        pose_key = gtsam.symbol('x', 1)
        anchor_key = gtsam.symbol('x', 0)
        landmark_key = gtsam.symbol('l', 0)
        
        factor = gtsam_unstable.LocalOrientedPlane3Factor(
            z, noise, pose_key, anchor_key, landmark_key)

        # 2. Create Values
        values = gtsam.Values()
        values.insert(pose_key, gtsam.Pose3())
        values.insert(anchor_key, gtsam.Pose3())
        values.insert(landmark_key, gtsam.OrientedPlane3(0.0, 0.0, 1.0, 1.5))
        
        # 3. Check that the error is zero for perfect measurements
        error = factor.error(values)
        self.assertAlmostEqual(error, 0.0, places=5)

if __name__ == "__main__":
    unittest.main()