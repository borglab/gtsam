import unittest
import gtsam
import numpy as np

class TestPriorFactor(unittest.TestCase):

    def test_PriorFactor(self):
        values = gtsam.Values()

        key = 5
        priorPose3 = gtsam.Pose3()
        model = gtsam.noiseModel_Unit.Create(6)
        factor = gtsam.PriorFactorPose3(key, priorPose3, model)
        values.insert(key, priorPose3)
        self.assertEqual(factor.error(values), 0)

        key = 3
        priorVector = np.array([0., 0., 0.])
        model = gtsam.noiseModel_Unit.Create(3)
        factor = gtsam.PriorFactorVector(key, priorVector, model)
        values.insert(key, priorVector)
        self.assertEqual(factor.error(values), 0)

if __name__ == "__main__":
    unittest.main()
