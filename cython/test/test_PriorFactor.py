import unittest
from gtsam import *
from math import *
import numpy as np

class TestPriorFactor(unittest.TestCase):

    def test_PriorFactor(self):
        values = Values()

        key = 5
        priorPose3 = Pose3()
        model = noiseModel_Unit.Create(6)
        factor = PriorFactorPose3(key, priorPose3, model)
        values.insertPose3(key, priorPose3)
        self.assertEqual(factor.error(values), 0)

        key = 3
        priorVector = np.array([0., 0., 0.])
        model = noiseModel_Unit.Create(3)
        factor = PriorFactorVector(key, priorVector, model)
        values.insertVector(key, priorVector)
        self.assertEqual(factor.error(values), 0)

if __name__ == "__main__":
    unittest.main()
