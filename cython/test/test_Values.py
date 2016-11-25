import unittest
from gtsam import *
from math import *
import numpy as np

class TestValues(unittest.TestCase):

    def test_values(self):
        values = Values()
        E = EssentialMatrix(Rot3, Unit3)
        tol = 1e-9

        values.insertPoint2(0, Point2())
        values.insertPoint3(1, Point3())
        values.insertRot2(2, Rot2())
        values.insertPose2(3, Pose2())
        values.insertRot3(4, Rot3())
        values.insertPose3(5, Pose3())
        values.insertCal3_S2(6, Cal3_S2())
        values.insertCal3DS2(7, Cal3DS2())
        values.insertCal3Bundler(8, Cal3Bundler())
        values.insertEssentialMatrix(9, E)
        values.insertimuBias_ConstantBias(10, imuBias_ConstantBias())

        # special cases for Vector and Matrix:
        vec = np.array([1., 2., 3.])
        values.insertVector(11, vec)
        mat = np.array([[1., 2.], [3., 4.]], order='F')
        values.insertMatrix(12, mat)

        self.assertTrue(values.atPoint2(0).equals(Point2(), tol))
        self.assertTrue(values.atPoint3(1).equals(Point3(), tol))
        self.assertTrue(values.atRot2(2).equals(Rot2(), tol))
        self.assertTrue(values.atPose2(3).equals(Pose2(), tol))
        self.assertTrue(values.atRot3(4).equals(Rot3(), tol))
        self.assertTrue(values.atPose3(5).equals(Pose3(), tol))
        self.assertTrue(values.atCal3_S2(6).equals(Cal3_S2(), tol))
        self.assertTrue(values.atCal3DS2(7).equals(Cal3DS2(), tol))
        self.assertTrue(values.atCal3Bundler(8).equals(Cal3Bundler(), tol))
        self.assertTrue(values.atEssentialMatrix(9).equals(E, tol))
        self.assertTrue(values.atimuBias_ConstantBias(
            10).equals(imuBias_ConstantBias(), tol))

        # special cases for Vector and Matrix:
        actualVector = values.atVector(11)
        self.assertTrue(np.allclose(vec, actualVector.ravel(), tol))
        actualMatrix = values.atMatrix(12)
        self.assertTrue(np.allclose(mat, actualMatrix, tol))

if __name__ == "__main__":
    unittest.main()
