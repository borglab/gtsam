import unittest
import numpy as np

from util import Point2, Point3
from gtsam_py.gtsam import (
    Unit3,
    Rot2,
    Pose2,
    Rot3,
    Pose3,
    Values,
    Cal3_S2,
    Cal3DS2,
    Cal3Bundler,
    EssentialMatrix,
)
from gtsam_py.gtsam.imuBias import ConstantBias


class TestValues(unittest.TestCase):
    def test_values(self):
        values = Values()
        E = EssentialMatrix(Rot3(), Unit3())
        tol = 1e-9

        values.insert(0, np.array([0., 0.]))
        values.insert(1, Point3(0, 0, 0))
        values.insert(2, Rot2())
        values.insert(3, Pose2())
        values.insert(4, Rot3())
        values.insert(5, Pose3())
        values.insert(6, Cal3_S2())
        values.insert(7, Cal3DS2())
        values.insert(8, Cal3Bundler())
        values.insert(9, E)
        values.insert(10, ConstantBias())

        vec = np.array([1., 2., 3., 4.])
        values.insert(11, vec)
        mat = np.array([
            [1., 2.],
            [3., 4.],
        ], order='F')
        values.insert(12, mat)
        mat2 = np.array([
            [1, 2],
            [3, 5],
        ])
        values.insert(13, mat2)

        np.testing.assert_almost_equal(values.atPoint2(0), Point2(), tol)
        np.testing.assert_almost_equal(values.atPoint3(1), Point3(), tol)
        self.assertTrue(values.atRot2(2).equals(Rot2(), tol))
        self.assertTrue(values.atPose2(3).equals(Pose2(), tol))
        self.assertTrue(values.atRot3(4).equals(Rot3(), tol))
        self.assertTrue(values.atPose3(5).equals(Pose3(), tol))
        self.assertTrue(values.atCal3_S2(6).equals(Cal3_S2(), tol))
        self.assertTrue(values.atCal3DS2(7).equals(Cal3DS2(), tol))
        self.assertTrue(values.atCal3Bundler(8).equals(Cal3Bundler(), tol))
        self.assertTrue(values.atEssentialMatrix(9).equals(E, tol))
        self.assertTrue(values.atConstantBias(10).equals(ConstantBias(), tol))

        # special cases for Vector and Matrix:
        actualVector = values.atVector(11)
        self.assertTrue(np.allclose(vec, actualVector, tol))
        actualMatrix = values.atMatrix(12)
        self.assertTrue(np.allclose(mat, actualMatrix, tol))
        actualMatrix2 = values.atMatrix(13)
        self.assertTrue(np.allclose(mat2, actualMatrix2, tol))

        # failure cases:
        vec3 = np.array([1., 2., 3.])
        values.insert(14, vec3)
        values.atVector(14)  # won't work because gtsam thinks it's a Point3.

        # failure cases:
        values.serialize()  # serialize and serializable are not yet supported.


if __name__ == "__main__":
    unittest.main()
