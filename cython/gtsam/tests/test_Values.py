import unittest
import numpy as np

from gtsam import Point2, Point3, Unit3, Rot2, Pose2, Rot3, Pose3
from gtsam import Values, Cal3_S2, Cal3DS2, Cal3Bundler, EssentialMatrix, imuBias_ConstantBias


class TestValues(unittest.TestCase):

    def test_values(self):
        values = Values()
        E = EssentialMatrix(Rot3(), Unit3())
        tol = 1e-9

        values.insert(0, Point2(0,0))
        values.insert(1, Point3(0,0,0))
        values.insert(2, Rot2())
        values.insert(3, Pose2())
        values.insert(4, Rot3())
        values.insert(5, Pose3())
        values.insert(6, Cal3_S2())
        values.insert(7, Cal3DS2())
        values.insert(8, Cal3Bundler())
        values.insert(9, E)
        values.insert(10, imuBias_ConstantBias())

        # Special cases for Vectors and Matrices
        # Note that gtsam's Eigen Vectors and Matrices requires double-precision
        # floating point numbers in column-major (Fortran style) storage order,
        # whereas by default, numpy.array is in row-major order and the type is
        # in whatever the number input type is, e.g. np.array([1,2,3])
        # will have 'int' type.
        #
        # The wrapper will automatically fix the type and storage order for you,
        # but for performance reasons, it's recommended to specify the correct
        # type and storage order.
        vec = np.array([1., 2., 3.]) # for vectors, the order is not important, but dtype still is
        values.insert(11, vec)
        mat = np.array([[1., 2.], [3., 4.]], order='F')
        values.insert(12, mat)
        # Test with dtype int and the default order='C'
        # This still works as the wrapper converts to the correct type and order for you
        # but is nornally not recommended!
        mat2 = np.array([[1,2,],[3,5]])
        values.insert(13, mat2)

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
        self.assertTrue(np.allclose(vec, actualVector, tol))
        actualMatrix = values.atMatrix(12)
        self.assertTrue(np.allclose(mat, actualMatrix, tol))
        actualMatrix2 = values.atMatrix(13)
        self.assertTrue(np.allclose(mat2, actualMatrix2, tol))

if __name__ == "__main__":
    unittest.main()
