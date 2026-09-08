"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Values unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
# pylint: disable=invalid-name, E1101, E0611
import unittest

import numpy as np

import gtsam
from gtsam import (Cal3_S2, Cal3Bundler, Cal3DS2, EssentialMatrix, Point2,
                   Point3, Pose2, Pose3, Rot2, Rot3, Unit3, Values, imuBias)
from gtsam.utils.test_case import GtsamTestCase


class TestValues(GtsamTestCase):

    def test_values(self):
        values = Values()
        E = EssentialMatrix(Rot3(), Unit3())
        tol = 1e-9

        values.insert(0, Point2(0, 0))
        values.insert(1, Point3(0, 0, 0))
        values.insert(2, Rot2())
        values.insert(3, Pose2())
        values.insert(4, Rot3())
        values.insert(5, Pose3())
        values.insert(6, Cal3_S2())
        values.insert(7, Cal3DS2())
        values.insert(8, Cal3Bundler())
        values.insert(9, E)
        values.insert(10, imuBias.ConstantBias())

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
        # for vectors, the order is not important, but dtype still is
        vec = np.array([1., 2., 3.])
        values.insert(11, vec)
        mat = np.array([[1., 2.], [3., 4.]], order='F')
        values.insert(12, mat)
        # Test with dtype int and the default order='C'
        # This still works as the wrapper converts to the correct type and order for you
        # but is nornally not recommended!
        mat2 = np.array([[1, 2, ], [3, 5]])
        values.insert(13, mat2)

        self.gtsamAssertEquals(values.atPoint2(0), Point2(0,0), tol)
        self.gtsamAssertEquals(values.atPoint3(1), Point3(0,0,0), tol)
        self.gtsamAssertEquals(values.atRot2(2), Rot2(), tol)
        self.gtsamAssertEquals(values.atPose2(3), Pose2(), tol)
        self.gtsamAssertEquals(values.atRot3(4), Rot3(), tol)
        self.gtsamAssertEquals(values.atPose3(5), Pose3(), tol)
        self.gtsamAssertEquals(values.atCal3_S2(6), Cal3_S2(), tol)
        self.gtsamAssertEquals(values.atCal3DS2(7), Cal3DS2(), tol)
        self.gtsamAssertEquals(values.atCal3Bundler(8), Cal3Bundler(), tol)
        self.gtsamAssertEquals(values.atEssentialMatrix(9), E, tol)
        self.gtsamAssertEquals(values.atConstantBias(
            10), imuBias.ConstantBias(), tol)

        # special cases for Vector and Matrix:
        actualVector = values.atVector(11)
        np.testing.assert_allclose(vec, actualVector, tol)
        actualMatrix = values.atMatrix(12)
        np.testing.assert_allclose(mat, actualMatrix, tol)
        actualMatrix2 = values.atMatrix(13)
        np.testing.assert_allclose(mat2, actualMatrix2, tol)

    def test_extract_by_keys(self):
        """extract(keys) returns a Values with copies of the named values,
        of any type, without the caller knowing each key's type."""
        values = gtsam.Values()
        values.insert(0, Point2(1.0, 2.0))
        values.insert(1, Pose3())
        values.insert(2, np.array([4.0, 5.0, 6.0]))
        values.insert(3, Rot3())

        subset = values.extract([2, 0])
        self.assertEqual(subset.size(), 2)
        self.assertTrue(subset.exists(0))
        self.assertTrue(subset.exists(2))
        self.assertFalse(subset.exists(1))
        np.testing.assert_allclose(subset.atPoint2(0), Point2(1.0, 2.0))
        np.testing.assert_allclose(subset.atVector(2), np.array([4.0, 5.0, 6.0]))

        # copies, not references
        subset.update(0, Point2(9.0, 9.0))
        np.testing.assert_allclose(values.atPoint2(0), Point2(1.0, 2.0))

        self.assertEqual(values.extract([]).size(), 0)
        with self.assertRaises(RuntimeError):
            values.extract([0, 99])


class TestCalculateEstimateForKeys(GtsamTestCase):
    """calculateEstimate(keys) on ISAM2 and both fixed-lag smoothers."""

    X, V, B, S = 0, 100, 200, 300

    def _graph(self):
        """A Pose3 / velocity / bias / double set, so the subset spans types."""
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.push_back(gtsam.PriorFactorPose3(
            self.X, Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
        values.insert(self.X, Pose3(Rot3.Rz(0.05), Point3(0.1, 0.0, 0.0)))
        graph.push_back(gtsam.PriorFactorPoint3(
            self.V, Point3(1.0, 0.0, 0.0), gtsam.noiseModel.Isotropic.Sigma(3, 0.1)))
        values.insert(self.V, Point3(0.9, 0.1, 0.0))
        graph.push_back(gtsam.PriorFactorConstantBias(
            self.B, gtsam.imuBias.ConstantBias(),
            gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
        values.insert(self.B, gtsam.imuBias.ConstantBias())
        graph.push_back(gtsam.PriorFactorDouble(
            self.S, 2.0, gtsam.noiseModel.Isotropic.Sigma(1, 0.1)))
        values.insert(self.S, 1.9)
        return graph, values

    def _check(self, subset, full):
        """The subset holds exactly the requested keys, equal to the full estimate."""
        self.assertEqual(subset.size(), 3)
        self.assertFalse(subset.exists(self.B))
        self.gtsamAssertEquals(subset.atPose3(self.X), full.atPose3(self.X), 1e-9)
        np.testing.assert_allclose(subset.atPoint3(self.V),
                                   full.atPoint3(self.V), atol=1e-9)
        self.assertAlmostEqual(subset.atDouble(self.S), full.atDouble(self.S),
                               places=9)

    def _requested(self):
        return [self.X, self.V, self.S]

    def test_isam2(self):
        graph, values = self._graph()
        isam = gtsam.ISAM2()
        isam.update(graph, values)
        # Request the subset first, so the full estimate cannot have warmed
        # anything the subset path depends on.
        subset = isam.calculateEstimate(self._requested())
        self._check(subset, isam.calculateEstimate())

    def test_incremental_fixed_lag_smoother(self):
        graph, values = self._graph()
        smoother = gtsam.IncrementalFixedLagSmoother(10.0)
        smoother.update(graph, values, {k: 0.0 for k in
                                        [self.X, self.V, self.B, self.S]})
        subset = smoother.calculateEstimate(self._requested())
        self._check(subset, smoother.calculateEstimate())

    def test_batch_fixed_lag_smoother(self):
        graph, values = self._graph()
        smoother = gtsam.BatchFixedLagSmoother(10.0)
        smoother.update(graph, values, {k: 0.0 for k in
                                        [self.X, self.V, self.B, self.S]})
        subset = smoother.calculateEstimate(self._requested())
        self._check(subset, smoother.calculateEstimate())

    def test_missing_key_raises(self):
        graph, values = self._graph()
        isam = gtsam.ISAM2()
        isam.update(graph, values)
        with self.assertRaises(RuntimeError):
            isam.calculateEstimate([999])


if __name__ == "__main__":
    unittest.main()
