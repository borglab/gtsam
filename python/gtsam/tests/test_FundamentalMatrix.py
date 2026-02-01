"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

FundamentalMatrix unit tests.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module
import unittest

import numpy as np
from gtsam.examples import SFMdata
from numpy.testing import assert_almost_equal

import gtsam
from gtsam import (Cal3_S2, EssentialMatrix, FundamentalMatrix,
                   PinholeCameraCal3_S2, Point2, Point3, Rot3,
                   SimpleFundamentalMatrix, Unit3)


class TestFundamentalMatrix(unittest.TestCase):

    def setUp(self):
        # Create two rotations and corresponding fundamental matrix F
        self.trueU = Rot3.Yaw(np.pi / 2)
        self.trueV = Rot3.Yaw(np.pi / 4)
        self.trueS = 0.5
        self.trueF = FundamentalMatrix(self.trueU.matrix(), self.trueS, self.trueV.matrix())

    def test_localCoordinates(self):
        expected = np.zeros(7)  # Assuming 7 dimensions for U, V, and s
        actual = self.trueF.localCoordinates(self.trueF)
        assert_almost_equal(expected, actual, decimal=8)

    def test_retract(self):
        actual = self.trueF.retract(np.zeros(7))
        self.assertTrue(self.trueF.equals(actual, 1e-9))

    def test_RoundTrip(self):
        d = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        hx = self.trueF.retract(d)
        actual = self.trueF.localCoordinates(hx)
        assert_almost_equal(d, actual, decimal=8)


class TestSimpleStereo(unittest.TestCase):

    def setUp(self):
        # Create the simplest SimpleFundamentalMatrix, a stereo pair
        self.defaultE = EssentialMatrix(Rot3(), Unit3(1, 0, 0))
        self.zero = Point2(0.0, 0.0)
        self.stereoF = SimpleFundamentalMatrix(self.defaultE, 1.0, 1.0, self.zero, self.zero)

    def test_Conversion(self):
        convertedF = FundamentalMatrix(self.stereoF.matrix())
        assert_almost_equal(self.stereoF.matrix(), convertedF.matrix(), decimal=8)

    def test_localCoordinates(self):
        expected = np.zeros(7)
        actual = self.stereoF.localCoordinates(self.stereoF)
        assert_almost_equal(expected, actual, decimal=8)

    def test_retract(self):
        actual = self.stereoF.retract(np.zeros(9))
        self.assertTrue(self.stereoF.equals(actual, 1e-9))

    def test_RoundTrip(self):
        d = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        hx = self.stereoF.retract(d)
        actual = self.stereoF.localCoordinates(hx)
        assert_almost_equal(d, actual, decimal=8)

    def test_EpipolarLine(self):
        # Create a point in b
        p_b = np.array([0, 2, 1])
        # Convert the point to a horizontal line in a
        l_a = self.stereoF.matrix() @ p_b
        # Check if the line is horizontal at height 2
        expected = np.array([0, -1, 2])
        assert_almost_equal(l_a, expected, decimal=8)


class TestPixelStereo(unittest.TestCase):

    def setUp(self):
        # Create a stereo pair in pixels, zero principal points
        self.focalLength = 1000.0
        self.defaultE = EssentialMatrix(Rot3(), Unit3(1, 0, 0))
        self.zero = Point2(0.0, 0.0)
        self.pixelStereo = SimpleFundamentalMatrix(
            self.defaultE, self.focalLength, self.focalLength, self.zero, self.zero
        )

    def test_Conversion(self):
        expected = self.pixelStereo.matrix()
        convertedF = FundamentalMatrix(self.pixelStereo.matrix())
        # Check equality of F-matrices up to a scale
        actual = convertedF.matrix()
        scale = expected[1, 2] / actual[1, 2]
        actual *= scale
        assert_almost_equal(expected, actual, decimal=5)

    def test_PointInBToHorizontalLineInA(self):
        # Create a point in b
        p_b = np.array([0, 300, 1])
        # Convert the point to a horizontal line in a
        l_a = self.pixelStereo.matrix() @ p_b
        # Check if the line is horizontal at height 0.3
        expected = np.array([0, -0.001, 0.3])
        assert_almost_equal(l_a, expected, decimal=8)


class TestRotatedPixelStereo(unittest.TestCase):

    def setUp(self):
        # Create a stereo pair with the right camera rotated 90 degrees
        self.focalLength = 1000.0
        self.zero = Point2(0.0, 0.0)
        self.aRb = Rot3.Rz(np.pi / 2)  # Rotate 90 degrees around the Z-axis
        self.rotatedE = EssentialMatrix(self.aRb, Unit3(1, 0, 0))
        self.rotatedPixelStereo = SimpleFundamentalMatrix(
            self.rotatedE, self.focalLength, self.focalLength, self.zero, self.zero
        )

    def test_Conversion(self):
        expected = self.rotatedPixelStereo.matrix()
        convertedF = FundamentalMatrix(self.rotatedPixelStereo.matrix())
        # Check equality of F-matrices up to a scale
        actual = convertedF.matrix()
        scale = expected[1, 2] / actual[1, 2]
        actual *= scale
        assert_almost_equal(expected, actual, decimal=4)

    def test_PointInBToHorizontalLineInA(self):
        # Create a point in b
        p_b = np.array([300, 0, 1])
        # Convert the point to a horizontal line in a
        l_a = self.rotatedPixelStereo.matrix() @ p_b
        # Check if the line is horizontal at height 0.3
        expected = np.array([0, -0.001, 0.3])
        assert_almost_equal(l_a, expected, decimal=8)


class TestStereoWithPrincipalPoints(unittest.TestCase):

    def setUp(self):
        # Now check that principal points also survive conversion
        self.focalLength = 1000.0
        self.principalPoint = Point2(640 / 2, 480 / 2)
        self.aRb = Rot3.Rz(np.pi / 2)
        self.rotatedE = EssentialMatrix(self.aRb, Unit3(1, 0, 0))
        self.stereoWithPrincipalPoints = SimpleFundamentalMatrix(
            self.rotatedE, self.focalLength, self.focalLength, self.principalPoint, self.principalPoint
        )

    def test_Conversion(self):
        expected = self.stereoWithPrincipalPoints.matrix()
        convertedF = FundamentalMatrix(self.stereoWithPrincipalPoints.matrix())
        # Check equality of F-matrices up to a scale
        actual = convertedF.matrix()
        scale = expected[1, 2] / actual[1, 2]
        actual *= scale
        assert_almost_equal(expected, actual, decimal=4)


class TestTripleF(unittest.TestCase):

    def setUp(self):
        # Generate three cameras on a circle, looking in
        self.cameraPoses = SFMdata.posesOnCircle(3, 1.0)
        self.focalLength = 1000.0
        self.principalPoint = Point2(640 / 2, 480 / 2)
        self.triplet = self.generateTripleF(self.cameraPoses)

    def generateTripleF(self, cameraPoses):
        F = []
        for i in range(3):
            j = (i + 1) % 3
            iPj = cameraPoses[i].between(cameraPoses[j])
            E = EssentialMatrix(iPj.rotation(), Unit3(iPj.translation()))
            F_ij = SimpleFundamentalMatrix(
                E, self.focalLength, self.focalLength, self.principalPoint, self.principalPoint
            )
            F.append(F_ij)
        return {"Fab": F[0], "Fbc": F[1], "Fca": F[2]}

    def transferToA(self, pb, pc):
        return gtsam.EpipolarTransfer(self.triplet["Fab"].matrix(), pb, self.triplet["Fca"].matrix().transpose(), pc)

    def transferToB(self, pa, pc):
        return gtsam.EpipolarTransfer(self.triplet["Fab"].matrix().transpose(), pa, self.triplet["Fbc"].matrix(), pc)

    def transferToC(self, pa, pb):
        return gtsam.EpipolarTransfer(self.triplet["Fca"].matrix(), pa, self.triplet["Fbc"].matrix().transpose(), pb)

    def test_Transfer(self):
        triplet = self.triplet
        # Check that they are all equal
        self.assertTrue(triplet["Fab"].equals(triplet["Fbc"], 1e-9))
        self.assertTrue(triplet["Fbc"].equals(triplet["Fca"], 1e-9))
        self.assertTrue(triplet["Fca"].equals(triplet["Fab"], 1e-9))

        # Now project a point into the three cameras
        P = Point3(0.1, 0.2, 0.3)
        K = Cal3_S2(self.focalLength, self.focalLength, 0.0, self.principalPoint[0], self.principalPoint[1])

        p = []
        for i in range(3):
            # Project the point into each camera
            camera = PinholeCameraCal3_S2(self.cameraPoses[i], K)
            p_i = camera.project(P)
            p.append(p_i)

        # Check that transfer works
        assert_almost_equal(p[0], self.transferToA(p[1], p[2]), decimal=9)
        assert_almost_equal(p[1], self.transferToB(p[0], p[2]), decimal=9)
        assert_almost_equal(p[2], self.transferToC(p[0], p[1]), decimal=9)


class TestManyCamerasCircle(unittest.TestCase):
    N = 6

    def setUp(self):
        # Generate six cameras on a circle, looking in
        self.cameraPoses = SFMdata.posesOnCircle(self.N, 1.0)
        self.focalLength = 1000.0
        self.principalPoint = Point2(640 / 2, 480 / 2)
        self.manyFs = self.generateManyFs(self.cameraPoses)

    def generateManyFs(self, cameraPoses):
        F = []
        for i in range(self.N):
            j = (i + 1) % self.N
            iPj = cameraPoses[i].between(cameraPoses[j])
            E = EssentialMatrix(iPj.rotation(), Unit3(iPj.translation()))
            F_ij = SimpleFundamentalMatrix(
                E, self.focalLength, self.focalLength, self.principalPoint, self.principalPoint
            )
            F.append(F_ij)
        return F

    def test_Conversion(self):
        for i in range(self.N):
            expected = self.manyFs[i].matrix()
            convertedF = FundamentalMatrix(self.manyFs[i].matrix())
            # Check equality of F-matrices up to a scale
            actual = convertedF.matrix()
            scale = expected[1, 2] / actual[1, 2]
            actual *= scale
            # print(f"\n{np.round(expected, 3)}", f"\n{np.round(actual, 3)}")
            assert_almost_equal(expected, actual, decimal=4)

    def test_Transfer(self):
        # Now project a point into the six cameras
        P = Point3(0.1, 0.2, 0.3)
        K = Cal3_S2(self.focalLength, self.focalLength, 0.0, self.principalPoint[0], self.principalPoint[1])

        p = []
        for i in range(self.N):
            # Project the point into each camera
            camera = PinholeCameraCal3_S2(self.cameraPoses[i], K)
            p_i = camera.project(P)
            p.append(p_i)

        # Check that transfer works
        for a in range(self.N):
            b = (a + 1) % self.N
            c = (a + 2) % self.N
            # We transfer from a to b and from c to b,
            # and check that the result lines up with the projected point in b.
            transferred = gtsam.EpipolarTransfer(
                self.manyFs[a].matrix().transpose(),  # need to transpose for a->b
                p[a],
                self.manyFs[c].matrix(),
                p[c],
            )
            assert_almost_equal(p[b], transferred, decimal=9)


if __name__ == "__main__":
    unittest.main()
