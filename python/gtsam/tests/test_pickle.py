"""
GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests to check pickling.

Author: Ayush Baid
"""
from gtsam import Cal3Bundler, PinholeCameraCal3Bundler, Point2, Point3, Pose3, Rot3, SfmTrack, Unit3 

from gtsam.utils.test_case import GtsamTestCase

class TestPickle(GtsamTestCase):
    """Tests pickling on some of the classes."""

    def test_cal3Bundler_roundtrip(self):
        obj = Cal3Bundler(fx=100, k1=0.1, k2=0.2, u0=100, v0=70)
        self.assertEqualityOnPickleRoundtrip(obj)
    
    def test_pinholeCameraCal3Bundler_roundtrip(self):
        obj = PinholeCameraCal3Bundler(
            Pose3(Rot3.RzRyRx(0, 0.1, -0.05), Point3(1, 1, 0)),
            Cal3Bundler(fx=100, k1=0.1, k2=0.2, u0=100, v0=70),
        )
        self.assertEqualityOnPickleRoundtrip(obj)
    
    def test_rot3_roundtrip(self):
        obj = Rot3.RzRyRx(0, 0.05, 0.1)
        self.assertEqualityOnPickleRoundtrip(obj)

    def test_pose3_roundtrip(self):
        obj = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))
        self.assertEqualityOnPickleRoundtrip(obj)

    def test_sfmTrack_roundtrip(self):
        obj = SfmTrack(Point3(1, 1, 0))
        obj.addMeasurement(0, Point2(-1, 5))
        obj.addMeasurement(1, Point2(6, 2))
        self.assertEqualityOnPickleRoundtrip(obj)

    def test_unit3_roundtrip(self):
        obj = Unit3(Point3(1, 1, 0))
        self.assertEqualityOnPickleRoundtrip(obj)
