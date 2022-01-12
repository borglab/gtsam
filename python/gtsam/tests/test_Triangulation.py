"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Test Triangulation
Authors: Frank Dellaert & Fan Jiang (Python) & Sushmita Warrier & John Lambert
"""
import unittest
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np

import gtsam
from gtsam import (
    Cal3_S2,
    Cal3Bundler,
    CameraSetCal3_S2,
    CameraSetCal3Bundler,
    PinholeCameraCal3_S2,
    PinholeCameraCal3Bundler,
    Point2,
    Point2Vector,
    Point3,
    Pose3,
    Pose3Vector,
    Rot3,
)
from gtsam.utils.test_case import GtsamTestCase

UPRIGHT = Rot3.Ypr(-np.pi / 2, 0.0, -np.pi / 2)


class TestTriangulationExample(GtsamTestCase):
    """Tests for triangulation with shared and individual calibrations"""

    def setUp(self):
        """Set up two camera poses"""
        # Looking along X-axis, 1 meter above ground plane (x-y)
        pose1 = Pose3(UPRIGHT, Point3(0, 0, 1))

        # create second camera 1 meter to the right of first camera
        pose2 = pose1.compose(Pose3(Rot3(), Point3(1, 0, 0)))
        # twoPoses
        self.poses = Pose3Vector()
        self.poses.append(pose1)
        self.poses.append(pose2)

        # landmark ~5 meters infront of camera
        self.landmark = Point3(5, 0.5, 1.2)

    def generate_measurements(
        self,
        calibration: Union[Cal3Bundler, Cal3_S2],
        camera_model: Union[PinholeCameraCal3Bundler, PinholeCameraCal3_S2],
        cal_params: Iterable[Iterable[Union[int, float]]],
        camera_set: Optional[Union[CameraSetCal3Bundler,
                                   CameraSetCal3_S2]] = None,
    ) -> Tuple[Point2Vector, Union[CameraSetCal3Bundler, CameraSetCal3_S2,
                                   List[Cal3Bundler], List[Cal3_S2]]]:
        """
        Generate vector of measurements for given calibration and camera model.

        Args:
            calibration: Camera calibration e.g. Cal3_S2
            camera_model: Camera model e.g. PinholeCameraCal3_S2
            cal_params: Iterable of camera parameters for `calibration` e.g. [K1, K2]
            camera_set: Cameraset object (for individual calibrations)

        Returns:
            list of measurements and list/CameraSet object for cameras
        """
        if camera_set is not None:
            cameras = camera_set()
        else:
            cameras = []
        measurements = Point2Vector()

        for k, pose in zip(cal_params, self.poses):
            K = calibration(*k)
            camera = camera_model(pose, K)
            cameras.append(camera)
            z = camera.project(self.landmark)
            measurements.append(z)

        return measurements, cameras

    def test_TriangulationExample(self) -> None:
        """Tests triangulation with shared Cal3_S2 calibration"""
        # Some common constants
        sharedCal = (1500, 1200, 0, 640, 480)

        measurements, _ = self.generate_measurements(
            calibration=Cal3_S2,
            camera_model=PinholeCameraCal3_S2,
            cal_params=(sharedCal, sharedCal))

        triangulated_landmark = gtsam.triangulatePoint3(self.poses,
                                                        Cal3_S2(sharedCal),
                                                        measurements,
                                                        rank_tol=1e-9,
                                                        optimize=True)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9)

        # Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
        measurements_noisy = Point2Vector()
        measurements_noisy.append(measurements[0] - np.array([0.1, 0.5]))
        measurements_noisy.append(measurements[1] - np.array([-0.2, 0.3]))

        triangulated_landmark = gtsam.triangulatePoint3(self.poses,
                                                        Cal3_S2(sharedCal),
                                                        measurements_noisy,
                                                        rank_tol=1e-9,
                                                        optimize=True)

        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-2)

    def test_distinct_Ks(self) -> None:
        """Tests triangulation with individual Cal3_S2 calibrations"""
        # two camera parameters
        K1 = (1500, 1200, 0, 640, 480)
        K2 = (1600, 1300, 0, 650, 440)

        measurements, cameras = self.generate_measurements(
            calibration=Cal3_S2,
            camera_model=PinholeCameraCal3_S2,
            cal_params=(K1, K2),
            camera_set=CameraSetCal3_S2)

        triangulated_landmark = gtsam.triangulatePoint3(cameras,
                                                        measurements,
                                                        rank_tol=1e-9,
                                                        optimize=True)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9)

    def test_distinct_Ks_Bundler(self) -> None:
        """Tests triangulation with individual Cal3Bundler calibrations"""
        # two camera parameters
        K1 = (1500, 0, 0, 640, 480)
        K2 = (1600, 0, 0, 650, 440)

        measurements, cameras = self.generate_measurements(
            calibration=Cal3Bundler,
            camera_model=PinholeCameraCal3Bundler,
            cal_params=(K1, K2),
            camera_set=CameraSetCal3Bundler)

        triangulated_landmark = gtsam.triangulatePoint3(cameras,
                                                        measurements,
                                                        rank_tol=1e-9,
                                                        optimize=True)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9)

    def test_triangulation_robust_three_poses(self) -> None:
        """Ensure triangulation with a robust model works."""
        sharedCal = Cal3_S2(1500, 1200, 0, 640, 480)

        # landmark ~5 meters infront of camera
        landmark = Point3(5, 0.5, 1.2)

        pose1 = Pose3(UPRIGHT, Point3(0, 0, 1))
        pose2 = pose1 * Pose3(Rot3(), Point3(1, 0, 0))
        pose3 = pose1 * Pose3(Rot3.Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -0.1))

        camera1 = PinholeCameraCal3_S2(pose1, sharedCal)
        camera2 = PinholeCameraCal3_S2(pose2, sharedCal)
        camera3 = PinholeCameraCal3_S2(pose3, sharedCal)

        z1: Point2 = camera1.project(landmark)
        z2: Point2 = camera2.project(landmark)
        z3: Point2 = camera3.project(landmark)

        poses = gtsam.Pose3Vector([pose1, pose2, pose3])
        measurements = Point2Vector([z1, z2, z3])

        # noise free, so should give exactly the landmark
        actual = gtsam.triangulatePoint3(poses,
                                         sharedCal,
                                         measurements,
                                         rank_tol=1e-9,
                                         optimize=False)
        self.assertTrue(np.allclose(landmark, actual, atol=1e-2))

        # Add outlier
        measurements[0] += Point2(100, 120)  # very large pixel noise!

        # now estimate does not match landmark
        actual2 = gtsam.triangulatePoint3(poses,
                                          sharedCal,
                                          measurements,
                                          rank_tol=1e-9,
                                          optimize=False)
        # DLT is surprisingly robust, but still off (actual error is around 0.26m)
        self.assertTrue(np.linalg.norm(landmark - actual2) >= 0.2)
        self.assertTrue(np.linalg.norm(landmark - actual2) <= 0.5)

        # Again with nonlinear optimization
        actual3 = gtsam.triangulatePoint3(poses,
                                          sharedCal,
                                          measurements,
                                          rank_tol=1e-9,
                                          optimize=True)
        # result from nonlinear (but non-robust optimization) is close to DLT and still off
        self.assertTrue(np.allclose(actual2, actual3, atol=0.1))

        # Again with nonlinear optimization, this time with robust loss
        model = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345),
            gtsam.noiseModel.Unit.Create(2))
        actual4 = gtsam.triangulatePoint3(poses,
                                          sharedCal,
                                          measurements,
                                          rank_tol=1e-9,
                                          optimize=True,
                                          model=model)
        # using the Huber loss we now have a quite small error!! nice!
        self.assertTrue(np.allclose(landmark, actual4, atol=0.05))


if __name__ == "__main__":
    unittest.main()
