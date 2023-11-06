"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests to ensure backwards compatibility of the Python wrapper.
Author: Varun Agrawal
"""
import unittest
from typing import Iterable, List, Optional, Tuple, Union

import numpy as np
from gtsam.gtsfm import Keypoints
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (BetweenFactorPose2, Cal3_S2, Cal3Bundler, CameraSetCal3_S2,
                   CameraSetCal3Bundler, IndexPair, LevenbergMarquardtParams,
                   PinholeCameraCal3_S2, PinholeCameraCal3Bundler, Point2,
                   Point2Pairs, Point3, Pose2, Pose2Pairs, Pose3, Rot2, Rot3,
                   SfmTrack2d, ShonanAveraging2, ShonanAveragingParameters2,
                   Similarity2, Similarity3, TriangulationParameters,
                   TriangulationResult)

UPRIGHT = Rot3.Ypr(-np.pi / 2, 0.0, -np.pi / 2)


class TestBackwardsCompatibility(GtsamTestCase):
    """Tests for backwards compatibility of the Python wrapper."""

    def setUp(self):
        """Setup test fixtures"""
        p1 = [-1.0, 0.0, -1.0]
        p2 = [1.0, 0.0, -1.0]
        q1 = Rot3(1.0, 0.0, 0.0, 0.0)
        q2 = Rot3(1.0, 0.0, 0.0, 0.0)
        pose1 = Pose3(q1, p1)
        pose2 = Pose3(q2, p2)
        camera1 = gtsam.PinholeCameraCal3Fisheye(pose1)
        camera2 = gtsam.PinholeCameraCal3Fisheye(pose2)
        self.origin = np.array([0.0, 0.0, 0.0])
        self.poses = gtsam.Pose3Vector([pose1, pose2])

        self.fisheye_cameras = gtsam.CameraSetCal3Fisheye([camera1, camera2])
        self.fisheye_measurements = gtsam.Point2Vector(
            [k.project(self.origin) for k in self.fisheye_cameras])

        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        self.stereographic = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1,
                                               p2, xi)
        camera1 = gtsam.PinholeCameraCal3Unified(pose1, self.stereographic)
        camera2 = gtsam.PinholeCameraCal3Unified(pose2, self.stereographic)
        self.unified_cameras = gtsam.CameraSetCal3Unified([camera1, camera2])
        self.unified_measurements = gtsam.Point2Vector(
            [k.project(self.origin) for k in self.unified_cameras])

        ## Set up two camera poses
        # Looking along X-axis, 1 meter above ground plane (x-y)
        pose1 = Pose3(UPRIGHT, Point3(0, 0, 1))

        # create second camera 1 meter to the right of first camera
        pose2 = pose1.compose(Pose3(Rot3(), Point3(1, 0, 0)))
        # twoPoses
        self.triangulation_poses = gtsam.Pose3Vector()
        self.triangulation_poses.append(pose1)
        self.triangulation_poses.append(pose2)

        # landmark ~5 meters infront of camera
        self.landmark = Point3(5, 0.5, 1.2)

    def test_Cal3Fisheye_triangulation_rectify(self):
        """
        Estimate spatial point from image measurements using
        rectification from a Cal3Fisheye camera model.
        """
        rectified = gtsam.Point2Vector([
            k.calibration().calibrate(pt)
            for k, pt in zip(self.fisheye_cameras, self.fisheye_measurements)
        ])
        shared_cal = gtsam.Cal3_S2()
        triangulated = gtsam.triangulatePoint3(self.poses,
                                               shared_cal,
                                               rectified,
                                               rank_tol=1e-9,
                                               optimize=False)
        self.gtsamAssertEquals(triangulated, self.origin)

    def test_Cal3Unified_triangulation_rectify(self):
        """
        Estimate spatial point from image measurements using
        rectification from a Cal3Unified camera model.
        """
        rectified = gtsam.Point2Vector([
            k.calibration().calibrate(pt)
            for k, pt in zip(self.unified_cameras, self.unified_measurements)
        ])
        shared_cal = gtsam.Cal3_S2()
        triangulated = gtsam.triangulatePoint3(self.poses,
                                               shared_cal,
                                               rectified,
                                               rank_tol=1e-9,
                                               optimize=False)
        self.gtsamAssertEquals(triangulated, self.origin)

    def test_track_generation(self) -> None:
        """Ensures that DSF generates three tracks from measurements
        in 3 images (H=200,W=400)."""
        kps_i0 = Keypoints(np.array([[10.0, 20], [30, 40]]))
        kps_i1 = Keypoints(np.array([[50.0, 60], [70, 80], [90, 100]]))
        kps_i2 = Keypoints(np.array([[110.0, 120], [130, 140]]))

        keypoints_list = gtsam.KeypointsVector()
        keypoints_list.append(kps_i0)
        keypoints_list.append(kps_i1)
        keypoints_list.append(kps_i2)

        # For each image pair (i1,i2), we provide a (K,2) matrix
        # of corresponding image indices (k1,k2).
        matches_dict = gtsam.MatchIndicesMap()
        matches_dict[IndexPair(0, 1)] = np.array([[0, 0], [1, 1]])
        matches_dict[IndexPair(1, 2)] = np.array([[2, 0], [1, 1]])

        tracks = gtsam.gtsfm.tracksFromPairwiseMatches(
            matches_dict,
            keypoints_list,
            verbose=False,
        )
        assert len(tracks) == 3

        # Verify track 0.
        track0 = tracks[0]
        assert track0.numberMeasurements() == 2
        np.testing.assert_allclose(track0.measurements[0][1], Point2(10, 20))
        np.testing.assert_allclose(track0.measurements[1][1], Point2(50, 60))
        assert track0.measurements[0][0] == 0
        assert track0.measurements[1][0] == 1
        np.testing.assert_allclose(
            track0.measurementMatrix(),
            [
                [10, 20],
                [50, 60],
            ],
        )
        np.testing.assert_allclose(track0.indexVector(), [0, 1])

        # Verify track 1.
        track1 = tracks[1]
        np.testing.assert_allclose(
            track1.measurementMatrix(),
            [
                [30, 40],
                [70, 80],
                [130, 140],
            ],
        )
        np.testing.assert_allclose(track1.indexVector(), [0, 1, 2])

        # Verify track 2.
        track2 = tracks[2]
        np.testing.assert_allclose(
            track2.measurementMatrix(),
            [
                [90, 100],
                [110, 120],
            ],
        )
        np.testing.assert_allclose(track2.indexVector(), [1, 2])

    def test_sfm_track_2d_constructor(self) -> None:
        """Test construction of 2D SfM track."""
        measurements = gtsam.SfmMeasurementVector()
        measurements.append((0, Point2(10, 20)))
        track = SfmTrack2d(measurements=measurements)
        track.measurement(0)
        assert track.numberMeasurements() == 1

    def test_FixedLagSmootherExample(self):
        '''
        Simple test that checks for equality between C++ example
        file and the Python implementation. See
        gtsam_unstable/examples/FixedLagSmootherExample.cpp
        '''
        # Define a batch fixed lag smoother, which uses
        # Levenberg-Marquardt to perform the nonlinear optimization
        lag = 2.0
        smoother_batch = gtsam.BatchFixedLagSmoother(lag)

        # Create containers to store the factors and linearization points
        # that will be sent to the smoothers
        new_factors = gtsam.NonlinearFactorGraph()
        new_values = gtsam.Values()
        new_timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

        # Create  a prior on the first pose, placing it at the origin
        prior_mean = Pose2(0, 0, 0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))
        X1 = 0
        new_factors.push_back(
            gtsam.PriorFactorPose2(X1, prior_mean, prior_noise))
        new_values.insert(X1, prior_mean)
        new_timestamps.insert((X1, 0.0))

        delta_time = 0.25
        time = 0.25

        i = 0

        ground_truth = [
            Pose2(0.995821, 0.0231012, 0.0300001),
            Pose2(1.49284, 0.0457247, 0.045),
            Pose2(1.98981, 0.0758879, 0.06),
            Pose2(2.48627, 0.113502, 0.075),
            Pose2(2.98211, 0.158558, 0.09),
            Pose2(3.47722, 0.211047, 0.105),
            Pose2(3.97149, 0.270956, 0.12),
            Pose2(4.4648, 0.338272, 0.135),
            Pose2(4.95705, 0.41298, 0.15),
            Pose2(5.44812, 0.495063, 0.165),
            Pose2(5.9379, 0.584503, 0.18),
        ]

        # Iterates from 0.25s to 3.0s, adding 0.25s each loop
        # In each iteration, the agent moves at a constant speed
        # and its two odometers measure the change. The smoothed
        # result is then compared to the ground truth
        while time <= 3.0:
            previous_key = int(1000 * (time - delta_time))
            current_key = int(1000 * time)

            # assign current key to the current timestamp
            new_timestamps.insert((current_key, time))

            # Add a guess for this pose to the new values
            # Assume that the robot moves at 2 m/s. Position is time[s] *
            # 2[m/s]
            current_pose = Pose2(time * 2, 0, 0)
            new_values.insert(current_key, current_pose)

            # Add odometry factors from two different sources with different
            # error stats
            odometry_measurement_1 = Pose2(0.61, -0.08, 0.02)
            odometry_noise_1 = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.1, 0.1, 0.05]))
            new_factors.push_back(
                gtsam.BetweenFactorPose2(previous_key, current_key,
                                         odometry_measurement_1,
                                         odometry_noise_1))

            odometry_measurement_2 = Pose2(0.47, 0.03, 0.01)
            odometry_noise_2 = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.05, 0.05, 0.05]))
            new_factors.push_back(
                gtsam.BetweenFactorPose2(previous_key, current_key,
                                         odometry_measurement_2,
                                         odometry_noise_2))

            # Update the smoothers with the new factors. In this case,
            # one iteration must pass for Levenberg-Marquardt to accurately
            # estimate
            if time >= 0.50:
                smoother_batch.update(new_factors, new_values, new_timestamps)

                estimate = smoother_batch.calculateEstimatePose2(current_key)
                self.assertTrue(estimate.equals(ground_truth[i], 1e-4))
                i += 1

                new_timestamps.clear()
                new_values.clear()
                new_factors.resize(0)

            time += delta_time

    def test_ordering(self):
        """Test ordering"""
        gfg = gtsam.GaussianFactorGraph()

        x0 = X(0)
        x1 = X(1)
        x2 = X(2)

        BETWEEN_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.ones(1))
        PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.ones(1))

        gfg.add(x1, np.eye(1), x0, -np.eye(1), np.ones(1), BETWEEN_NOISE)
        gfg.add(x2, np.eye(1), x1, -np.eye(1), 2 * np.ones(1), BETWEEN_NOISE)
        gfg.add(x0, np.eye(1), np.zeros(1), PRIOR_NOISE)

        keys = (x0, x1, x2)
        ordering = gtsam.Ordering()
        for key in keys[::-1]:
            ordering.push_back(key)

        bn = gfg.eliminateSequential(ordering)
        self.assertEqual(bn.size(), 3)

        keyVector = gtsam.KeyVector()
        keyVector.append(keys[2])
        ordering = gtsam.Ordering.ColamdConstrainedLastGaussianFactorGraph(
            gfg, keyVector)
        bn = gfg.eliminateSequential(ordering)
        self.assertEqual(bn.size(), 3)

    def test_find(self):
        """
        Check that optimizing for Karcher mean (which minimizes Between distance)
        gets correct result.
        """
        R = Rot3.Expmap(np.array([0.1, 0, 0]))

        rotations = gtsam.Rot3Vector([R, R.inverse()])
        expected = Rot3()
        actual = gtsam.FindKarcherMean(rotations)
        self.gtsamAssertEquals(expected, actual)

    def test_find_karcher_mean_identity(self):
        """Averaging 3 identity rotations should yield the identity."""
        a1Rb1 = Rot3()
        a2Rb2 = Rot3()
        a3Rb3 = Rot3()

        aRb_list = gtsam.Rot3Vector([a1Rb1, a2Rb2, a3Rb3])
        aRb_expected = Rot3()

        aRb = gtsam.FindKarcherMean(aRb_list)
        self.gtsamAssertEquals(aRb, aRb_expected)

    def test_factor(self):
        """Check that the InnerConstraint factor leaves the mean unchanged."""
        # Make a graph with two variables, one between, and one InnerConstraint
        # The optimal result should satisfy the between, while moving the other
        # variable to make the mean the same as before.
        # Mean of R and R' is identity. Let's make a BetweenFactor making R21 =
        # R*R*R, i.e. geodesic length is 3 rather than 2.
        R = Rot3.Expmap(np.array([0.1, 0, 0]))
        MODEL = gtsam.noiseModel.Unit.Create(3)

        graph = gtsam.NonlinearFactorGraph()
        R12 = R.compose(R.compose(R))
        graph.add(gtsam.BetweenFactorRot3(1, 2, R12, MODEL))
        keys = gtsam.KeyVector()
        keys.append(1)
        keys.append(2)
        graph.add(gtsam.KarcherMeanFactorRot3(keys))

        initial = gtsam.Values()
        initial.insert(1, R.inverse())
        initial.insert(2, R)
        expected = Rot3()

        result = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
        actual = gtsam.FindKarcherMean(
            gtsam.Rot3Vector([result.atRot3(1),
                              result.atRot3(2)]))
        self.gtsamAssertEquals(expected, actual)
        self.gtsamAssertEquals(R12, result.atRot3(1).between(result.atRot3(2)))

    def test_align(self) -> None:
        """Ensure estimation of the Pose2 element to align two 2d point clouds succeeds.

        Two point clouds represent horseshoe-shapes of the same size, just rotated and translated:

                |  X---X
                |  |
                |  X---X
        ------------------
                |
                |
              O | O
              | | |
              O---O
        """
        pts_a = [
            Point2(1, -3),
            Point2(1, -5),
            Point2(-1, -5),
            Point2(-1, -3),
        ]
        pts_b = [
            Point2(3, 1),
            Point2(1, 1),
            Point2(1, 3),
            Point2(3, 3),
        ]

        ab_pairs = Point2Pairs(list(zip(pts_a, pts_b)))
        aTb = Pose2.Align(ab_pairs)
        self.assertIsNotNone(aTb)

        for pt_a, pt_b in zip(pts_a, pts_b):
            pt_a_ = aTb.transformFrom(pt_b)
            np.testing.assert_allclose(pt_a, pt_a_)

        # Matrix version
        A = np.array(pts_a).T
        B = np.array(pts_b).T
        aTb = Pose2.Align(A, B)
        self.assertIsNotNone(aTb)

        for pt_a, pt_b in zip(pts_a, pts_b):
            pt_a_ = aTb.transformFrom(pt_b)
            np.testing.assert_allclose(pt_a, pt_a_)

    def test_align_squares(self):
        """Test if Align method can align 2 squares."""
        square = np.array([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0]],
                          float).T
        sTt = Pose3(Rot3.Rodrigues(0, 0, -np.pi), gtsam.Point3(2, 4, 0))
        transformed = sTt.transformTo(square)

        st_pairs = gtsam.Point3Pairs()
        for j in range(4):
            st_pairs.append((square[:, j], transformed[:, j]))

        # Recover the transformation sTt
        estimated_sTt = Pose3.Align(st_pairs)
        self.gtsamAssertEquals(estimated_sTt, sTt, 1e-10)

        # Matrix version
        estimated_sTt = Pose3.Align(square, transformed)
        self.gtsamAssertEquals(estimated_sTt, sTt, 1e-10)

    def test_constructorBetweenFactorPose2s(self) -> None:
        """Check if ShonanAveraging2 constructor works when not initialized from g2o file.

        GT pose graph:

           | cam 1 = (0,4)
         --o
           | .
           .   .
           .     .
           |       |
           o-- ... o--
        cam 0       cam 2 = (4,0)
          (0,0)
        """
        num_images = 3

        wTi_list = [
            Pose2(Rot2.fromDegrees(0), np.array([0, 0])),
            Pose2(Rot2.fromDegrees(90), np.array([0, 4])),
            Pose2(Rot2.fromDegrees(0), np.array([4, 0])),
        ]

        edges = [(0, 1), (1, 2), (0, 2)]
        i2Ri1_dict = {(i1, i2):
                      wTi_list[i2].inverse().compose(wTi_list[i1]).rotation()
                      for (i1, i2) in edges}

        lm_params = LevenbergMarquardtParams.CeresDefaults()
        shonan_params = ShonanAveragingParameters2(lm_params)
        shonan_params.setUseHuber(False)
        shonan_params.setCertifyOptimality(True)

        noise_model = gtsam.noiseModel.Unit.Create(3)
        between_factors = gtsam.BetweenFactorPose2s()
        for (i1, i2), i2Ri1 in i2Ri1_dict.items():
            i2Ti1 = Pose2(i2Ri1, np.zeros(2))
            between_factors.append(
                BetweenFactorPose2(i2, i1, i2Ti1, noise_model))

        obj = ShonanAveraging2(between_factors, shonan_params)
        initial = obj.initializeRandomly()
        result_values, _ = obj.run(initial, min_p=2, max_p=100)

        wRi_list = [result_values.atRot2(i) for i in range(num_images)]
        thetas_deg = np.array([wRi.degrees() for wRi in wRi_list])

        # map all angles to [0,360)
        thetas_deg = thetas_deg % 360
        thetas_deg -= thetas_deg[0]

        expected_thetas_deg = np.array([0.0, 90.0, 0.0])
        np.testing.assert_allclose(thetas_deg, expected_thetas_deg, atol=0.1)

    def test_align_poses2_along_straight_line(self) -> None:
        """Test Align of list of Pose2Pair.

        Scenario:
           3 object poses
           same scale (no gauge ambiguity)
           world frame has poses rotated about 180 degrees.
           world and egovehicle frame translated by 15 meters w.r.t. each other
        """
        R180 = Rot2.fromDegrees(180)

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose2(Rot2(), np.array([5, 0]))
        eTo1 = Pose2(Rot2(), np.array([10, 0]))
        eTo2 = Pose2(Rot2(), np.array([15, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world "w" frame)
        wTo0 = Pose2(R180, np.array([-10, 0]))
        wTo1 = Pose2(R180, np.array([-5, 0]))
        wTo2 = Pose2(R180, np.array([0, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = Pose2Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity2.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses2_along_straight_line_gauge(self):
        """Test if Align Pose2Pairs method can account for gauge ambiguity.

        Scenario:
           3 object poses
           with gauge ambiguity (2x scale)
           world frame has poses rotated by 90 degrees.
           world and egovehicle frame translated by 11 meters w.r.t. each other
        """
        R90 = Rot2.fromDegrees(90)

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose2(Rot2(), np.array([1, 0]))
        eTo1 = Pose2(Rot2(), np.array([2, 0]))
        eTo2 = Pose2(Rot2(), np.array([4, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose2(R90, np.array([0, 12]))
        wTo1 = Pose2(R90, np.array([0, 14]))
        wTo2 = Pose2(R90, np.array([0, 18]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = Pose2Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity2.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses2_scaled_squares(self):
        """Test if Align Pose2Pairs method can account for gauge ambiguity.

        Make sure a big and small square can be aligned.
        The u's represent a big square (10x10), and v's represents a small square (4x4).

        Scenario:
           4 object poses
           with gauge ambiguity (2.5x scale)
        """
        # 0, 90, 180, and 270 degrees yaw
        R0 = Rot2.fromDegrees(0)
        R90 = Rot2.fromDegrees(90)
        R180 = Rot2.fromDegrees(180)
        R270 = Rot2.fromDegrees(270)

        aTi0 = Pose2(R0, np.array([2, 3]))
        aTi1 = Pose2(R90, np.array([12, 3]))
        aTi2 = Pose2(R180, np.array([12, 13]))
        aTi3 = Pose2(R270, np.array([2, 13]))

        aTi_list = [aTi0, aTi1, aTi2, aTi3]

        bTi0 = Pose2(R0, np.array([4, 3]))
        bTi1 = Pose2(R90, np.array([8, 3]))
        bTi2 = Pose2(R180, np.array([8, 7]))
        bTi3 = Pose2(R270, np.array([4, 7]))

        bTi_list = [bTi0, bTi1, bTi2, bTi3]

        ab_pairs = Pose2Pairs(list(zip(aTi_list, bTi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        aSb = Similarity2.Align(ab_pairs)

        for aTi, bTi in zip(aTi_list, bTi_list):
            self.gtsamAssertEquals(aTi, aSb.transformFrom(bTi))

    def test_align_poses3_along_straight_line(self):
        """Test Align Pose3Pairs method.

        Scenario:
           3 object poses
           same scale (no gauge ambiguity)
           world frame has poses rotated about x-axis (90 degree roll)
           world and egovehicle frame translated by 15 meters w.r.t. each other
        """
        Rx90 = Rot3.Rx(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([5, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([10, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([15, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rx90, np.array([-10, 0, 0]))
        wTo1 = Pose3(Rx90, np.array([-5, 0, 0]))
        wTo2 = Pose3(Rx90, np.array([0, 0, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = gtsam.Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses3_along_straight_line_gauge(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Scenario:
           3 object poses
           with gauge ambiguity (2x scale)
           world frame has poses rotated about z-axis (90 degree yaw)
           world and egovehicle frame translated by 11 meters w.r.t. each other
        """
        Rz90 = Rot3.Rz(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([1, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([2, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([4, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rz90, np.array([0, 12, 0]))
        wTo1 = Pose3(Rz90, np.array([0, 14, 0]))
        wTo2 = Pose3(Rz90, np.array([0, 18, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = gtsam.Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses3_scaled_squares(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Make sure a big and small square can be aligned.
        The u's represent a big square (10x10), and v's represents a small square (4x4).

        Scenario:
           4 object poses
           with gauge ambiguity (2.5x scale)
        """
        # 0, 90, 180, and 270 degrees yaw
        R0 = Rot3.Rz(np.deg2rad(0))
        R90 = Rot3.Rz(np.deg2rad(90))
        R180 = Rot3.Rz(np.deg2rad(180))
        R270 = Rot3.Rz(np.deg2rad(270))

        aTi0 = Pose3(R0, np.array([2, 3, 0]))
        aTi1 = Pose3(R90, np.array([12, 3, 0]))
        aTi2 = Pose3(R180, np.array([12, 13, 0]))
        aTi3 = Pose3(R270, np.array([2, 13, 0]))

        aTi_list = [aTi0, aTi1, aTi2, aTi3]

        bTi0 = Pose3(R0, np.array([4, 3, 0]))
        bTi1 = Pose3(R90, np.array([8, 3, 0]))
        bTi2 = Pose3(R180, np.array([8, 7, 0]))
        bTi3 = Pose3(R270, np.array([4, 7, 0]))

        bTi_list = [bTi0, bTi1, bTi2, bTi3]

        ab_pairs = gtsam.Pose3Pairs(list(zip(aTi_list, bTi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        aSb = Similarity3.Align(ab_pairs)

        for aTi, bTi in zip(aTi_list, bTi_list):
            self.gtsamAssertEquals(aTi, aSb.transformFrom(bTi))

    def generate_measurements(
        self,
        calibration: Union[Cal3Bundler, Cal3_S2],
        camera_model: Union[PinholeCameraCal3Bundler, PinholeCameraCal3_S2],
        cal_params: Iterable[Iterable[Union[int, float]]],
        camera_set: Optional[Union[CameraSetCal3Bundler,
                                   CameraSetCal3_S2]] = None,
    ) -> Tuple[List[Point2], Union[CameraSetCal3Bundler, CameraSetCal3_S2,
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
        measurements = gtsam.Point2Vector()

        for k, pose in zip(cal_params, self.triangulation_poses):
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

        triangulated_landmark = gtsam.triangulatePoint3(
            self.triangulation_poses,
            Cal3_S2(sharedCal),
            measurements,
            rank_tol=1e-9,
            optimize=True)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9)

        # Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
        measurements_noisy = gtsam.Point2Vector()
        measurements_noisy.append(measurements[0] - np.array([0.1, 0.5]))
        measurements_noisy.append(measurements[1] - np.array([-0.2, 0.3]))

        triangulated_landmark = gtsam.triangulatePoint3(
            self.triangulation_poses,
            Cal3_S2(sharedCal),
            measurements_noisy,
            rank_tol=1e-9,
            optimize=True)

        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-2)

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
        measurements = gtsam.Point2Vector([z1, z2, z3])

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

    def test_outliers_and_far_landmarks(self) -> None:
        """Check safe triangulation function."""
        pose1, pose2 = self.poses

        K1 = Cal3_S2(1500, 1200, 0, 640, 480)
        # create first camera. Looking along X-axis, 1 meter above ground plane (x-y)
        camera1 = PinholeCameraCal3_S2(pose1, K1)

        # create second camera 1 meter to the right of first camera
        K2 = Cal3_S2(1600, 1300, 0, 650, 440)
        camera2 = PinholeCameraCal3_S2(pose2, K2)

        # 1. Project two landmarks into two cameras and triangulate
        z1 = camera1.project(self.landmark)
        z2 = camera2.project(self.landmark)

        cameras = CameraSetCal3_S2()
        cameras.append(camera1)
        cameras.append(camera2)

        measurements = gtsam.Point2Vector()
        measurements.append(z1)
        measurements.append(z2)

        landmarkDistanceThreshold = 10  # landmark is closer than that
        # all default except landmarkDistanceThreshold:
        params = TriangulationParameters(1.0, False, landmarkDistanceThreshold)
        actual: TriangulationResult = gtsam.triangulateSafe(
            cameras, measurements, params)
        self.gtsamAssertEquals(actual.get(), self.landmark, 1e-2)
        self.assertTrue(actual.valid())

        landmarkDistanceThreshold = 4  # landmark is farther than that
        params2 = TriangulationParameters(1.0, False,
                                          landmarkDistanceThreshold)
        actual = gtsam.triangulateSafe(cameras, measurements, params2)
        self.assertTrue(actual.farPoint())

        # 3. Add a slightly rotated third camera above with a wrong measurement
        # (OUTLIER)
        pose3 = pose1 * Pose3(Rot3.Ypr(0.1, 0.2, 0.1), Point3(0.1, -2, -.1))
        K3 = Cal3_S2(700, 500, 0, 640, 480)
        camera3 = PinholeCameraCal3_S2(pose3, K3)
        z3 = camera3.project(self.landmark)

        cameras.append(camera3)
        measurements.append(z3 + Point2(10, -10))

        landmarkDistanceThreshold = 10  # landmark is closer than that
        outlierThreshold = 100  # loose, the outlier is going to pass
        params3 = TriangulationParameters(1.0, False,
                                          landmarkDistanceThreshold,
                                          outlierThreshold)
        actual = gtsam.triangulateSafe(cameras, measurements, params3)
        self.assertTrue(actual.valid())

        # now set stricter threshold for outlier rejection
        outlierThreshold = 5  # tighter, the outlier is not going to pass
        params4 = TriangulationParameters(1.0, False,
                                          landmarkDistanceThreshold,
                                          outlierThreshold)
        actual = gtsam.triangulateSafe(cameras, measurements, params4)
        self.assertTrue(actual.outlier())


if __name__ == "__main__":
    unittest.main()
