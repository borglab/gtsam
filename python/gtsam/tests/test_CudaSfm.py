"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

CUDA SFM Python wrapper tests.
"""

import math
import unittest

import gtsam
import numpy as np
from gtsam import (Cal3Bundler, PinholeCameraCal3Bundler, Point3, Pose3,
                   Rot3, SfmData, SfmTrack)
from gtsam.symbol_shorthand import C, P

cuda = getattr(gtsam, "cuda", None)


def _make_zero_error_sfm_data():
    """Create a tiny BAL-like problem whose initial values already fit."""
    data = SfmData()
    cameras = [
        PinholeCameraCal3Bundler(
            Pose3(), Cal3Bundler(100.0, 0.0, 0.0, 0.0, 0.0)),
        PinholeCameraCal3Bundler(
            Pose3(), Cal3Bundler(120.0, 0.0, 0.0, 0.0, 0.0)),
    ]
    points = [Point3(0.0, 0.0, 5.0), Point3(1.0, 0.5, 6.0)]

    for camera in cameras:
        data.addCamera(camera)

    for point in points:
        track = SfmTrack(point)
        for camera_idx, camera in enumerate(cameras):
            track.addMeasurement(camera_idx, camera.project(point))
        data.addTrack(track)

    return data


def _make_true_bal_like_data():
    """Mirror the small BAL-like C++ CUDA LM test fixture."""
    data = SfmData()
    cameras = [
        PinholeCameraCal3Bundler(
            Pose3(Rot3.RzRyRx(0.03, -0.02, 0.01),
                  Point3(0.2, -0.1, -0.3)),
            Cal3Bundler(150.0, 0.01, -0.0005, 0.0, 0.0)),
        PinholeCameraCal3Bundler(
            Pose3(Rot3.RzRyRx(-0.02, 0.015, -0.025),
                  Point3(0.8, 0.1, -0.2)),
            Cal3Bundler(165.0, -0.008, 0.0007, 0.0, 0.0)),
    ]
    points = [
        Point3(-0.8, -0.4, 4.5),
        Point3(0.6, -0.2, 5.0),
        Point3(-0.2, 0.7, 5.8),
        Point3(0.9, 0.5, 6.4),
    ]

    for camera in cameras:
        data.addCamera(camera)

    for point in points:
        track = SfmTrack(point)
        for camera_idx, camera in enumerate(cameras):
            track.addMeasurement(camera_idx, camera.project(point))
        data.addTrack(track)

    return data


def _make_perturbed_bal_like_data(measured_data):
    """Perturb the true BAL-like fixture using the same deltas as C++."""
    data = SfmData()
    camera_deltas = [
        np.array([0.003, -0.002, 0.001, 0.04, -0.03, 0.02, 1.5, 0.0004,
                  -0.00003]),
        np.array([-0.002, 0.0015, -0.0025, -0.05, 0.02, -0.01, -2.0,
                  -0.0003, 0.00004]),
    ]
    point_deltas = [
        Point3(0.03, -0.02, 0.04),
        Point3(-0.025, 0.015, -0.03),
        Point3(0.02, 0.025, 0.05),
        Point3(-0.015, -0.02, -0.04),
    ]

    for camera_idx, delta in enumerate(camera_deltas):
        data.addCamera(measured_data.camera(camera_idx).retract(delta))

    for track_idx, delta in enumerate(point_deltas):
        measured_track = measured_data.track(track_idx)
        point = measured_track.point3()
        perturbed_point = Point3(point[0] + delta[0], point[1] + delta[1],
                                 point[2] + delta[2])
        track = SfmTrack(perturbed_point)
        for measurement_idx in range(measured_track.numberMeasurements()):
            camera_idx, measurement = measured_track.measurement(measurement_idx)
            track.addMeasurement(camera_idx, measurement)
        data.addTrack(track)

    return data


@unittest.skipIf(cuda is None, "GTSAM was not built with CUDA")
class TestSfm(unittest.TestCase):

    def _run_or_skip_unavailable_runtime(self, func):
        try:
            return func()
        except RuntimeError as exc:
            message = str(exc)
            unavailable_fragments = (
                "no CUDA-capable device",
                "CUDA driver version is insufficient",
                "initialization error",
                "not initialized",
            )
            if any(fragment in message for fragment in unavailable_fragments):
                self.skipTest(f"CUDA runtime is unavailable: {message}")
            raise

    def test_cuda_sfm_params_are_wrapped(self):
        params = cuda.SfmLevenbergMarquardtParams.ceresDefaults()
        params.setLinearSolver(cuda.LinearSolverType.DenseCholesky)
        params.setEliminationMode(gtsam.SfmEliminationMode.Schur)

        self.assertEqual(cuda.LinearSolverType.DenseCholesky,
                         params.getLinearSolver())
        self.assertEqual(gtsam.SfmEliminationMode.Schur,
                         params.getEliminationMode())

    def test_cuda_full_mode_is_exposed_but_reserved(self):
        data = _make_zero_error_sfm_data()
        params = cuda.SfmLevenbergMarquardtParams()
        params.setEliminationMode(gtsam.SfmEliminationMode.Full)
        with self.assertRaisesRegex(
                (ValueError, RuntimeError),
                "Full elimination mode is not implemented"):
            cuda.optimizeSfm(data, params)

    def test_optimize_cuda_sfm_runs_from_python(self):
        data = _make_zero_error_sfm_data()
        params = cuda.SfmLevenbergMarquardtParams()
        params.setMaxIterations(0)

        result = self._run_or_skip_unavailable_runtime(
            lambda: cuda.optimizeSfm(data, params))

        self.assertEqual(0, result.iterations)
        self.assertGreaterEqual(result.initialError, 0.0)
        self.assertAlmostEqual(result.initialError, result.finalError)
        self.assertEqual(data.numberCameras() + data.numberTracks(),
                         result.optimizedValues.size())

    def test_optimize_cuda_sfm_matches_cpp_tiny_bal_behavior(self):
        data = _make_perturbed_bal_like_data(_make_true_bal_like_data())
        params = cuda.SfmLevenbergMarquardtParams()
        params.setMaxIterations(5)
        params.setRelativeErrorTol(1e-12)
        params.setlambdaInitial(1e-3)

        result = self._run_or_skip_unavailable_runtime(
            lambda: cuda.optimizeSfm(data, params))

        self.assertGreater(result.iterations, 0)
        self.assertGreater(result.acceptedSteps, 0)
        self.assertGreater(result.solveLoopElapsed, 0.0)
        self.assertLess(result.finalError, result.initialError)
        self.assertTrue(result.optimizedValues.exists(C(0)))
        self.assertTrue(result.optimizedValues.exists(P(0)))

        camera0 = result.optimizedValues.atPinholeCameraCal3Bundler(C(0))
        point0 = result.optimizedValues.atPoint3(P(0))
        self.assertTrue(math.isfinite(camera0.calibration().fx()))
        self.assertGreater(camera0.calibration().fx(), 0.0)
        self.assertTrue(math.isfinite(point0[0]))


if __name__ == "__main__":
    unittest.main()
