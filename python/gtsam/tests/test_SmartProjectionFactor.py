"""Tests for exact SmartProjectionFactor wrapper signatures."""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


class TestSmartProjectionFactor(GtsamTestCase):
    """Exercise the specialized Hessian and Jacobian return types."""

    def test_linear_factor_specializations_are_registered(self):
        """Every camera dimension used by the wrapper has a registered type."""
        for dimension in (6, 9, 11, 15, 16):
            hessian_type = getattr(gtsam, f"RegularHessianFactor{dimension}")
            jacobian_type = getattr(gtsam, f"JacobianFactorQ{dimension}2")
            self.assertTrue(issubclass(hessian_type, gtsam.HessianFactor))
            self.assertTrue(issubclass(jacobian_type, gtsam.JacobianFactor))

    def test_exact_linear_factor_returns(self):
        """Specialized shared pointers downcast to their wrapped factor bases."""
        calibration = gtsam.Cal3_S2(500.0, 500.0, 0.0, 320.0, 240.0)
        cameras = [
            gtsam.PinholeCameraCal3_S2(gtsam.Pose3(), calibration),
            gtsam.PinholeCameraCal3_S2(
                gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
                calibration,
            ),
        ]
        point = gtsam.Point3(0.0, 0.0, 5.0)
        camera_set = gtsam.CameraSetCal3_S2(cameras)
        factor = gtsam.SmartProjectionFactorPinholeCameraCal3_S2(
            gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
        )
        values = gtsam.Values()

        for index, camera in enumerate(cameras):
            key = X(index)
            factor.add(camera.project(point), key)
            values.insert(key, camera)

        self.assertIsInstance(
            factor.createHessianFactor(camera_set), gtsam.HessianFactor
        )
        self.assertIsInstance(
            factor.createJacobianQFactor(camera_set, 0.0), gtsam.JacobianFactor
        )
        self.assertIsInstance(
            factor.createJacobianQFactor(values, 0.0), gtsam.JacobianFactor
        )
        self.assertIsInstance(factor.linearizeToHessian(values), gtsam.HessianFactor)
        self.assertIsInstance(factor.linearizeToJacobian(values), gtsam.JacobianFactor)

    @staticmethod
    def three_camera_scene():
        """Three poses observing one landmark, with nonzero residuals."""
        calibration = gtsam.Cal3_S2(500.0, 500.0, 0.0, 320.0, 240.0)
        point = gtsam.Point3(0.5, -0.2, 5.0)
        poses = [
            gtsam.Pose3(),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 1.0, 0.0)),
        ]
        factor = gtsam.SmartProjectionPoseFactorCal3_S2(
            gtsam.noiseModel.Isotropic.Sigma(2, 1.0), calibration
        )
        values = gtsam.Values()
        for index, pose in enumerate(poses):
            camera = gtsam.PinholePoseCal3_S2(pose, calibration)
            factor.add(
                camera.project(point) + gtsam.Point2(0.3 * index, -0.1),
                X(index),
            )
            values.insert(X(index), pose)
        return poses, factor, values

    def test_condition_on_fixes_a_camera(self):
        """conditionOn drops fixed keys, keeps the error, never fixes all."""
        poses, factor, values = self.three_camera_scene()

        fixed_values = gtsam.Values()
        fixed_values.insert(X(0), poses[0])
        conditioned = factor.conditionOn(fixed_values)
        self.assertIsInstance(conditioned, gtsam.NonlinearFactor)
        self.assertEqual(list(conditioned.keys()), [X(1), X(2)])
        self.assertEqual(conditioned.dim(), factor.dim())
        live_values = gtsam.Values()
        live_values.insert(X(1), poses[1])
        live_values.insert(X(2), poses[2])
        self.assertAlmostEqual(
            factor.error(values), conditioned.error(live_values), places=9
        )
        self.assertEqual(
            list(conditioned.linearize(live_values).keys()), [X(1), X(2)]
        )
        # Nothing to fix, or everything to fix, gives no factor.
        self.assertIsNone(factor.conditionOn(gtsam.Values()))
        self.assertIsNone(factor.conditionOn(values))

    def test_fix_camera_and_accessors(self):
        """fixCamera returns the concrete type; accessors report the camera."""
        poses, factor, values = self.three_camera_scene()
        fixed = factor.fixCamera(X(1), values)
        self.assertIsInstance(fixed, gtsam.SmartProjectionPoseFactorCal3_S2)
        self.assertEqual(list(fixed.keys()), [X(0), X(2)])
        self.assertEqual(list(fixed.fixedMeasurements()), [1])
        self.assertEqual(list(fixed.activeMeasurements()), [0, 2])
        self.assertTrue(fixed.isFixedMeasurement(1))
        self.assertFalse(fixed.isFixedMeasurement(0))
        fixed_cameras = fixed.fixedCameras()
        self.gtsamAssertEquals(fixed_cameras.at(0).pose(), poses[1])
        with self.assertRaises(IndexError):
            fixed_cameras.at(1)
        # The original is unchanged, and fixCamera agrees with conditionOn.
        self.assertEqual(list(factor.fixedMeasurements()), [])
        fixed_values = gtsam.Values()
        fixed_values.insert(X(1), poses[1])
        self.assertTrue(fixed.equals(factor.conditionOn(fixed_values), 1e-9))
        # A key that is no longer a variable of the factor is rejected.
        with self.assertRaises(ValueError):
            fixed.fixCamera(X(1), values)

    def test_rig_factor_fixes_every_measurement_of_a_pose(self):
        """Conditioning a rig on a body pose fixes both of its cameras."""
        calibration = gtsam.Cal3_S2(500.0, 500.0, 0.0, 320.0, 240.0)
        rig = gtsam.CameraSetPinholePoseCal3_S2()
        rig.push_back(gtsam.PinholePoseCal3_S2(gtsam.Pose3(), calibration))
        rig.push_back(
            gtsam.PinholePoseCal3_S2(
                gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 0.2, 0.0)),
                calibration,
            )
        )
        params = gtsam.SmartProjectionParams(
            gtsam.LinearizationMode.HESSIAN,
            gtsam.DegeneracyMode.ZERO_ON_DEGENERACY,
        )
        factor = gtsam.SmartProjectionRigFactorPinholePoseCal3_S2(
            gtsam.noiseModel.Isotropic.Sigma(2, 1.0), rig, params
        )
        point = gtsam.Point3(0.5, -0.2, 5.0)
        bodies = [
            gtsam.Pose3(),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 1.0, 0.0)),
        ]
        values = gtsam.Values()
        for index, body in enumerate(bodies):
            for camera_id in range(2):
                camera = gtsam.PinholePoseCal3_S2(
                    body.compose(rig.at(camera_id).pose()), calibration
                )
                offset = gtsam.Point2(0.2 * index, -0.1 * camera_id)
                factor.add(camera.project(point) + offset, X(index), camera_id)
            values.insert(X(index), body)

        fixed_values = gtsam.Values()
        fixed_values.insert(X(0), bodies[0])
        conditioned = factor.conditionOn(fixed_values)
        self.assertIsInstance(
            conditioned, gtsam.SmartProjectionRigFactorPinholePoseCal3_S2
        )
        self.assertEqual(list(conditioned.keys()), [X(1), X(2)])
        self.assertEqual(list(conditioned.fixedMeasurements()), [0, 1])
        live_values = gtsam.Values()
        live_values.insert(X(1), bodies[1])
        live_values.insert(X(2), bodies[2])
        self.assertAlmostEqual(
            factor.error(values), conditioned.error(live_values), places=9
        )
        self.assertTrue(
            factor.fixCamera(X(0), values).equals(conditioned, 1e-9)
        )
        # Measurement 5 is camera 1 of the third body pose.
        expected = gtsam.PinholePoseCal3_S2(
            bodies[2].compose(rig.at(1).pose()), calibration
        )
        self.gtsamAssertEquals(
            factor.cameraForMeasurement(5, bodies[2]), expected
        )


if __name__ == "__main__":
    unittest.main()
