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


if __name__ == "__main__":
    unittest.main()
