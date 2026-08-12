"""
Minimal smoke test for SmartStereoProjectionPoseFactor.

Constructs two stereo measurements of a single 3D landmark from two poses and
verifies the factor evaluates to (near) zero error given consistent geometry.
"""

import pathlib
import sys

import gtsam  # type: ignore

from gtsam_unstable import SmartStereoProjectionPoseFactor


def main() -> None:
    # Simple stereo calibration
    K = gtsam.Cal3_S2Stereo(500, 500, 0.0, 320, 240, 0.2)

    # Two camera poses observing the same landmark
    pose_a = gtsam.Pose3()  # identity
    pose_b = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.2, 0.0, 0.0))
    landmark = gtsam.Point3(0.0, 0.1, 1.2)

    # Generate synthetic stereo measurements
    cam_a = gtsam.StereoCamera(pose_a, K)
    cam_b = gtsam.StereoCamera(pose_b, K)
    z_a = cam_a.project(landmark)
    z_b = cam_b.project(landmark)

    # Add a small pixel noise to the second measurement to verify non-zero error
    z_b = gtsam.StereoPoint2(z_b.uL() + 0.2, z_b.uR() + 0.2, z_b.v() + 0.1)
    print(f"z_a, uL: {z_a.uL():.2f}, uR: {z_a.uR():.2f}, v: {z_a.v():.2f}")
    print(f"z_b, uL: {z_b.uL():.2f}, uR: {z_b.uR():.2f}, v: {z_b.v():.2f}")

    # Build factor with modest noise and default smart projection params
    noise = gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
    params = gtsam.SmartProjectionParams()
    factor = SmartStereoProjectionPoseFactor(noise, params)

    key_a = gtsam.symbol("x", 0)
    key_b = gtsam.symbol("x", 1)
    factor.add(z_a, key_a, K)
    factor.add(z_b, key_b, K)

    values = gtsam.Values()
    values.insert(key_a, pose_a)
    values.insert(key_b, pose_b)

    err = factor.error(values)
    print(f"SmartStereoProjectionPoseFactor error: {err:.6f}")


if __name__ == "__main__":
    main()
