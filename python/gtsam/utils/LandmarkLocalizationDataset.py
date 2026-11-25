"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Synthetic data for the known-landmark localization examples.
"""

from typing import NamedTuple, Optional, Tuple

import gtsam
import numpy as np
from gtsam.symbol_shorthand import L

# Each point measurement has a ray-aligned covariance ellipsoid. Its two equal
# lateral standard deviations are sigma_l, and its radial standard deviation is
# sigma_r. We parameterize their ratio by
#
#   rho = anisotropicity = sigma_r / sigma_l,
#
# so sigma_r = rho*sigma_l. The covariance eigenvalues are sigma_l^2,
# sigma_l^2, and sigma_r^2, and
#
#   cond(Sigma) = sigma_r^2 / sigma_l^2 = rho^2.
#
# Thus ANISOTROPICITY is the axis ratio of the one-standard-deviation ellipsoid,
# equivalently sqrt(cond(Sigma)). A value of 10 means that noise along the
# observation ray has ten times the standard deviation, and one hundred times
# the variance, of noise perpendicular to the ray.
LATERAL_SIGMA = 0.01
ANISOTROPICITY = 10.0
ODOMETRY_SIGMA = 0.1


class LandmarkMeasurement(NamedTuple):
    """A known wL observed as kP with a full information matrix."""

    k: int
    l: int
    wL: np.ndarray
    kP: np.ndarray
    information: np.ndarray


class OdometryMeasurement(NamedTuple):
    """An iTj measurement between consecutive poses."""

    i: int
    j: int
    iTj: gtsam.Pose3


class Measurements(NamedTuple):
    """Landmark and odometry measurements for one trajectory."""

    landmarks: Tuple[LandmarkMeasurement, ...]
    odometry: Tuple[OdometryMeasurement, ...]


class LandmarkLocalizationDataset:
    """Create conventional wTk ground truth and corresponding measurements."""

    def __init__(self, num_poses: int):
        if num_poses < 1:
            raise ValueError("num_poses must be positive")

        self.num_poses = num_poses
        wLs = (
            np.array([4.0, 1.0, 2.0]),
            np.array([-1.0, 3.0, 5.0]),
            np.array([2.0, -4.0, 3.0]),
            np.array([5.0, 2.0, -1.0]),
        )
        for wL in wLs:
            wL.setflags(write=False)
        self.wLs = wLs

        kTkp1 = gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.03, -0.05, 0.08),
            np.array([0.3, -0.1, 0.2]),
        )
        wTks = [gtsam.Pose3()]
        for _ in range(1, num_poses):
            wTks.append(wTks[-1].compose(kTkp1))
        self.wTks = tuple(wTks)

    @staticmethod
    def _information(kP: np.ndarray) -> np.ndarray:
        # Let u be a unit observation ray. The orthogonal projectors onto the
        # radial and lateral subspaces are P_r = uu^T and P_l = I-uu^T. Hence
        #
        #   Sigma = sigma_l^2 P_l + sigma_r^2 P_r
        #         = sigma_l^2 I + (sigma_r^2-sigma_l^2) uu^T.
        #
        # Since P_l and P_r are orthogonal projectors, Sigma is inverted by
        # inverting its eigenvalues:
        #
        #   W = Sigma^{-1}
        #     = (1/sigma_l^2) P_l + (1/sigma_r^2) P_r
        #     = lateral_precision I
        #       + (radial_precision-lateral_precision) uu^T.
        #
        # Because sigma_r > sigma_l, radial errors are penalized less strongly
        # than lateral errors.
        kRay = kP / np.linalg.norm(kP)
        radial_sigma = ANISOTROPICITY * LATERAL_SIGMA
        lateral_precision = 1.0 / LATERAL_SIGMA**2
        radial_precision = 1.0 / radial_sigma**2
        return lateral_precision * np.eye(3) + (
            radial_precision - lateral_precision
        ) * np.outer(kRay, kRay)

    def exact_measurements(self) -> Measurements:
        """Return exact measurements without modifying the trajectory."""
        return self._measurements()

    def perturbed_measurements(
        self, point_seed: int = 42, odometry_seed: int = 43
    ) -> Measurements:
        """Return deterministic noisy measurements without changing ground truth."""
        return self._measurements(point_seed, odometry_seed)

    @staticmethod
    def _perturbed_kP(kP: np.ndarray, seed: int) -> np.ndarray:
        """Perturb kP with a sample from its ray-aligned covariance."""
        # Use NumPy's explicitly seeded MT19937 stream instead of the wrapped
        # C++ sampler, whose generated sequence differs between toolchains.
        z = np.random.RandomState(seed).standard_normal(3)
        kRay = kP / np.linalg.norm(kP)
        radial = np.dot(z, kRay) * kRay
        lateral = z - radial
        radial_sigma = ANISOTROPICITY * LATERAL_SIGMA
        return kP + LATERAL_SIGMA * lateral + radial_sigma * radial

    def write_g2o(self, filename: str, measurements: Measurements) -> None:
        """Write conventional wTk, wL, odometry, and landmark observations."""
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        for k, wTk in enumerate(self.wTks):
            values.insert(k, wTk)
        for l, wL in enumerate(self.wLs):
            values.insertPoint3(L(l), wL)

        for measurement in measurements.landmarks:
            kP = measurement.kP
            range_ = np.linalg.norm(kP)
            bearing = gtsam.Unit3(kP)
            J = np.column_stack((range_ * bearing.basis(), bearing.unitVector()))
            bearing_range_information = J.T @ measurement.information @ J
            graph.add(
                gtsam.BearingRangeFactor3D(
                    measurement.k,
                    L(measurement.l),
                    bearing,
                    range_,
                    gtsam.noiseModel.Gaussian.Information(bearing_range_information),
                )
            )

        odometry_model = gtsam.noiseModel.Isotropic.Sigma(6, ODOMETRY_SIGMA)
        for measurement in measurements.odometry:
            graph.add(
                gtsam.BetweenFactorPose3(
                    measurement.i,
                    measurement.j,
                    measurement.iTj,
                    odometry_model,
                )
            )

        gtsam.writeG2o(graph, values, filename)

    def _measurements(
        self,
        point_seed: Optional[int] = None,
        odometry_seed: Optional[int] = None,
    ) -> Measurements:
        landmarks = []
        for k, wTk in enumerate(self.wTks):
            for l, wL in enumerate(self.wLs):
                kP = wTk.transformTo(wL)
                information = self._information(kP)
                if point_seed is not None:
                    kP = self._perturbed_kP(kP, point_seed + len(landmarks))
                landmarks.append(LandmarkMeasurement(k, l, wL, kP, information))

        odometry = []
        odometry_rng = None
        if odometry_seed is not None:
            odometry_rng = np.random.RandomState(odometry_seed)
        for i in range(self.num_poses - 1):
            j = i + 1
            iTj = self.wTks[i].between(self.wTks[j])
            if odometry_rng is not None:
                tangent_noise = ODOMETRY_SIGMA * odometry_rng.standard_normal(6)
                iTj = iTj.retract(tangent_noise)
            odometry.append(OdometryMeasurement(i, j, iTj))

        return Measurements(tuple(landmarks), tuple(odometry))
