"""TorchFactor: Bridge between PyTorch autograd and GTSAM factor graphs.

Allows defining GTSAM factors using PyTorch tensors and automatic
differentiation, eliminating the need for hand-derived Jacobians.

Requires: pip install torch
"""

from typing import Callable, List, Optional, Sequence

import numpy as np

try:
    import torch
    from torch import Tensor
except ImportError:
    raise ImportError(
        "TorchFactor requires PyTorch. Install with: pip install torch"
    )

import gtsam


class TorchFactor:
    """A GTSAM factor whose error and Jacobians are computed via PyTorch autograd.

    Usage:
        def residual_fn(poses: List[Tensor]) -> Tensor:
            # poses[i] is a tensor for the i-th variable
            # Return the unwhitened error vector
            p0, p1 = poses
            diff = p1 - p0 - measurement
            return diff

        factor = TorchFactor(
            noise_model=gtsam.noiseModel.Isotropic.Sigma(3, 0.1),
            keys=[key0, key1],
            residual_fn=residual_fn,
            value_extractors=[extract_pose3, extract_pose3],
        )
        graph.add(factor.as_custom_factor())

    The residual_fn receives a list of torch tensors (one per key) and must
    return a 1-D tensor representing the unwhitened error. Jacobians are
    computed automatically via torch.autograd.functional.jacobian.
    """

    def __init__(
        self,
        noise_model: gtsam.noiseModel.Base,
        keys: Sequence[int],
        residual_fn: Callable[[List[Tensor]], Tensor],
        value_extractors: Optional[Sequence[Callable]] = None,
        dtype: torch.dtype = torch.float64,
    ):
        """
        Args:
            noise_model: GTSAM noise model for this factor.
            keys: Variable keys this factor connects to.
            residual_fn: Function mapping list of tensors to error tensor.
            value_extractors: Functions to extract numpy arrays from Values
                for each key. If None, assumes all variables are vectors
                accessed via values.atVector(key).
            dtype: Torch dtype for computations (float64 for GTSAM compatibility).
        """
        self._noise_model = noise_model
        self._keys = list(keys)
        self._residual_fn = residual_fn
        self._dtype = dtype

        if value_extractors is None:
            self._extractors = [self._default_extractor] * len(keys)
        else:
            if len(value_extractors) != len(keys):
                raise ValueError(
                    f"Got {len(value_extractors)} extractors for {len(keys)} keys"
                )
            self._extractors = list(value_extractors)

    @staticmethod
    def _default_extractor(values: gtsam.Values, key: int) -> np.ndarray:
        """Default: extract variable as a flat vector."""
        return values.atVector(key)

    def _error_fn(
        self, this: gtsam.CustomFactor, values: gtsam.Values, H
    ) -> np.ndarray:
        """Error function passed to CustomFactor."""
        # Extract numpy values for each key
        np_values = [
            self._extractors[i](values, self._keys[i])
            for i in range(len(self._keys))
        ]

        if H is not None:
            # Need Jacobians: use autograd
            tensors = [
                torch.tensor(v, dtype=self._dtype, requires_grad=True)
                for v in np_values
            ]

            error = self._residual_fn(tensors)
            error_np = error.detach().numpy()

            # Compute Jacobian for each input
            for i, t in enumerate(tensors):
                J = torch.autograd.functional.jacobian(
                    lambda x: self._residual_fn(
                        tensors[:i] + [x] + tensors[i + 1:]
                    ),
                    t,
                )
                H[i] = J.detach().numpy()
        else:
            # No Jacobians needed, just compute error
            tensors = [
                torch.tensor(v, dtype=self._dtype, requires_grad=False)
                for v in np_values
            ]
            error = self._residual_fn(tensors)
            error_np = error.detach().numpy()

        return error_np

    def as_custom_factor(self) -> gtsam.CustomFactor:
        """Create the GTSAM CustomFactor wrapping this TorchFactor."""
        return gtsam.CustomFactor(
            self._noise_model, self._keys, self._error_fn
        )


# --- Convenience extractors for common GTSAM types ---

def extract_vector(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract a Vector variable."""
    return values.atVector(key)


def extract_pose2(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract Pose2 as [x, y, theta]."""
    pose = values.atPose2(key)
    return np.array([pose.x(), pose.y(), pose.theta()])


def extract_pose3(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract Pose3 as 6-vector in tangent space (relative to identity)."""
    pose = values.atPose3(key)
    return gtsam.Pose3.Logmap(pose)


def extract_point2(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract Point2 as [x, y]."""
    return values.atPoint2(key)


def extract_point3(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract Point3 as [x, y, z]."""
    return values.atPoint3(key)


def extract_rot3(values: gtsam.Values, key: int) -> np.ndarray:
    """Extract Rot3 as 3-vector in tangent space."""
    rot = values.atRot3(key)
    return gtsam.Rot3.Logmap(rot)
