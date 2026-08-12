"""Module definition file for GTSAM"""

# pylint: disable=import-outside-toplevel, global-variable-not-assigned, possibly-unused-variable, import-error, import-self

import sys
import os

if sys.platform == "win32":
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)))

from gtsam.utils import findExampleDataFile

from gtsam import gtsam, utils
from gtsam.gtsam import *

#### Typedefs to allow for backwards compatibility
#TODO(Varun) deprecate in future release
# gtsam
KeyVector = list
# base
IndexPairSetMap = dict
IndexPairVector = list
# geometry
Point2Vector = list
Pose3Vector = list
Rot3Vector = list
Point2Pairs = list
Point3Pairs = list
Pose2Pairs = list
Pose3Pairs = list
Se23 = ExtendedPose32
# sfm
BinaryMeasurementsPoint3 = list
BinaryMeasurementsUnit3 = list
BinaryMeasurementsRot3 = list
KeyPairDoubleMap = dict
SfmTrack2dVector = list
SfmTracks = list
SfmCameras = list
SfmMeasurementVector = list
MatchIndicesMap = dict
KeypointsVector = list
# slam
BetweenFactorPose3s = list
BetweenFactorPose2s = list


class FixedLagSmootherKeyTimestampMap(dict):
    """Class to provide backwards compatibility"""
    def insert(self, key_value):
        self[key_value[0]] = key_value[1]


#### End typedefs


def _init():
    """This function is to add shims for the long-gone Point2 and Point3 types"""

    import numpy as np

    global Point2  # export function

    def Point2(x=np.nan, y=np.nan):
        """Shim for the deleted Point2 type."""
        if isinstance(x, np.ndarray):
            assert x.shape == (2, ), "Point2 takes 2-vector"
            return x  # "copy constructor"
        return np.array([x, y], dtype=float)

    global Point3  # export function

    def Point3(x=np.nan, y=np.nan, z=np.nan):
        """Shim for the deleted Point3 type."""
        if isinstance(x, np.ndarray):
            assert x.shape == (3, ), "Point3 takes 3-vector"
            return x  # "copy constructor"
        return np.array([x, y, z], dtype=float)

    # Validate rotation matrix input from Python (zero overhead in C++).
    # The tolerance is deliberately loose (1e-3): the goal is to reject the
    # degenerate inputs that previously crashed the wrapper (zero, scaled, or
    # reflected matrices, see issue #2300) while still accepting real-world
    # rotation matrices rounded to a few decimals. Use Rot3.IsValid(R, tol)
    # for strict checking, or Rot3.ClosestTo(R) to project onto SO(3).
    global Rot3
    _Rot3_orig = gtsam.Rot3

    class Rot3(_Rot3_orig):
        def __init__(self, *args, **kwargs):
            if len(args) == 1 and not kwargs:
                try:
                    arr = np.asarray(args[0], dtype=float)
                except (TypeError, ValueError):
                    arr = None
                if arr is not None and arr.shape == (3, 3) and not _Rot3_orig.IsValid(arr, 1e-3):
                    raise ValueError(
                        "Rot3: matrix is not a valid rotation "
                        "(must be orthonormal with det +1). "
                        "Use Rot3.ClosestTo() to project to the nearest rotation.")
            super().__init__(*args, **kwargs)

    # for interactive debugging
    if __name__ == "__main__":
        # we want all definitions accessible
        globals().update(locals())


_init()
