"""Module definition file for GTSAM"""

# pylint: disable=import-outside-toplevel, global-variable-not-assigned, possibly-unused-variable, import-error, import-self

import sys

from gtsam import gtsam, utils
from gtsam.gtsam import *
from gtsam.utils import findExampleDataFile


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

    # for interactive debugging
    if __name__ == "__main__":
        # we want all definitions accessible
        globals().update(locals())


_init()
