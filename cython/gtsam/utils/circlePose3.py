import gtsam
import numpy as np
from math import pi, cos, sin


def circlePose3(numPoses=8, radius=1.0, symbolChar=0):
    """
    circlePose3 generates a set of poses in a circle. This function
    returns those poses inside a gtsam.Values object, with sequential
    keys starting from 0. An optional character may be provided, which
    will be stored in the msb of each key (i.e. gtsam.Symbol).

    We use aerospace/navlab convention, X forward, Y right, Z down
    First pose will be at (R,0,0)
    ^y   ^ X
    |    |
    z-->xZ--> Y  (z pointing towards viewer, Z pointing away from viewer)
    Vehicle at p0 is looking towards y axis (X-axis points towards world y)
    """

    # Force symbolChar to be a single character
    if type(symbolChar) is str:
        symbolChar = ord(symbolChar[0])

    values = gtsam.Values()
    theta = 0.0
    dtheta = 2 * pi / numPoses
    gRo = gtsam.Rot3(
        np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]], order='F'))
    for i in range(numPoses):
        key = gtsam.symbol(symbolChar, i)
        gti = gtsam.Point3(radius * cos(theta), radius * sin(theta), 0)
        oRi = gtsam.Rot3.Yaw(
            -theta)  # negative yaw goes counterclockwise, with Z down !
        gTi = gtsam.Pose3(gRo.compose(oRi), gti)
        values.insert(key, gTi)
        theta = theta + dtheta
    return values
