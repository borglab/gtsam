"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Numerical derivative functions.
Author: Joel Truher & Frank Dellaert
"""

# pylint: disable=C0103,C0114,C0116,E0611,R0913
# mypy: disable-error-code="import-untyped"
# see numericalDerivative.h

# pybind wants to wrap concrete types, which would have been
# a whole lot of them, so i reimplemented the part of this that
# I needed, using the python approach to "generic" typing.

from typing import Callable, TypeVar
import numpy as np

Y = TypeVar("Y")
X = TypeVar("X")
X1 = TypeVar("X1")
X2 = TypeVar("X2")
X3 = TypeVar("X3")
X4 = TypeVar("X4")
X5 = TypeVar("X5")
X6 = TypeVar("X6")


def local(a: Y, b: Y) -> np.ndarray:
    if type(a) is not type(b):
        raise TypeError(f"a {type(a)} b {type(b)}")
    if isinstance(a, np.ndarray):
        return b - a
    if isinstance(a, (float, int)):
        return np.ndarray([[b - a]])  # type:ignore
    # there is no common superclass for Y
    return a.localCoordinates(b)  # type:ignore


def retract(a, xi: np.ndarray):
    if isinstance(a, (np.ndarray, float, int)):
        return a + xi
    return a.retract(xi)


def numericalDerivative11(h: Callable[[X], Y], x: X, delta=1e-5) -> np.ndarray:
    hx: Y = h(x)
    zeroY = local(hx, hx)
    m = zeroY.shape[0]
    zeroX = local(x, x)
    N = zeroX.shape[0]
    dx = np.zeros(N)
    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = local(hx, h(retract(x, dx)))
        dx[j] = -delta
        dy2 = local(hx, h(retract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21(
    h: Callable[[X1, X2], Y], x1: X1, x2: X2, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2), x1, delta)


def numericalDerivative22(
    h: Callable[[X1, X2], Y], x1: X1, x2: X2, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x), x2, delta)


def numericalDerivative31(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, delta)


def numericalDerivative32(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, delta)


def numericalDerivative33(
    h: Callable[[X1, X2, X3], Y], x1: X1, x2: X2, x3: X3, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x), x3, delta)


def numericalDerivative41(
    h: Callable[[X1, X2, X3, X4], Y], x1: X1, x2: X2, x3: X3, x4: X4, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3, x4), x1, delta)


def numericalDerivative42(
    h: Callable[[X1, X2, X3, X4], Y], x1: X1, x2: X2, x3: X3, x4: X4, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x, x3, x4), x2, delta)


def numericalDerivative43(
    h: Callable[[X1, X2, X3, X4], Y], x1: X1, x2: X2, x3: X3, x4: X4, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x, x4), x3, delta)


def numericalDerivative44(
    h: Callable[[X1, X2, X3, X4], Y], x1: X1, x2: X2, x3: X3, x4: X4, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x), x4, delta)


def numericalDerivative51(
    h: Callable[[X1, X2, X3, X4, X5], Y], x1: X1, x2: X2, x3: X3, x4: X4, x5: X5, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3, x4, x5), x1, delta)


def numericalDerivative52(
    h: Callable[[X1, X2, X3, X4, X5], Y], x1: X1, x2: X2, x3: X3, x4: X4, x5: X5, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x, x3, x4, x5), x2, delta)


def numericalDerivative53(
    h: Callable[[X1, X2, X3, X4, X5], Y], x1: X1, x2: X2, x3: X3, x4: X4, x5: X5, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x, x4, x5), x3, delta)


def numericalDerivative54(
    h: Callable[[X1, X2, X3, X4, X5], Y], x1: X1, x2: X2, x3: X3, x4: X4, x5: X5, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x, x5), x4, delta)


def numericalDerivative55(
    h: Callable[[X1, X2, X3, X4, X5], Y], x1: X1, x2: X2, x3: X3, x4: X4, x5: X5, delta=1e-5
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x4, x), x5, delta)


def numericalDerivative61(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x, x2, x3, x4, x5, x6), x1, delta)


def numericalDerivative62(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x, x3, x4, x5, x6), x2, delta)


def numericalDerivative63(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x, x4, x5, x6), x3, delta)


def numericalDerivative64(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x, x5, x6), x4, delta)


def numericalDerivative65(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x4, x, x6), x5, delta)


def numericalDerivative66(
    h: Callable[[X1, X2, X3, X4, X5, X6], Y],
    x1: X1,
    x2: X2,
    x3: X3,
    x4: X4,
    x5: X5,
    x6: X6,
    delta=1e-5,
) -> np.ndarray:
    return numericalDerivative11(lambda x: h(x1, x2, x3, x4, x5, x), x6, delta)
