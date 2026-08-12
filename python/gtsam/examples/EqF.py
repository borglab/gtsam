"""
Implementation of Attitude-Bias-Calibration EqF form:
"Overcoming Bias: Equivariant Filter Design for Biased Attitude Estimation with Online Calibration"
https://ieeexplore.ieee.org/document/9905914

This module is Alessandro Fornasier's equivariant filter code (https://github.com/aau-cns/ABC-EqF) 
converted to use GTSAM's libraries.

Authors: Jennifer Oum & Darshan Rajasekaran
"""

import numpy as np
import gtsam
from gtsam import Rot3, Unit3
from dataclasses import dataclass
from typing import List

coordinate = "EXPONENTIAL"


def checkNorm(x: np.ndarray, tol: float = 1e-3):
    """Check norm of a vector being 1 or nan

    :param x: A numpy array
    :param tol: tollerance, default 1e-3
    :return: Boolean true if norm is 1 or nan
    """
    return abs(np.linalg.norm(x) - 1) < tol or np.isnan(np.linalg.norm(x))


class State:
    """Define the state of the Biased Attitude System
    ----------
    R is a rotation matrix representing the attitude of the body
    b is a 3-vector representing the gyroscope bias
    S is a list of rotation matrix, each representing the calibration of the corresponding direction sensor
    ----------
    Let's assume we want to use three known direction a, b, and c, where only the sensor that measure b is
    uncalibrated (we'd like to estimate the calibration states). Therefore, the system's d list looks like
    d = [b, a, c], and the S list should look like S = [Sb]. The association between d and S is done via indeces.
    In general S[i] correspond to the calibration state of the sensor that measure the direcion d[i]
    ----------
    """

    # Attitude rotation matrix R
    R: Rot3

    # Gyroscope bias b
    b: np.ndarray

    # Sensor calibrations S
    S: List[Rot3]

    def __init__(
        self,
        R: Rot3 = Rot3.Identity(),
        b: np.ndarray = np.zeros(3),
        S: List[Rot3] = None,
    ):
        """Initialize State

        :param R: A SO3 element representing the attitude of the system as a rotation matrix
        :param b: A numpy array with size 3 representing the gyroscope bias
        :param S: A list of SO3 elements representing the calibration states for "uncalibrated" sensors,
        if no sensor require a calibration state, than S will be initialized as an empty list
        """

        if not isinstance(R, gtsam.Rot3):

            raise TypeError(
                "the attitude rotation matrix R has to be of type SO3 but type is",
                type(R),
            )
        self.R = R

        if not (isinstance(b, np.ndarray) and b.size == 3):
            raise TypeError(
                "The gyroscope bias has to be probvided as numpy array with size 3"
            )
        self.b = b

        if S is None:
            self.S = []
        else:
            if not isinstance(S, list):
                raise TypeError("Calibration states has to be provided as a list")
            for calibration in S:
                if not isinstance(calibration, Rot3):
                    raise TypeError(
                        "Elements of the list of calibration states have to be of type SO3"
                    )
            self.S = S

    @staticmethod
    def identity(n: int):
        """Return a identity state  with n calibration states

        :param n: number of elements in list B associated with calibration states
        :return: The identity element of the State
        """

        return State(Rot3.Identity(), np.zeros(3), [Rot3.Identity() for _ in range(n)])


class Input:
    """Define the input space of the Biased Attitude System
    ----------
    w is a 3-vector representing the angular velocity measured by a gyroscope
    ----------
    """

    # Angular velocity
    w: np.ndarray

    # Noise covariance of angular velocity
    Sigma: np.ndarray

    def __init__(self, w: np.ndarray, Sigma: np.ndarray):
        """Initialize Input

        :param w: A numpy array with size 3 representing the angular velocity measurement from a gyroscope
        :param Sigma: A numpy array with shape (6, 6) representing the noise covariance of the
        angular velocity measurement and gyro bias random walk
        """

        if not (isinstance(w, np.ndarray) and w.size == 3):
            raise TypeError(
                "Angular velocity has to be provided as a numpy array with size 3"
            )
        if not (
            isinstance(Sigma, np.ndarray) and Sigma.shape[0] == Sigma.shape[1] == 6
        ):
            raise TypeError(
                "Input measurement noise covariance has to be provided as a numpy array with shape (6. 6)"
            )
        if not np.all(np.linalg.eigvals(Sigma) >= 0):
            raise TypeError("Covariance matrix has to be semi-positive definite")

        self.w = w
        self.Sigma = Sigma

    @staticmethod
    def random() -> "Input":
        """Return a random angular velocity

        :return: A random angular velocity as a Input element
        """

        return Input(np.random.randn(3), np.eye(6))

    def W(self) -> np.ndarray:
        """Return the Input as a skew-symmetric matrix

        :return: self.w as a skew-symmetric matrix
        """

        return Rot3.Hat(self.w)


class G:
    """Symmetry group (SO(3) |x so(3)) x SO(3) x ... x SO(3)
    ----------
    Each element of the B list is associated with a calibration states in State's S list where the association is done
    via corresponding index. In general B[i] is the SO(3) element of the symmetry group that correspond to the
    state's calibration state S[i]. For example, let's assume we want to use three known direction a, b, and c, where
    only the sensor that measure b is uncalibrated (we'd like to estimate the calibration states). Therefore,
    the system's d list is defined as d = [b, a, c], and the state's S list is defined as S = [Sb]. The symmetry group
    B list should be defined as B = [Bb] where Ba is the SO(3) element of the symmetry group that is related to Sb
    ----------
    """

    A: Rot3
    a: np.ndarray
    B: List[Rot3]

    def __init__(
        self,
        A: Rot3 = Rot3.Identity(),
        a: np.ndarray = np.zeros((3, 3)),
        B: List[Rot3] = None,
    ):
        """Initialize the symmetry group G

        :param A: SO3 element
        :param a: np.ndarray with shape (3, 3) corresponding to a skew symmetric matrix
        :param B: list of SO3 elements
        """

        if not isinstance(A, Rot3):
            raise TypeError("A has to be of type SO3")
        self.A = A
        if not (isinstance(a, np.ndarray) and a.shape == (3, 3)):
            raise TypeError("a has to be a numpy array with shape (3, 3)")
        self.a = a
        if B is None:
            self.B = []
        else:
            for b in B:
                if not isinstance(b, Rot3):
                    raise TypeError("Elements of B have to be of type SO3")
            self.B = B

    def __mul__(self, other: "G") -> "G":
        """Define the group operation

        :param other: G
        :return: A element of the group G given by the "multiplication" of self and other
        """

        assert isinstance(other, G)
        assert len(self.B) == len(other.B)
        return G(
            self.A * other.A,
            self.a + Rot3.Hat(self.A.matrix() @ Rot3.Vee(other.a)),
            [self.B[i] * other.B[i] for i in range(len(self.B))],
        )

    @staticmethod
    def identity(n: int):
        """Return the identity of the symmetry group with n elements of SO3 related to sensor calibration states

        :param n: number of elements in list B associated with calibration states
        :return: The identity of the group G
        """

        return G(Rot3.Identity(), np.zeros((3, 3)), [Rot3.Identity() for _ in range(n)])

    @staticmethod
    def Rot3LeftJacobian(arr: np.ndarray) -> np.ndarray:
        """Return the SO(3) Left Jacobian
        :param arr: A numpy array with size 3
        :return: The left Jacobian of SO(3)
        """
        if not (isinstance(arr, np.ndarray) and arr.size == 3):
            raise ValueError("A numpy array with size 3 has to be provided")
        angle = np.linalg.norm(arr)
        # Near |phi|==0, use first order Taylor expansion
        if np.isclose(angle, 0.0):
            return np.eye(3) + 0.5 * Rot3.Hat(arr)
        axis = arr / angle
        s = np.sin(angle)
        c = np.cos(angle)
        return (
            (s / angle) * np.eye(3)
            + (1 - (s / angle)) * np.outer(axis, axis)
            + ((1 - c) / angle) * Rot3.Hat(axis)
        )

    def exp(x: np.ndarray) -> "G":
        """Return a group element X given by X = exp(x) where x is a numpy array

        :param x: A numpy array
        :return: A element of the group G given by the exponential of x
        """

        if not (isinstance(x, np.ndarray) and x.size >= 6):
            raise ValueError(
                "Wrong shape, a numpy array with size 3n has to be provided"
            )
        if (x.size % 3) != 0:
            raise ValueError(
                "Wrong size, a numpy array with size multiple of 3 has to be provided"
            )

        n = int((x.size - 6) / 3)
        A = Rot3.Expmap(x[0:3])
        a = Rot3.Hat(G.Rot3LeftJacobian(x[0:3]) @ x[3:6])
        B = [Rot3.Expmap(x[(6 + 3 * i) : (9 + 3 * i)]) for i in range(n)]

        return G(A, a, B)

    def inv(self) -> "G":
        """Return the inverse element of the symmetry group

        :return: A element of the group G given by the inverse of self
        """

        return G(
            self.A.inverse(),
            -Rot3.Hat(self.A.inverse().matrix() @ Rot3.Vee(self.a)),
            [B.inverse() for B in self.B],
        )


class Direction:
    """Define a direction as a S2 element"""

    # Direction
    d: Unit3

    def __init__(self, d: np.ndarray):
        """Initialize direction

        :param d: A numpy array with size 3 and norm 1 representing the direction
        """

        if not (isinstance(d, np.ndarray) and d.size == 3 and checkNorm(d)):
            raise TypeError("Direction has to be provided as a 3 vector")
        self.d = Unit3(d)


def blockDiag(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Create a lock diagonal matrix from blocks A and B

    :param A: numpy array
    :param B: numpy array
    :return: numpy array representing a block diagonal matrix composed of blocks A and B
    """

    if A is None:
        return B
    elif B is None:
        return A
    else:
        return np.block(
            [
                [A, np.zeros((A.shape[0], B.shape[1]))],
                [np.zeros((B.shape[0], A.shape[1])), B],
            ]
        )


def repBlock(A: np.ndarray, n: int) -> np.ndarray:
    """Create a block diagonal matrix repeating the A block n times

    :param A: numpy array representing the block A
    :param n: number of times to repeat A
    :return: numpy array representing a block diagonal matrix composed of n-times the blocks A
    """

    res = None
    for _ in range(n):
        res = blockDiag(res, A)
    return res


def numericalDifferential(f, x) -> np.ndarray:
    """Compute the numerical derivative via central difference"""

    if isinstance(x, float):
        x = np.reshape([x], (1, 1))
    h = 1e-6
    fx = f(x)
    n = fx.shape[0]
    m = x.shape[0]
    Df = np.zeros((n, m))
    for j in range(m):
        ej = np.zeros(m)
        ej[j] = 1.0
        Df[:, j : j + 1] = (f(x + h * ej) - f(x - h * ej)).reshape(m, 1) / (2 * h)
    return Df


def lift(xi: State, u: Input) -> np.ndarray:
    """The Lift of the system (Theorem 3.8, Equation 7)

    :param xi: A element of the State
    :param u: A element of the Input space
    :return: A numpy array representing the Lift
    """

    n = len(xi.S)
    L = np.zeros(6 + 3 * n)
    L[0:3] = u.w - xi.b
    L[3:6] = -u.W() @ xi.b
    for i in range(n):
        L[(6 + 3 * i) : (9 + 3 * i)] = xi.S[i].inverse().matrix() @ L[0:3]

    return L


def checkNorm(x: np.ndarray, tol: float = 1e-3):
    """Check norm of a vector being 1 or nan

    :param x: A numpy array
    :param tol: tollerance, default 1e-3
    :return: Boolean true if norm is 1 or nan
    """
    return abs(np.linalg.norm(x) - 1) < tol or np.isnan(np.linalg.norm(x))


def stateAction(X: G, xi: State) -> State:
    """Action of the symmetry group on the state space, return phi(X, xi) (Equation 4)

    :param X: A element of the group G
    :param xi: A element of the State
    :return: A new element of the state given by the action of phi of G in the State space
    """

    if len(xi.S) != len(X.B):
        raise ValueError(
            "the number of calibration states and B elements of the symmetry group has to match"
        )

    return State(
        xi.R * X.A,
        X.A.inverse().matrix() @ (xi.b - Rot3.Vee(X.a)),
        [(X.A.inverse() * xi.S[i] * X.B[i]) for i in range(len(X.B))],
    )


def velocityAction(X: G, u: Input) -> Input:
    """Action of the symmetry group on the input space, return psi(X, u) (Equation 5)

    :param X: A element of the group G
    :param u: A element of the Input
    :return: A new element of the Input given by the action of psi of G in the Input space
    """

    return Input(X.A.inverse().matrix() @ (u.w - Rot3.Vee(X.a)), u.Sigma)


def outputAction(X: G, y: Direction, idx: int = -1) -> np.ndarray:
    """Action of the symmetry group on the output space, return rho(X, y) (Equation 6)

    :param X: A element of the group G
    :param y: A direction measurement
    :param idx: indicate the index of the B element in the list, -1 in case no B element exist
    :return: A numpy array given by the action of rho of G in the Output space
    """

    if idx == -1:
        return X.A.inverse().matrix() @ y.d.unitVector()
    else:
        return X.B[idx].inverse().matrix() @ y.d.unitVector()


def local_coords(e: State) -> np.ndarray:
    """Local coordinates assuming __xi_0 = identity (Equation 9)

    :param e: A element of the State representing the equivariant error
    :return: Local coordinates assuming __xi_0 = identity
    """

    if coordinate == "EXPONENTIAL":
        tmp = [Rot3.Logmap(S) for S in e.S]
        eps = np.concatenate(
            (
                Rot3.Logmap(e.R),
                e.b,
                np.asarray(tmp).reshape(
                    3 * len(tmp),
                ),
            )
        )
    elif coordinate == "NORMAL":
        raise ValueError("Normal coordinate representation is not implemented yet")
        # X = G(e.R, -SO3.Rot3.Hat(e.R @ e.b), e.S)
        # eps = G.log(X)
    else:
        raise ValueError("Invalid coordinate representation")

    return eps


def local_coords_inv(eps: np.ndarray) -> "State":
    """Local coordinates inverse assuming __xi_0 = identity

    :param eps: A numpy array representing the equivariant error in local coordinates
    :return: Local coordinates inverse assuming __xi_0 = identity
    """

    X = G.exp(eps)  # G
    if coordinate == "EXPONENTIAL":
        e = State(X.A, eps[3:6, :], X.B)  # State
    elif coordinate == "NORMAL":
        raise ValueError("Normal coordinate representation is not implemented yet")
        # stateAction(X, State(SO3.identity(), np.zeros(3), [SO3.identity() for _ in range(len(X.B))]))
    else:
        raise ValueError("Invalid coordinate representation")

    return e


def stateActionDiff(xi: State) -> np.ndarray:
    """Differential of the phi action phi(xi, E) at E = Id in local coordinates (can be found within equation 23)

    :param xi: A element of the State
    :return: (Dtheta) * (Dphi(xi, E) at E = Id)
    """
    coordsAction = lambda U: local_coords(stateAction(G.exp(U), xi))
    differential = numericalDifferential(coordsAction, np.zeros(6 + 3 * len(xi.S)))
    return differential


class Measurement:
    """Define a measurement
    ----------
    cal_idx is a index corresponding to the cal_idx-th calibration related to the measurement. Let's consider the case
    of 2 uncalibrated sensor with two associated calibration state in State.S = [S0, S1], and a single calibrated sensor.
    cal_idx = 0 indicates a measurement coming from the sensor that has calibration S0, cal_idx = 1 indicates a
    measurement coming from the sensor that has calibration S1. cal_idx = -1 indicates that the measurement is coming
    from a calibrated sensor
    ----------
    """

    # measurement
    y: Direction

    # Known direction in global frame
    d: Direction

    # Covariance matrix of the measurement
    Sigma: np.ndarray

    # Calibration index
    cal_idx: int = -1

    def __init__(self, y: np.ndarray, d: np.ndarray, Sigma: np.ndarray, i: int = -1):
        """Initialize measurement

        :param y: A numpy array with size 3 and norm 1 representing the direction measurement in the sensor frame
        :param d: A numpy array with size 3 and norm 1 representing the direction in the global frame
        :param Sigma: A numpy array with shape (3, 3) representing the noise covariance of the direction measurement
        :param i: index of the corresponding calibration state
        """

        if not (isinstance(y, np.ndarray) and y.size == 3 and checkNorm(y)):
            raise TypeError("Measurement has to be provided as a (3, 1) vector")
        if not (isinstance(d, np.ndarray) and d.size == 3 and checkNorm(d)):
            raise TypeError("Direction has to be provided as a (3, 1) vector")
        if not (
            isinstance(Sigma, np.ndarray) and Sigma.shape[0] == Sigma.shape[1] == 3
        ):
            raise TypeError(
                "Direction measurement noise covariance has to be provided as a numpy array with shape (3. 3)"
            )
        if not np.all(np.linalg.eigvals(Sigma) >= 0):
            raise TypeError("Covariance matrix has to be semi-positive definite")
        if not (isinstance(i, int) or i == -1 or i > 0):
            raise TypeError("calibration index is a positive integer or -1")

        self.y = Direction(y)
        self.d = Direction(d)
        self.Sigma = Sigma
        self.cal_idx = i


@dataclass
class Data:
    """Define ground-truth, input and output data"""

    # Ground-truth state
    xi: State
    n_cal: int

    # Input measurements
    u: Input

    # Output measurements as a list of Measurement
    y: list
    n_meas: int

    # Time
    t: float
    dt: float


class EqF:
    def __init__(self, Sigma: np.ndarray, n: int, m: int):
        """Initialize EqF

        :param Sigma: Initial covariance
        :param n: Number of calibration states
        :param m: Total number of available sensor
        """

        self.__dof = 6 + 3 * n
        self.__n_cal = n
        self.__n_sensor = m

        if not (
            isinstance(Sigma, np.ndarray)
            and (Sigma.shape[0] == Sigma.shape[1] == self.__dof)
        ):
            raise TypeError(
                f"Initial covariance has to be provided as a numpy array with shape ({self.__dof}, {self.__dof})"
            )
        if not np.all(np.linalg.eigvals(Sigma) >= 0):
            raise TypeError("Covariance matrix has to be semi-positive definite")
        if not (isinstance(n, int) and n >= 0):
            raise TypeError("Number of calibration state has to be unsigned")
        if not (isinstance(m, int) and m > 1):
            raise TypeError("Number of direction sensor has to be grater-equal than 2")

        self.__X_hat = G.identity(n)
        self.__Sigma = Sigma
        self.__xi_0 = State.identity(n)
        self.__Dphi0 = stateActionDiff(self.__xi_0)  # Within equation 23
        self.__InnovationLift = np.linalg.pinv(self.__Dphi0)  # Within equation 23

    def stateEstimate(self) -> State:
        """Return estimated state

        :return: Estimated state
        """
        return stateAction(self.__X_hat, self.__xi_0)

    def propagation(self, u: Input, dt: float):
        """Propagate the filter state

        :param u: Angular velocity measurement from IMU
        :param dt: delta time between timestamp of last propagation/update and timestamp of angular velocity measurement
        """

        if not isinstance(u, Input):
            raise TypeError(
                "angular velocity measurement has to be provided as a Input element"
            )

        L = lift(self.stateEstimate(), u)  # Equation 7

        Phi_DT = self.__stateTransitionMatrix(u, dt)  # Equation 17
        # Equivalent
        # A0t = self.__stateMatrixA(u)                                                              # Equation 14a
        # Phi_DT = expm(A0t * dt)

        Bt = self.__inputMatrixBt()  # Equation 27
        M_DT = (
            Bt @ blockDiag(u.Sigma, repBlock(1e-9 * np.eye(3), self.__n_cal)) @ Bt.T
        ) * dt

        self.__X_hat = self.__X_hat * G.exp(L * dt)  # Equation 18
        self.__Sigma = Phi_DT @ self.__Sigma @ Phi_DT.T + M_DT  # Equation 19

    def update(self, y: Measurement):
        """Update the filter state

        :param y: A  measurement
        """

        # Cross-check calibration
        assert y.cal_idx <= self.__n_cal

        Ct = self.__measurementMatrixC(y.d, y.cal_idx)  # Equation 14b
        delta = Rot3.Hat(y.d.d.unitVector()) @ outputAction(
            self.__X_hat.inv(), y.y, y.cal_idx
        )
        Dt = self.__outputMatrixDt(y.cal_idx)
        S = Ct @ self.__Sigma @ Ct.T + Dt @ y.Sigma @ Dt.T  # Equation 21
        K = self.__Sigma @ Ct.T @ np.linalg.inv(S)  # Equation 22
        Delta = self.__InnovationLift @ K @ delta  # Equation 23
        self.__X_hat = G.exp(Delta) * self.__X_hat  # Equation 24
        self.__Sigma = (np.eye(self.__dof) - K @ Ct) @ self.__Sigma  # Equation 25

    def __stateMatrixA(self, u: Input) -> np.ndarray:
        """Return the state matrix A0t (Equation 14a)

        :param u: Input
        :return: numpy array representing the state matrix A0t
        """

        W0 = velocityAction(self.__X_hat.inv(), u).W()
        A1 = np.zeros((6, 6))

        if coordinate == "EXPONENTIAL":
            A1[0:3, 3:6] = -np.eye(3)
            A1[3:6, 3:6] = W0
            A2 = repBlock(W0, self.__n_cal)
        elif coordinate == "NORMAL":
            raise ValueError("Normal coordinate representation is not implemented yet")
        else:
            raise ValueError("Invalid coordinate representation")

        return blockDiag(A1, A2)

    def __stateTransitionMatrix(self, u: Input, dt: float) -> np.ndarray:
        """Return the state transition matrix Phi (Equation 17)

        :param u: Input
        :param dt: Delta time
        :return: numpy array representing the state transition matrix Phi
        """

        W0 = velocityAction(self.__X_hat.inv(), u).W()
        Phi1 = np.zeros((6, 6))
        Phi12 = -dt * (np.eye(3) + (dt / 2) * W0 + ((dt**2) / 6) * W0 * W0)
        Phi22 = np.eye(3) + dt * W0 + ((dt**2) / 2) * W0 * W0

        if coordinate == "EXPONENTIAL":
            Phi1[0:3, 0:3] = np.eye(3)
            Phi1[0:3, 3:6] = Phi12
            Phi1[3:6, 3:6] = Phi22
            Phi2 = repBlock(Phi22, self.__n_cal)
        elif coordinate == "NORMAL":
            raise ValueError("Normal coordinate representation is not implemented yet")
        else:
            raise ValueError("Invalid coordinate representation")

        return blockDiag(Phi1, Phi2)

    def __inputMatrixBt(self) -> np.ndarray:
        """Return the Input matrix Bt

        :return: numpy array representing the state matrix Bt
        """

        if coordinate == "EXPONENTIAL":
            B1 = blockDiag(self.__X_hat.A.matrix(), self.__X_hat.A.matrix())
            B2 = None
            for B in self.__X_hat.B:
                B2 = blockDiag(B2, B.matrix())
        elif coordinate == "NORMAL":
            raise ValueError("Normal coordinate representation is not implemented yet")
        else:
            raise ValueError("Invalid coordinate representation")

        return blockDiag(B1, B2)

    def __measurementMatrixC(self, d: Direction, idx: int) -> np.ndarray:
        """Return the measurement matrix C0 (Equation 14b)

        :param d: Known direction
        :param idx: index of the related calibration state
        :return: numpy array representing the measurement matrix C0
        """

        Cc = np.zeros((3, 3 * self.__n_cal))

        # If the measurement is related to a sensor that has a calibration state
        if idx >= 0:
            Cc[(3 * idx) : (3 + 3 * idx), :] = Rot3.Hat(d.d.unitVector())

        return Rot3.Hat(d.d.unitVector()) @ np.hstack(
            (Rot3.Hat(d.d.unitVector()), np.zeros((3, 3)), Cc)
        )

    def __outputMatrixDt(self, idx: int) -> np.ndarray:
        """Return the measurement output matrix Dt

        :param idx: index of the related calibration state
        :return: numpy array representing the output matrix Dt
        """

        # If the measurement is related to a sensor that has a calibration state
        if idx >= 0:
            return self.__X_hat.B[idx].matrix()
        else:
            return self.__X_hat.A.matrix()
