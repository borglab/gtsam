"""
Minimal end-to-end example of the TG-EqF biased-INS filter
(gtsam_unstable.tgeqf.TGEqF): build a reference state and initial
covariance, construct the filter, propagate a few IMU steps, apply a
position and a DVL update, and inspect the resulting estimate.
"""
import numpy as np

from gtsam_unstable import tgeqf


def main():
    # 1. Reference state xi_ref: the fixed origin the filter linearizes
    # about. Using the manifold identity keeps the origin-chart transport
    # trivial (see TGEqF's constructor doc).
    xi_ref = tgeqf.State.identity()

    # 2. Initial covariance, ordered [R, v, p, b_w, b_a] (15x15 physical
    # block); TGEqF.initialCovariance() appends the virtual-bias block to
    # produce the full 18x18 Sigma0 the filter expects.
    Sigma_physical = np.diag([
        (0.05)**2, (0.05)**2, (0.05)**2,  # attitude, rad
        0.1**2, 0.1**2, 0.1**2,           # velocity, m/s
        0.2**2, 0.2**2, 0.2**2,           # position, m
        (1e-3)**2, (1e-3)**2, (1e-3)**2,  # gyro bias, rad/s
        (1e-2)**2, (1e-2)**2, (1e-2)**2,  # accel bias, m/s^2
    ])
    Sigma0 = tgeqf.TGEqF.initialCovariance(Sigma_physical)

    # 3. Construct the filter at the reference state with X0 = identity, so
    # the initial estimate coincides with xi_ref.
    filt = tgeqf.TGEqF(xi_ref, Sigma0)

    # 4. Propagate a few IMU steps. A body at rest under gravity measures
    # zero angular rate and specific force -g_vec.
    g_vec = np.array([0.0, 0.0, -9.81])
    w_meas = np.array([0.0, 0.0, 0.0])
    a_meas = -g_vec
    noise = tgeqf.ImuNoise()
    noise.gyro = np.full(3, 1e-4)
    noise.accel = np.full(3, 1e-3)
    noise.gyro_rw = np.full(3, 1e-6)
    noise.accel_rw = np.full(3, 1e-5)
    dt = 0.01

    for _ in range(100):
        filt.propagate(w_meas, a_meas, g_vec, noise, dt)

    print("After 1s of propagation:")
    print("  attitude:", filt.attitude())
    print("  position:", filt.position())
    print("  velocity:", filt.velocity())

    # 5. Position update: a GNSS-like fix pulls the estimate toward a known
    # world-frame position.
    R_pos = 0.01 * np.eye(3)
    filt.update_position(np.array([0.1, -0.05, 0.0]), R_pos)

    # 6. DVL update: a body-frame velocity measurement pulls the estimate
    # toward the (still at-rest) true body velocity.
    R_dvl = 0.001 * np.eye(3)
    filt.update_dvl(np.array([0.0, 0.0, 0.0]), R_dvl)

    print("\nAfter position + DVL updates:")
    print("  attitude:", filt.attitude())
    print("  position:", filt.position())
    print("  velocity:", filt.velocity())
    print("  error covariance diagonal:\n", np.diag(filt.errorCovariance()))


if __name__ == "__main__":
    main()
