"""
Generates a CSV file with ground-truth trajectory data for testing a
DiscreteScenario in GTSAM.

The trajectory is a simple horizontal circle at a constant height and speed.

CSV Format:
timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz,omegax,omegay,omegaz,ax,ay,az
"""
import numpy as np
import csv
from gtsam import Rot3

# --- Trajectory Parameters ---
RADIUS = 5.0  # meters
HEIGHT = 1.0  # meters
OMEGA_Z = 0.2  # rad/s (angular velocity around Z-axis)
DURATION = 20.0  # seconds
FREQUENCY = 100  # Hz
FILENAME = "trajectory.csv"


def generate_data():
    """Generates and saves the trajectory data."""

    # Time vector
    t = np.arange(0.0, DURATION, 1.0 / FREQUENCY)

    # Angle at each time step
    angle = OMEGA_Z * t

    # Position (p_n) in navigation frame
    px = RADIUS * np.cos(angle)
    py = RADIUS * np.sin(angle)
    pz = np.full_like(t, HEIGHT)

    # Velocity (v_n) in navigation frame (derivative of position)
    speed = RADIUS * OMEGA_Z
    vx = -speed * np.sin(angle)
    vy = speed * np.cos(angle)
    vz = np.zeros_like(t)

    # Acceleration (a_n) in navigation frame (derivative of velocity)
    # This is the centripetal acceleration.
    ax = -speed * OMEGA_Z * np.cos(angle)
    ay = -speed * OMEGA_Z * np.sin(angle)
    az = np.zeros_like(t)

    # Angular velocity (omega_b) in the body frame
    # The body is only rotating around its z-axis to face forward.
    omegax = np.zeros_like(t)
    omegay = np.zeros_like(t)
    omegaz = np.full_like(t, OMEGA_Z)

    # Orientation (quaternion q_n_b)
    # We use gtsam.Rot3 to easily get the quaternion
    quaternions = [Rot3.Yaw(a).toQuaternion() for a in angle]

    # --- Write to CSV ---
    header = [
        'timestamp',
        'px',
        'py',
        'pz',  # Position
        'qw',
        'qx',
        'qy',
        'qz',  # Quaternion (orientation)
        'vx',
        'vy',
        'vz',  # Velocity (nav frame)
        'omegax',
        'omegay',
        'omegaz',  # Angular velocity (body frame)
        'ax',
        'ay',
        'az'  # Acceleration (nav frame)
    ]

    with open(FILENAME, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for i in range(len(t)):
            q = quaternions[i]
            row = [
                t[i], px[i], py[i], pz[i],
                q.w(),
                q.x(),
                q.y(),
                q.z(), vx[i], vy[i], vz[i], omegax[i], omegay[i], omegaz[i],
                ax[i], ay[i], az[i]
            ]
            writer.writerow(row)

    print(f"Successfully generated trajectory data at '{FILENAME}'")


if __name__ == "__main__":
    generate_data()
