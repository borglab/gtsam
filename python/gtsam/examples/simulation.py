# simulation.py
import numpy as np

import gtsam


def generate_simulation_data(
    num_landmarks,
    world_size,
    robot_radius,
    robot_angular_vel,
    num_steps,
    dt,
    odometry_noise_model,
    measurement_noise_model,
    max_sensor_range,
    X,  # Symbol generator function
    L,  # Symbol generator function
    odom_seed=42,
    meas_seed=42,
    landmark_seed=42,
):
    """Generates ground truth and simulated measurements for SLAM.

    Args:
        num_landmarks: Number of landmarks to generate.
        world_size: Size of the square world environment.
        robot_radius: Radius of the robot's circular path.
        robot_angular_vel: Angular velocity of the robot (rad/step).
        num_steps: Number of simulation steps.
        dt: Time step duration.
        odometry_noise_model: GTSAM noise model for odometry.
        measurement_noise_model: GTSAM noise model for bearing-range.
        max_sensor_range: Maximum range of the bearing-range sensor.
        X: GTSAM symbol shorthand function for poses.
        L: GTSAM symbol shorthand function for landmarks.
        odom_seed: Random seed for odometry noise.
        meas_seed: Random seed for measurement noise.
        landmark_seed: Random seed for landmark placement.

    Returns:
        tuple: Contains:
            - landmarks_gt_dict (dict): L(i) -> gtsam.Point2 ground truth.
            - poses_gt (list): List of gtsam.Pose2 ground truth poses.
            - odometry_measurements (list): List of noisy gtsam.Pose2 odometry.
            - measurements_sim (list): List of lists, measurements_sim[k] contains
                                       tuples (L(lm_id), bearing, range) for step k.
            - landmarks_gt_array (np.array): 2xN numpy array of landmark positions.
    """
    np.random.seed(landmark_seed)
    odometry_noise_sampler = gtsam.Sampler(odometry_noise_model, odom_seed)
    measurement_noise_sampler = gtsam.Sampler(measurement_noise_model, meas_seed)

    # 1. Ground Truth Landmarks
    landmarks_gt_array = (np.random.rand(2, num_landmarks) - 0.5) * world_size
    landmarks_gt_dict = {
        L(i): gtsam.Point2(landmarks_gt_array[:, i]) for i in range(num_landmarks)
    }

    # 2. Ground Truth Robot Path
    poses_gt = []
    current_pose_gt = gtsam.Pose2(robot_radius, 0, np.pi / 2)  # Start on circle edge
    poses_gt.append(current_pose_gt)

    for _ in range(num_steps):
        delta_theta = robot_angular_vel * dt
        arc_length = robot_angular_vel * robot_radius * dt
        motion_command = gtsam.Pose2(arc_length, 0, delta_theta)
        current_pose_gt = current_pose_gt.compose(motion_command)
        poses_gt.append(current_pose_gt)

    # 3. Simulate Noisy Odometry Measurements
    odometry_measurements = []
    for k in range(num_steps):
        pose_k = poses_gt[k]
        pose_k1 = poses_gt[k + 1]
        true_odom = pose_k.between(pose_k1)

        # Sample noise directly for Pose2 composition (approximate)
        odom_noise_vec = odometry_noise_sampler.sample()
        noisy_odom = true_odom.compose(
            gtsam.Pose2(odom_noise_vec[0], odom_noise_vec[1], odom_noise_vec[2])
        )
        odometry_measurements.append(noisy_odom)

    # 4. Simulate Noisy Bearing-Range Measurements
    measurements_sim = [[] for _ in range(num_steps + 1)]
    for k in range(num_steps + 1):
        robot_pose = poses_gt[k]
        for lm_id in range(num_landmarks):
            lm_gt_pt = landmarks_gt_dict[L(lm_id)]
            try:
                measurement_factor = gtsam.BearingRangeFactor2D(
                    X(k),
                    L(lm_id),
                    robot_pose.bearing(lm_gt_pt),
                    robot_pose.range(lm_gt_pt),
                    measurement_noise_model,
                )
                true_range = measurement_factor.measured().range()
                true_bearing = measurement_factor.measured().bearing()

                # Check sensor limits (range and Field of View - e.g. +/- 45 degrees)
                if (
                    true_range <= max_sensor_range
                    and abs(true_bearing.theta()) < np.pi / 2
                ):
                    # Sample noise
                    noise_vec = measurement_noise_sampler.sample()
                    noisy_bearing = true_bearing.retract(
                        np.array([noise_vec[0]])
                    )  # Retract on SO(2)
                    noisy_range = true_range + noise_vec[1]

                    if noisy_range > 0:  # Ensure range is positive
                        measurements_sim[k].append(
                            (L(lm_id), noisy_bearing, noisy_range)
                        )
            except Exception as e:
                # Catch potential errors like point being too close to the pose
                # print(f"Sim Warning at step {k}, landmark {lm_id}: {e}") # Can be verbose
                pass

    print(f"Simulation Generated: {num_landmarks} landmarks.")
    print(
        f"Simulation Generated: {num_steps + 1} ground truth poses and {num_steps} odometry measurements."
    )
    num_meas_total = sum(len(m_list) for m_list in measurements_sim)
    print(f"Simulation Generated: {num_meas_total} bearing-range measurements.")

    return (
        landmarks_gt_dict,
        poses_gt,
        odometry_measurements,
        measurements_sim,
        landmarks_gt_array,
    )
