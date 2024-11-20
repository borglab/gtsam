"""
  Compare several methods for optimizing the view-graph.
  We measure the distance from the ground truth in terms of the norm of 
  local coordinates (geodesic distance) on the F-manifold. 
  We also plot the final error of the optimization.
  
  Author: Frank Dellaert (with heavy assist from ChatGPT)
  Date: October 2024
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
from gtsam.examples import SFMdata

import gtsam
from gtsam import (
    Cal3f,
    EdgeKey,
    EssentialMatrix,
    FundamentalMatrix,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    PinholeCameraCal3f,
    SimpleFundamentalMatrix,
    Values,
)

# For symbol shorthand (e.g., K(0), K(1))
K = gtsam.symbol_shorthand.K

# Methods to compare
methods = ["SimpleF", "Fundamental", "Essential+Ks", "Calibrated"]


# Formatter function for printing keys
def formatter(key):
    sym = gtsam.Symbol(key)
    if sym.chr() == ord("k"):
        return f"k{sym.index()}"
    else:
        edge = EdgeKey(key)
        return f"({edge.i()},{edge.j()})"


def simulate_geometry(num_cameras, rng, num_random_points=12):
    """simulate geometry (points and poses)"""
    # Define the camera calibration parameters
    cal = Cal3f(50.0, 50.0, 50.0)

    # Create the set of 8 ground-truth landmarks
    points = SFMdata.createPoints()

    # Create extra random points in the -10,10 cube around the origin
    extra_points = rng.uniform(-10, 10, (num_random_points, 3))
    points.extend([gtsam.Point3(p) for p in extra_points])

    # Create the set of ground-truth poses
    poses = SFMdata.posesOnCircle(num_cameras, 30)

    return points, poses, cal


def simulate_data(points, poses, cal, rng, noise_std):
    """Simulate measurements from each camera pose"""
    measurements = [[None for _ in points] for _ in poses]
    for i, pose in enumerate(poses):
        camera = PinholeCameraCal3f(pose, cal)
        for j, point in enumerate(points):
            projection = camera.project(point)
            noise = rng.normal(0, noise_std, size=2)
            measurements[i][j] = projection + noise

    return measurements


# Function to compute ground truth matrices
def compute_ground_truth(method, poses, cal):
    E1 = EssentialMatrix.FromPose3(poses[0].between(poses[1]))
    E2 = EssentialMatrix.FromPose3(poses[0].between(poses[2]))
    F1 = FundamentalMatrix(cal.K(), E1, cal.K())
    F2 = FundamentalMatrix(cal.K(), E2, cal.K())
    if method == "Fundamental":
        return F1, F2
    elif method == "SimpleF":
        f = cal.fx()
        c = cal.principalPoint()
        SF1 = SimpleFundamentalMatrix(E1, f, f, c, c)
        SF2 = SimpleFundamentalMatrix(E2, f, f, c, c)
        return SF1, SF2
    elif method == "Essential+Ks" or method == "Calibrated":
        return E1, E2
    else:
        raise ValueError(f"Unknown method {method}")


def build_factor_graph(method, num_cameras, measurements, cal):
    """build the factor graph"""
    graph = NonlinearFactorGraph()

    if method == "Fundamental":
        FactorClass = gtsam.TransferFactorFundamentalMatrix
    elif method == "SimpleF":
        FactorClass = gtsam.TransferFactorSimpleFundamentalMatrix
    elif method == "Essential+Ks":
        FactorClass = gtsam.EssentialTransferFactorKCal3f
        # add priors on all calibrations:
        for i in range(num_cameras):
            model = gtsam.noiseModel.Isotropic.Sigma(1, 10.0)
            graph.addPriorCal3f(K(i), cal, model)
    elif method == "Calibrated":
        FactorClass = gtsam.EssentialTransferFactorCal3f
        # No priors on calibration needed
    else:
        raise ValueError(f"Unknown method {method}")

    z = measurements  # shorthand

    for a in range(num_cameras):
        b = (a + 1) % num_cameras  # Next camera
        c = (a + 2) % num_cameras  # Camera after next

        # Vectors to collect tuples for each factor
        tuples1 = []
        tuples2 = []
        tuples3 = []

        # Collect data for the three factors
        for j in range(len(measurements[0])):
            tuples1.append((z[a][j], z[b][j], z[c][j]))
            tuples2.append((z[a][j], z[c][j], z[b][j]))
            tuples3.append((z[c][j], z[b][j], z[a][j]))

        # Add transfer factors between views a, b, and c.
        if method in ["Calibrated"]:
            graph.add(FactorClass(EdgeKey(a, c), EdgeKey(b, c), tuples1, cal))
            graph.add(FactorClass(EdgeKey(a, b), EdgeKey(b, c), tuples2, cal))
            graph.add(FactorClass(EdgeKey(a, c), EdgeKey(a, b), tuples3, cal))
        else:
            graph.add(FactorClass(EdgeKey(a, c), EdgeKey(b, c), tuples1))
            graph.add(FactorClass(EdgeKey(a, b), EdgeKey(b, c), tuples2))
            graph.add(FactorClass(EdgeKey(a, c), EdgeKey(a, b), tuples3))

    return graph


def get_initial_estimate(method, num_cameras, ground_truth, cal):
    """get initial estimate for method"""
    initialEstimate = Values()
    total_dimension = 0

    if method in ["Fundamental", "SimpleF"]:
        F1, F2 = ground_truth
        for a in range(num_cameras):
            b = (a + 1) % num_cameras  # Next camera
            c = (a + 2) % num_cameras  # Camera after next
            initialEstimate.insert(EdgeKey(a, b).key(), F1)
            initialEstimate.insert(EdgeKey(a, c).key(), F2)
            total_dimension += F1.dim() + F2.dim()
    elif method in ["Essential+Ks", "Calibrated"]:
        E1, E2 = ground_truth
        for a in range(num_cameras):
            b = (a + 1) % num_cameras  # Next camera
            c = (a + 2) % num_cameras  # Camera after next
            initialEstimate.insert(EdgeKey(a, b).key(), E1)
            initialEstimate.insert(EdgeKey(a, c).key(), E2)
            total_dimension += E1.dim() + E2.dim()
    else:
        raise ValueError(f"Unknown method {method}")

    if method == "Essential+Ks":
        # Insert initial calibrations
        for i in range(num_cameras):
            initialEstimate.insert(K(i), cal)
            total_dimension += cal.dim()

    print(f"Total dimension of the problem: {total_dimension}")
    return initialEstimate


def optimize(graph, initialEstimate, method):
    """optimize the graph"""
    params = LevenbergMarquardtParams()
    params.setlambdaInitial(1e10)  # Initialize lambda to a high value
    params.setlambdaUpperBound(1e10)
    # params.setAbsoluteErrorTol(0.1)
    params.setVerbosityLM("SUMMARY")
    optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, params)
    result = optimizer.optimize()
    iterations = optimizer.iterations()
    return result, iterations


def compute_distances(method, result, ground_truth, num_cameras, cal):
    """Compute geodesic distances from ground truth"""
    distances = []

    F1, F2 = ground_truth["Fundamental"]

    for a in range(num_cameras):
        b = (a + 1) % num_cameras
        c = (a + 2) % num_cameras
        key_ab = EdgeKey(a, b).key()
        key_ac = EdgeKey(a, c).key()

        if method in ["Essential+Ks", "Calibrated"]:
            E_est_ab = result.atEssentialMatrix(key_ab)
            E_est_ac = result.atEssentialMatrix(key_ac)

        # Compute estimated FundamentalMatrices
        if method == "Fundamental":
            F_est_ab = result.atFundamentalMatrix(key_ab)
            F_est_ac = result.atFundamentalMatrix(key_ac)
        elif method == "SimpleF":
            SF_est_ab = result.atSimpleFundamentalMatrix(key_ab).matrix()
            SF_est_ac = result.atSimpleFundamentalMatrix(key_ac).matrix()
            F_est_ab = FundamentalMatrix(SF_est_ab)
            F_est_ac = FundamentalMatrix(SF_est_ac)
        elif method == "Essential+Ks":
            # Retrieve calibrations from result:
            cal_a = result.atCal3f(K(a))
            cal_b = result.atCal3f(K(b))
            cal_c = result.atCal3f(K(c))

            # Convert estimated EssentialMatrices to FundamentalMatrices
            F_est_ab = FundamentalMatrix(cal_a.K(), E_est_ab, cal_b.K())
            F_est_ac = FundamentalMatrix(cal_a.K(), E_est_ac, cal_c.K())
        elif method == "Calibrated":
            # Use ground truth calibration
            F_est_ab = FundamentalMatrix(cal.K(), E_est_ab, cal.K())
            F_est_ac = FundamentalMatrix(cal.K(), E_est_ac, cal.K())
        else:
            raise ValueError(f"Unknown method {method}")

        # Compute local coordinates (geodesic distance on the F-manifold)
        dist_ab = np.linalg.norm(F1.localCoordinates(F_est_ab))
        dist_ac = np.linalg.norm(F2.localCoordinates(F_est_ac))
        distances.append(dist_ab)
        distances.append(dist_ac)

    return distances


def plot_results(results):
    """plot results"""
    methods = list(results.keys())
    final_errors = [results[method]["final_error"] for method in methods]
    distances = [results[method]["distances"] for method in methods]
    iterations = [results[method]["iterations"] for method in methods]

    fig, ax1 = plt.subplots()

    color = "tab:red"
    ax1.set_xlabel("Method")
    ax1.set_ylabel("Median Error (log scale)", color=color)
    ax1.set_yscale("log")
    ax1.bar(methods, final_errors, color=color, alpha=0.6)
    ax1.tick_params(axis="y", labelcolor=color)

    ax2 = ax1.twinx()
    color = "tab:blue"
    ax2.set_ylabel("Median Geodesic Distance", color=color)
    ax2.plot(methods, distances, color=color, marker="o", linestyle="-")
    ax2.tick_params(axis="y", labelcolor=color)

    # Annotate the blue data points with the average number of iterations
    for i, method in enumerate(methods):
        ax2.annotate(
            f"{iterations[i]:.1f}",
            (i, distances[i]),
            textcoords="offset points",
            xytext=(0, 10),
            ha="center",
            color=color,
        )

    plt.title("Comparison of Methods (Labels show avg iterations)")
    fig.tight_layout()
    plt.show()


# Main function
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Compare Fundamental and Essential Matrix Methods")
    parser.add_argument("--num_cameras", type=int, default=4, help="Number of cameras (default: 4)")
    parser.add_argument("--num_extra_points", type=int, default=12, help="Number of extra random points (default: 12)")
    parser.add_argument("--num_trials", type=int, default=5, help="Number of trials (default: 5)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed (default: 42)")
    parser.add_argument("--noise_std", type=float, default=0.5, help="Standard deviation of noise (default: 0.5)")
    args = parser.parse_args()

    # Initialize the random number generator
    rng = np.random.default_rng(seed=args.seed)

    # Initialize results dictionary
    results = {method: {"distances": [], "final_error": [], "iterations": []} for method in methods}

    # Simulate geometry
    points, poses, cal = simulate_geometry(args.num_cameras, rng, args.num_extra_points)

    # Compute ground truth matrices
    ground_truth = {method: compute_ground_truth(method, poses, cal) for method in methods}

    # Get initial estimates
    initial_estimate: dict[Values] = {
        method: get_initial_estimate(method, args.num_cameras, ground_truth[method], cal) for method in methods
    }
    simple_f_result: Values = Values()

    for trial in range(args.num_trials):
        print(f"\nTrial {trial + 1}/{args.num_trials}")

        # Simulate data
        measurements = simulate_data(points, poses, cal, rng, args.noise_std)

        for method in methods:
            print(f"\nRunning method: {method}")

            # Build the factor graph
            graph = build_factor_graph(method, args.num_cameras, measurements, cal)

            # For F, initialize from SimpleF:
            if method == "Fundamental":
                initial_estimate[method] = simple_f_result

            # Optimize the graph
            result, iterations = optimize(graph, initial_estimate[method], method)

            # Store SimpleF result as a set of FundamentalMatrices
            if method == "SimpleF":
                simple_f_result = Values()
                for a in range(args.num_cameras):
                    b = (a + 1) % args.num_cameras  # Next camera
                    c = (a + 2) % args.num_cameras  # Camera after next
                    key_ab = EdgeKey(a, b).key()
                    key_ac = EdgeKey(a, c).key()
                    F1 = result.atSimpleFundamentalMatrix(key_ab).matrix()
                    F2 = result.atSimpleFundamentalMatrix(key_ac).matrix()
                    simple_f_result.insert(key_ab, FundamentalMatrix(F1))
                    simple_f_result.insert(key_ac, FundamentalMatrix(F2))

            # Compute distances from ground truth
            distances = compute_distances(method, result, ground_truth, args.num_cameras, cal)

            # Compute final error
            final_error = graph.error(result)

            # Store results
            results[method]["distances"].extend(distances)
            results[method]["final_error"].append(final_error)
            results[method]["iterations"].append(iterations)

            print(f"Method: {method}")
            print(f"Final Error: {final_error:.3f}")
            print(f"Mean Geodesic Distance: {np.mean(distances):.3f}")
            print(f"Number of Iterations: {iterations}\n")

    # Average results over trials
    for method in methods:
        results[method]["final_error"] = np.median(results[method]["final_error"])
        results[method]["distances"] = np.median(results[method]["distances"])
        results[method]["iterations"] = np.median(results[method]["iterations"])

    # Plot results
    plot_results(results)


if __name__ == "__main__":
    main()
