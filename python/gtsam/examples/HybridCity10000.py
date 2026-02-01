"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Script for running hybrid estimator on the City10000 dataset.

Author: Varun Agrawal
"""

import argparse
import time

import numpy as np
from gtsam.symbol_shorthand import L, M, X
from matplotlib import pyplot as plt

import gtsam
from gtsam import (BetweenFactorPose2, HybridNonlinearFactor,
                   HybridNonlinearFactorGraph, HybridSmoother, HybridValues,
                   Pose2, PriorFactorPose2, Values)


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_file",
                        help="The path to the City10000 data file",
                        default="T1_city10000_04.txt")
    parser.add_argument(
        "--max_loop_count",
        "-l",
        type=int,
        default=10000,
        help="The maximum number of loops to run over the dataset")
    parser.add_argument(
        "--update_frequency",
        "-u",
        type=int,
        default=3,
        help="After how many steps to run the smoother update.")
    parser.add_argument(
        "--max_num_hypotheses",
        "-m",
        type=int,
        default=10,
        help="The maximum number of hypotheses to keep at any time.")
    parser.add_argument(
        "--plot_hypotheses",
        "-p",
        action="store_true",
        help="Plot all hypotheses. NOTE: This is exponential, use with caution."
    )
    return parser.parse_args()


# Noise models
open_loop_model = gtsam.noiseModel.Diagonal.Sigmas(np.ones(3) * 10)
open_loop_constant = open_loop_model.negLogConstant()

prior_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
    np.asarray([0.0001, 0.0001, 0.0001]))

pose_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
    np.asarray([1.0 / 20.0, 1.0 / 20.0, 1.0 / 100.0]))
pose_noise_constant = pose_noise_model.negLogConstant()


class City10000Dataset:
    """Class representing the City10000 dataset."""

    def __init__(self, filename):
        self.filename_ = filename
        try:
            self.f_ = open(self.filename_, 'r')
        except OSError:
            print(f"Failed to open file: {self.filename_}")

    def __del__(self):
        self.f_.close()

    def read_line(self, line: str, delimiter: str = " "):
        """Read a `line` from the dataset, separated by the `delimiter`."""
        return line.split(delimiter)

    def parse_line(self,
                   line: str) -> tuple[list[Pose2], tuple[int, int], bool]:
        """Parse line from file"""
        parts = self.read_line(line)

        key_s = int(parts[1])
        key_t = int(parts[3])

        is_ambiguous_loop = bool(int(parts[4]))

        num_measurements = int(parts[5])
        pose_array = [Pose2()] * num_measurements

        for i in range(num_measurements):
            x = float(parts[6 + 3 * i])
            y = float(parts[7 + 3 * i])
            rad = float(parts[8 + 3 * i])
            pose_array[i] = Pose2(x, y, rad)

        return pose_array, (key_s, key_t), is_ambiguous_loop

    def next(self):
        """Read and parse the next line."""
        line = self.f_.readline()
        if line:
            return self.parse_line(line)
        else:
            return None, None, None


def plot_all_results(ground_truth,
                     all_results,
                     iters=0,
                     estimate_color=(0.1, 0.1, 0.9, 0.4),
                     estimate_label="Hybrid Factor Graphs",
                     text="",
                     filename="city10000_results.svg"):
    """Plot the City10000 estimates against the ground truth.

    Args:
        ground_truth: The ground truth trajectory as xy values.
        all_results (List[Tuple(np.ndarray, str)]): All the estimates trajectory as xy values,
            as well as assginment strings.
        estimate_color (tuple, optional): The color to use for the graph of estimates.
            Defaults to (0.1, 0.1, 0.9, 0.4).
        estimate_label (str, optional): Label for the estimates, used in the legend.
            Defaults to "Hybrid Factor Graphs".
    """
    if len(all_results) == 1:
        fig, axes = plt.subplots(1, 1)
        axes = [axes]
    else:
        fig, axes = plt.subplots(int(np.ceil(len(all_results) / 2)), 2)
        axes = axes.flatten()

    for i, (estimates, s, prob) in enumerate(all_results):
        ax = axes[i]
        ax.axis('equal')
        ax.axis((-75.0, 100.0, -75.0, 75.0))

        gt = ground_truth[:estimates.shape[0]]
        ax.plot(gt[:, 0],
                gt[:, 1],
                '--',
                linewidth=1,
                color=(0.1, 0.7, 0.1, 0.5),
                label="Ground Truth")
        ax.plot(estimates[:, 0],
                estimates[:, 1],
                '-',
                linewidth=1,
                color=estimate_color,
                label=estimate_label)
        # ax.legend()
        ax.set_title(f"P={prob:.3f}\n{s}", fontdict={'fontsize': 10})

    fig.suptitle(f"After {iters} iterations")

    num_chunks = int(np.ceil(len(text) / 90))
    text = "\n".join(text[i * 60:(i + 1) * 60] for i in range(num_chunks))
    fig.text(0.5,
             0.015,
             s=text,
             wrap=True,
             horizontalalignment='center',
             fontsize=12)

    fig.savefig(filename, format="svg")


class Experiment:
    """Experiment Class"""

    def __init__(self,
                 filename: str,
                 marginal_threshold: float = 0.9999,
                 max_loop_count: int = 150,
                 update_frequency: int = 3,
                 max_num_hypotheses: int = 10,
                 relinearization_frequency: int = 10,
                 plot_hypotheses: bool = False):
        self.dataset_ = City10000Dataset(filename)
        self.max_loop_count = max_loop_count
        self.update_frequency = update_frequency
        self.max_num_hypotheses = max_num_hypotheses
        self.relinearization_frequency = relinearization_frequency

        self.smoother_ = HybridSmoother(marginal_threshold)
        self.new_factors_ = HybridNonlinearFactorGraph()
        self.initial_ = Values()

        self.plot_hypotheses = plot_hypotheses

    def hybrid_loop_closure_factor(self, loop_counter, key_s, key_t,
                                   measurement: Pose2):
        """
        Create a hybrid loop closure factor where
        0 - loose noise model and 1 - loop noise model.
        """
        l = (L(loop_counter), 2)
        f0 = BetweenFactorPose2(X(key_s), X(key_t), measurement,
                                open_loop_model)
        f1 = BetweenFactorPose2(X(key_s), X(key_t), measurement,
                                pose_noise_model)
        factors = [(f0, open_loop_constant), (f1, pose_noise_constant)]
        mixture_factor = HybridNonlinearFactor(l, factors)
        return mixture_factor

    def hybrid_odometry_factor(self, key_s, key_t, m,
                               pose_array) -> HybridNonlinearFactor:
        """Create hybrid odometry factor with discrete measurement choices."""
        f0 = BetweenFactorPose2(X(key_s), X(key_t), pose_array[0],
                                pose_noise_model)
        f1 = BetweenFactorPose2(X(key_s), X(key_t), pose_array[1],
                                pose_noise_model)

        factors = [(f0, pose_noise_constant), (f1, pose_noise_constant)]
        mixture_factor = HybridNonlinearFactor(m, factors)

        return mixture_factor

    def smoother_update(self, max_num_hypotheses) -> float:
        """Perform smoother update and optimize the graph."""
        print(f"Smoother update: {self.new_factors_.size()}")
        before_update = time.time()
        self.smoother_.update(self.new_factors_, self.initial_,
                              max_num_hypotheses)
        self.new_factors_.resize(0)
        after_update = time.time()
        return after_update - before_update

    def reinitialize(self) -> float:
        """Re-linearize, solve ALL, and re-initialize smoother."""
        print(f"================= Re-Initialize: {self.smoother_.allFactors().size()}")
        before_update = time.time()
        self.smoother_.relinearize()
        self.initial_ = self.smoother_.linearizationPoint()
        after_update = time.time()
        print(f"Took {after_update - before_update} seconds.")
        return after_update - before_update

    def run(self):
        """Run the main experiment with a given max_loop_count."""
        # Initialize local variables
        discrete_count = 0
        index = 0
        loop_count = 0
        update_count = 0

        time_list = []  #list[(int, float)]

        # Set up initial prior
        priorPose = Pose2(0, 0, 0)
        self.initial_.insert(X(0), priorPose)
        self.new_factors_.push_back(
            PriorFactorPose2(X(0), priorPose, prior_noise_model))

        # Initial update
        update_time = self.smoother_update(self.max_num_hypotheses)
        smoother_update_times = []  # list[(int, float)]
        smoother_update_times.append((index, update_time))

        # Flag to decide whether to run smoother update
        number_of_hybrid_factors = 0

        # Start main loop
        result = Values()
        start_time = time.time()

        while index < self.max_loop_count:
            pose_array, keys, is_ambiguous_loop = self.dataset_.next()
            if pose_array is None:
                break
            key_s = keys[0]
            key_t = keys[1]

            num_measurements = len(pose_array)

            # Take the first one as the initial estimate
            # odom_pose = pose_array[np.random.choice(num_measurements)]
            odom_pose = pose_array[0]
            if key_s == key_t - 1:
                # Odometry factor
                if num_measurements > 1:
                    # Add hybrid factor
                    m = (M(discrete_count), num_measurements)
                    mixture_factor = self.hybrid_odometry_factor(
                        key_s, key_t, m, pose_array)
                    self.new_factors_.push_back(mixture_factor)

                    discrete_count += 1
                    number_of_hybrid_factors += 1
                    print(f"mixture_factor: {key_s} {key_t}")
                else:
                    self.new_factors_.push_back(
                        BetweenFactorPose2(X(key_s), X(key_t), odom_pose,
                                           pose_noise_model))

                # Insert next pose initial guess
                self.initial_.insert(
                    X(key_t),
                    self.initial_.atPose2(X(key_s)) * odom_pose)
            else:
                # Loop closure
                if is_ambiguous_loop:
                    loop_factor = self.hybrid_loop_closure_factor(
                        loop_count, key_s, key_t, odom_pose)

                else:
                    loop_factor = BetweenFactorPose2(X(key_s), X(key_t),
                                                     odom_pose,
                                                     pose_noise_model)

                # print loop closure event keys:
                print(f"Loop closure: {key_s} {key_t}")
                self.new_factors_.push_back(loop_factor)
                number_of_hybrid_factors += 1
                loop_count += 1

            if number_of_hybrid_factors >= self.update_frequency:
                update_time = self.smoother_update(self.max_num_hypotheses)
                smoother_update_times.append((index, update_time))
                number_of_hybrid_factors = 0
                update_count += 1

                if update_count % self.relinearization_frequency == 0:
                    self.reinitialize()

            #  Record timing for odometry edges only
            if key_s == key_t - 1:
                cur_time = time.time()
                time_list.append(cur_time - start_time)

            # Print some status every 100 steps
            if index % 100 == 0:
                print(f"Index: {index}")

                if len(time_list) != 0:
                    print(f"Accumulate time: {time_list[-1]} seconds")

            index += 1

        # Final update
        update_time = self.smoother_update(self.max_num_hypotheses)
        smoother_update_times.append((index, update_time))

        # Final optimize
        delta = self.smoother_.optimize()

        result.insert_or_assign(self.initial_.retract(delta.continuous()))

        print(f"Final error: {self.smoother_.hybridBayesNet().error(delta)}")

        end_time = time.time()
        total_time = end_time - start_time
        print(f"Total time: {total_time} seconds")

        # self.save_results(result, key_t + 1, time_list)

        if self.plot_hypotheses:
            # Get all the discrete values
            discrete_keys = gtsam.DiscreteKeys()
            for key in delta.discrete().keys():
                # TODO Get cardinality from DiscreteFactor
                discrete_keys.push_back((key, 2))
            print("plotting all hypotheses")
            self.plot_all_hypotheses(discrete_keys, key_t + 1, index)

    def plot_all_hypotheses(self, discrete_keys, num_poses, num_iters=0):
        """Plot all possible hypotheses."""

        # Get ground truth
        gt = np.loadtxt(gtsam.findExampleDataFile("ISAM2_GT_city10000.txt"),
                        delimiter=" ")

        dkeys = gtsam.DiscreteKeys()
        for i in range(discrete_keys.size()):
            key, cardinality = discrete_keys.at(i)
            if key not in self.smoother_.fixedValues().keys():
                dkeys.push_back((key, cardinality))
        fixed_values_str = " ".join(
            f"{gtsam.DefaultKeyFormatter(k)}:{v}"
            for k, v in self.smoother_.fixedValues().items())

        all_assignments = gtsam.cartesianProduct(dkeys)

        all_results = []
        for assignment in all_assignments:
            result = gtsam.Values()
            gbn = self.smoother_.hybridBayesNet().choose(assignment)

            # Check to see if the GBN has any nullptrs, if it does it is null overall
            is_invalid_gbn = False
            for i in range(gbn.size()):
                if gbn.at(i) is None:
                    is_invalid_gbn = True
                    break
            if is_invalid_gbn:
                continue

            delta = self.smoother_.hybridBayesNet().optimize(assignment)
            result.insert_or_assign(self.initial_.retract(delta))

            poses = np.zeros((num_poses, 3))
            for i in range(num_poses):
                pose = result.atPose2(X(i))
                poses[i] = np.asarray((pose.x(), pose.y(), pose.theta()))

            assignment_string = " ".join([
                f"{gtsam.DefaultKeyFormatter(k)}={v}"
                for k, v in assignment.items()
            ])

            conditional = self.smoother_.hybridBayesNet().at(
                self.smoother_.hybridBayesNet().size() - 1).asDiscrete()
            discrete_values = self.smoother_.fixedValues()
            for k, v in assignment.items():
                discrete_values[k] = v

            if conditional is None:
                probability = 1.0
            else:
                probability = conditional.evaluate(discrete_values)

            all_results.append((poses, assignment_string, probability))

        plot_all_results(gt,
                         all_results,
                         iters=num_iters,
                         text=fixed_values_str,
                         filename=f"city10000_results_{num_iters}.svg")

    def save_results(self, result, final_key, time_list):
        """Save results to file."""
        # Write results to file
        self.write_result(result, final_key, "Hybrid_City10000.txt")

        # Write timing info to file
        self.write_timing_info(time_list=time_list)

    def write_result(self, result, num_poses, filename="Hybrid_city10000.txt"):
        """
        Write the result of optimization to file.

        Args:
            result (Values): he Values object with the final result.
            num_poses (int): The number of poses to write to the file.
            filename (str): The file name to save the result to.
        """
        with open(filename, 'w') as outfile:

            for i in range(num_poses):
                out_pose = result.atPose2(X(i))
                outfile.write(
                    f"{out_pose.x()} {out_pose.y()} {out_pose.theta()}\n")

        print(f"Output written to {filename}")

    def write_timing_info(self,
                          time_list,
                          time_filename="Hybrid_City10000_time.txt"):
        """Log all the timing information to a file"""

        with open(time_filename, 'w') as out_file_time:

            for acc_time in time_list:
                out_file_time.write(f"{acc_time}\n")

            print(f"Output {time_filename} file.")


def main():
    """Main runner"""
    args = parse_arguments()

    experiment = Experiment(gtsam.findExampleDataFile(args.data_file),
                            max_loop_count=args.max_loop_count,
                            update_frequency=args.update_frequency,
                            max_num_hypotheses=args.max_num_hypotheses,
                            plot_hypotheses=args.plot_hypotheses)
    experiment.run()


if __name__ == "__main__":
    main()
