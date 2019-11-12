"""
Process timing results from timeShonanAveraging
"""

import xlrd
import numpy as np
import matplotlib.pyplot as plt

def make_time_plot(timings):
    """Plot timings given as a dictionary {name:time}."""
    for name,time in timings.items():
        plt.plot(p_value, time[:5], label="tinygrid vertex = 9, edge = 11")
    plt.xlabel("SO(P), p number")
    plt.ylabel("Time used to optimize \ seconds")
    plt.title('The relationship between the p and time used for optimization', fontsize=12)
    plt.legend(loc='upper right', frameon=True)
    plt.show()
    plt.savefig("p_time_6")

def make_combined_plot(p_values, times, costs):
    """ Make a plot that combines timing and SO(3) cost.
        Arguments:
            p_values: list of p-values (int)
            times: list of timings (seconds)
            costs: list of costs (double)
    """
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(p_values, times, label="time")
    ax1.set_ylabel('Time used to optimize \ seconds')
    ax1.set_xlabel('p_value')
    ax2 = ax1.twinx()
    ax2.plot(p_values, costs, 'r', label="cost")
    ax2.set_ylabel('Cost at SO(P) form')
    ax2.set_xlabel('p_value')
    ax2.set_xticks(p_values)
    plt.title('cubicle vertex = 5750, edge = 16869', fontsize=12)
    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    plt.show()

# Process arguments and call plot function
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("csv_file")
args = parser.parse_args()
print(args.csv_file)

import csv

# Parse CSV file
p_values, times, costs = [],[],[]
with open(args.csv_file) as csvfile:
    reader = csv.reader(csvfile, delimiter='\t')
    for row in reader:
        print(row)
        p_values.append(int(row[0]))
        times.append(float(row[1]))
        costs.append(float(row[3]))

#plot
make_combined_plot(p_values, times, costs)