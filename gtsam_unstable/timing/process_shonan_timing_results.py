"""
Process timing results from timeShonanAveraging
"""

import xlrd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import heapq
from collections import Counter

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

def make_combined_plot(name, p_values, times, costs, min_cost_range=10):
    """ Make a plot that combines timing and SO(3) cost.
        Arguments:
            name: string of the plot title
            p_values: list of p-values (int)
            times: list of timings (seconds)
            costs: list of costs (double)
        Will calculate the range of the costs, default minimum range = 10.0
    """
    min_cost = min(costs)
    cost_range = max(max(costs)-min_cost,min_cost_range)
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(p_values, times, label="time")
    ax1.set_ylabel('Time used to optimize \ seconds')
    ax1.set_xlabel('p_value')
    ax2 = ax1.twinx()
    ax2.plot(p_values, costs, 'r', label="cost")
    ax2.set_ylabel('Cost at SO(3) form')
    ax2.set_xlabel('p_value')
    ax2.set_xticks(p_values)
    ax2.set_ylim(min_cost, min_cost + cost_range)
    plt.title(name, fontsize=12)
    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    plt.show()

def make_convergence_plot(name, p_values, times, costs, iter=10):
    """ Make a bar that show the success rate for each p_value according to whether the SO(3) cost converges
        Arguments:
            name: string of the plot title
            p_values: list of p-values (int)
            times: list of timings (seconds)
            costs: list of costs (double)
            iter: int of iteration number for each p_value
    """
    
    max_cost = np.mean(np.array(heapq.nlargest(iter, costs)))
    # calculate mean costs for each p value
    p_values = list(dict(Counter(p_values)).keys())
    p_mean_cost = [np.mean(np.array(costs[i*iter:(i+1)*iter])) for i in range(len(p_values))]
    p_max = p_values[p_mean_cost.index(max(p_mean_cost))]
    # print(p_mean_cost)
    # print(p_max)

    # calculate the convergence rate for each p_value
    p_success_rates = []
    if p_mean_cost[0] >= 0.9*np.mean(np.array(costs)) and p_mean_cost[0] <= 1.1*np.mean(np.array(costs)):
        p_success_rates = [ 1.0 for p in p_values]
    else:
        for p in p_values:
            if p > p_max:
                p_costs = costs[p_values.index(p)*iter:(p_values.index(p)+1)*iter]
                # print(p_costs)
                converged = [ int(p_cost < 0.3*max_cost) for p_cost in p_costs]
                success_rate = sum(converged)/len(converged)    
                p_success_rates.append(success_rate)
            else:
                p_success_rates.append(0)

    fig, ax = plt.subplots()
    plt.bar(p_values, p_success_rates, align='center', alpha=0.5)
    plt.xticks(p_values)
    plt.yticks(np.arange(0, 1.2, 0.2), ['0%', '20%', '40%', '60%', '80%', '100%'])
    plt.xlabel("p_value")
    plt.ylabel("success rate")
    plt.title(name, fontsize=12)
    plt.show()


# Process arguments and call plot function
import argparse
import csv
import os

parser = argparse.ArgumentParser()
parser.add_argument("path")
args = parser.parse_args()


file_path = []
domain = os.path.abspath(args.path)
for info in os.listdir(args.path):
    file_path.append(os.path.join(domain, info))
file_path.sort()
print(file_path)


# name of all the plots
names = {}
names[0] = 'tinyGrid3D vertex = 9, edge = 11'
names[1] = 'smallGrid3D vertex = 125, edge = 297'
names[2] = 'parking-garage vertex = 1661, edge = 6275'
names[3] = 'sphere2500 vertex = 2500, edge = 4949'
names[4] = 'sphere_bignoise vertex = 2200, edge = 8647'
names[5] = 'torus3D vertex = 5000, edge = 9048'
names[6] = 'cubicle vertex = 5750, edge = 16869'

# Parse CSV file
for key, name in names.items():
    print(key, name)

    # find  according file to process
    name_file = None
    for path in file_path:
        if name[0:7] in path:
            name_file = path
    if name_file == None:
        print("The file %s is not in the path" % name)
        continue

    p_values, times, costs = [],[],[]
    with open(name_file) as csvfile:
        reader = csv.reader(csvfile, delimiter='\t')
        for row in reader:
            print(row)
            p_values.append(int(row[0]))
            times.append(float(row[1]))
            costs.append(float(row[3]))

    #plot
    # make_combined_plot(name, p_values, times, costs)
    make_convergence_plot(name, p_values, times, costs)