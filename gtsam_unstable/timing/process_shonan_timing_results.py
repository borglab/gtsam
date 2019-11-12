import xlrd
import numpy as np
import matplotlib.pyplot as plt

# workbook = xlrd.open_workbook("//home/jingwu/Desktop/CS8903/Timing_ShonanAveraging.xlsx")
workbook = xlrd.open_workbook("//home/jingwu/Desktop/CS8903/timingresult.xlsx")
booksheet = workbook.sheet_by_index(1)

tinygrid = booksheet.col_values(0)
smallgrid = booksheet.col_values(1)
parkinggarage = booksheet.col_values(2)
sphere2500 = booksheet.col_values(3)
spherebignoise = booksheet.col_values(4)
torus = booksheet.col_values(5)
cubicle = booksheet.col_values(6)

# p_value = np.arange(3, 17)
p_value = np.arange(3, 8)
time1 = [tinygrid[4 + i * 8] for i in range(17)]
cost1 = [tinygrid[8 + i * 8] for i in range(17)]
time2 = [smallgrid[4 + i * 8] for i in range(17)]
cost2 = [smallgrid[8 + i * 8] for i in range(17)]
time3 = [parkinggarage[4 + i * 8] for i in range(11)]
cost3 = [parkinggarage[8 + i * 8] for i in range(11)]
time4 = [sphere2500[4 + i * 8] for i in range(11)]
cost4 = [sphere2500[8 + i * 8] for i in range(11)]
time5 = [spherebignoise[4 + i * 8] for i in range(11)]
cost5 = [spherebignoise[8 + i * 8] for i in range(11)]
time6 = [torus[4 + i * 8] for i in range(5)]
cost6 = [torus[8 + i * 8] for i in range(5)]
time7 = [cubicle[4 + i * 8] for i in range(5)]
cost7 = [cubicle[8 + i * 8] for i in range(5)]

p_time_plot = False
if p_time_plot == True:
    plt.plot(p_value, time1[:5], label="tinygrid vertex = 9, edge = 11")
    plt.plot(p_value, time2[:5], label="smallgrid vertex = 125, edge = 297")
    plt.plot(p_value, time3[:5], label="parkinggarage vertex = 1661, edge = 6275")
    plt.plot(p_value, time4[:5], label="sphere2500 vertex = 2500, edge = 4949")
    plt.plot(p_value, time5[:5], label="spherebignoise vertex = 2200, edge = 8647")
    plt.plot(p_value, time6, label="torus vertex = 5000, edge = 9048")
    plt.plot(p_value, time7, label="cubicle vertex = 5750, edge = 16869")
    plt.xlabel("SO(P), p number")
    plt.ylabel("Time used to optimize \ seconds")
    plt.title('The relationship between the p and time used for optimization', fontsize=12)
    plt.legend(loc='upper right', frameon=True)
    plt.show()
    plt.savefig("p_time_6")

p_cost_time_plot = True
if p_cost_time_plot == True:
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.plot(p_value, time7, label="time")
    ax1.set_ylabel('Time used to optimize \ seconds')
    ax1.set_xlabel('p_value')
    ax2 = ax1.twinx()
    ax2.plot(p_value, cost7, 'r', label="cost")
    ax2.set_ylabel('Cost at SO(P) form')
    ax2.set_xlabel('p_value')
    ax2.set_xticks(p_value)
    # plt.plot(p_value, k[3:30], label=k[2])
    # plt.plot(p_value, l[3:30], label=l[2])
    # plt.plot(p_value, m[3:30], label=m[2])
    # plt.plot(p_value, n[3:30], label=n[2])
    # plt.plot(p_value, o[3:30], label=o[2])
    # plt.plot(p_value, p[3:30], label=p[2])
    # plt.plot(p_value, q[3:30], label=q[2])
    plt.title('cubicle vertex = 5750, edge = 16869', fontsize=12)
    ax1.legend(loc="upper left")
    ax2.legend(loc="upper right")
    plt.show()
