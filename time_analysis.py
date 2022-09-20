from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


data = pd.read_csv('time.csv')


bins = [[0],[0],[0],[0],[0],[0]]

for i in range(len(data)):
    priority = data['priority'][i]

    if 1<=priority<2:
        bins[0].append(data['time'][i])
    elif 2<=priority<3:
        bins[1].append(data['time'][i])
    elif 3<=priority<4:
        bins[2].append(data['time'][i])
    elif 4<=priority<5:
        bins[3].append(data['time'][i])
    elif 5<=priority<6:
        bins[4].append(data['time'][i])

avgs = [np.mean(bins[i]) for i in range(len(bins))]

plt.bar([2,3,4,5,6],avgs)

plt.show()