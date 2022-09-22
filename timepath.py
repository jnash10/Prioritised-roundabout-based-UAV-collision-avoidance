from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import os


def create_plot(filename): 
    data = pd.read_csv('time.csv')


    bins = [[0],[0],[0],[0],[0]]

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
    for bin in bins:
        if len(bin)>1:
            bin.pop(0)


    fig, axs = plt.subplots(1,2, sharey=False, figsize=(10,10))
    axs[0].bar(['1-2','2-3','3-4','4-5','5-6'],[len(bin) for bin in bins])
    #axs[1].set_ylim([0,40])
    axs[1].boxplot(bins)








    fig.savefig(f'outputs/{filename}/times.eps')
    plt.show()


def create_pathplot(filename):
    n=20
    datas = []
    for i in range(n):
        datas.append(pd.read_csv(f"outputs/{filename}/uav{i+1}.csv"))

    fig, ax = plt.subplots(figsize=(12,12))
    ax.set(xlim=(-100,100), ylim=(-100,100))


    for i in range(n):
        ax.scatter(datas[i].iloc[:,0], datas[i].iloc[:,1],s=5)

    #plt.legend()

    plt.show()
    fig.savefig(f'outputs/{filename}/path.eps')

name = input("please enter filename: ")
create_plot(name)
create_pathplot(name)



