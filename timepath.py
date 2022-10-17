from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import os

n=3
def create_plot(filename): 
    data = pd.read_csv(f'outputs/{filename}/time.csv')


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


    fig, axs = plt.subplots(1,2, sharey=False, figsize=(14,9))
    axs[0].bar(['1-2','2-3','3-4','4-5','5-6'],[len(bin)-1 for bin in bins])
    fig.suptitle(f'Number of drones = {len(data)}')
    axs[0].set_title(f'Distribution of priority')
    axs[0].set_xlabel('Priority')
    axs[0].set_ylabel('No .of UAVs')
    maxi = max([max(bin) for bin in bins])
    axs[1].set_ylim([0,maxi+5])
    axs[1].boxplot(bins)
    axs[1].set_xlabel('Priority')
    axs[1].set_ylabel('Time taken (seconds)')
    axs[1].set_title('Time taken v/s priority')

    fig.savefig(f'outputs/{filename}/times.eps')
    plt.show()

    fig, ax=plt.subplots(1,1,figsize=(10,12))
    ax.bar(['1-2','2-3','3-4','4-5','5-6'],[len(bin)-1 for bin in bins])
    ax.set_title(f'Distribution of priority.')
    ax.set_xlabel('Priority')
    ax.set_ylabel('UAVs')
    fig.savefig(f'outputs/{filename}/dis.eps')
    plt.show()


    fig, ax=plt.subplots(1,1,figsize=(10,12))
    ax.boxplot(bins)
    ax.set_title('Time taken v/s priority', fontdict=font)
    ax.set_ylim([0,maxi+5])
    ax.set_xlabel('priority', fontdict=font)
    ax.set_ylabel('Time(seconds)', fontdict=font)
    ax.set_ylim((20,40))
    ax.tick_params(axis='both', which='major', labelsize=14)
    fig.savefig(f'outputs/{filename}/bigtime.eps')
    plt.show()

font = {'family': 'serif',
        'color':  'black',
        'weight': 'normal',
        'size': 20,
        }

def create_pathplot(filename):
    global n
    datas = []
    for i in range(n):
        datas.append(pd.read_csv(f"outputs/{filename}/uav{i+1}.csv"))

    fig, ax = plt.subplots(figsize=(12,12))
    ax.set(xlim=(-120,120), ylim=(-120,120))


    for i in range(n):
        ax.scatter(datas[i].iloc[:,0], datas[i].iloc[:,1],s=5)

    #plt.legend()

    plt.show()
    fig.savefig(f'outputs/{filename}/path.eps')

name = input("please enter filename: ")
create_plot(name)
create_pathplot(name)



