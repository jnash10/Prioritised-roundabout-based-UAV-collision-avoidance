import matplotlib.pyplot as plt
import pandas as pd


data = pd.read_csv('pose.csv')


x = data[:,0]
y = data[:,1]

fig = plt.plot(x,y)

