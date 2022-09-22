import matplotlib.pyplot as plt

import pandas as pd

# n = input("how many drones?")
# n = int(n)

n=40
datas = []
for i in range(n):
    datas.append(pd.read_csv(f"uav{i+1}.csv"))

fig, ax = plt.subplots(figsize=(12,12))
ax.set(xlim=(-100,100), ylim=(-100,100))


for i in range(n):
    ax.scatter(datas[i].iloc[:,0], datas[i].iloc[:,1],s=5)

#plt.legend()


plt.show()