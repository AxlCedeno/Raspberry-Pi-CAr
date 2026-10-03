import matplotlib.pyplot as plt
import numpy as num
from matplotlib.path import Path
from matplotlib.patches import Wedge
import matplotlib.patches as patches
import random

coordinates = []

fig, ax = plt.subplots(figsize=(12, 4))
ax.set_xlim(1, 24)
ax.set_ylim(1, 10)
ax.set_xticks(range(25))
ax.set_yticks(range(10))
ax.grid(True, color='black', linewidth=1.0)
ax.invert_yaxis()
ax.set_facecolor('#a9c4dd')
obj1 = patches.Rectangle((7, 3), 2, 2, facecolor='black')
obj2 = patches.Rectangle((20, 6), 2, 2, facecolor='black')
ax.add_patch(obj1)
ax.add_patch(obj2)
ax.set_xticklabels([])
ax.set_yticklabels([])

obstacle1 = False

xi = 1
yi = float(input("Y Starting Space: "))
xt = 24
yt = float(input("Y Ending Space: "))
epsilon = 0.5
radius = 1.0

ax.plot(xi - 0.5, yi - 0.5, 'o',color = 'red')
ax.plot(xt - 0.5, yt - 0.5, 'o', color = 'green')

if (yi == 4 or yi == 5):
    obstacle1 = True
    print(f'Obstacle 1: {obstacle1}')



plt.savefig("Testing_Grid.png")
