import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'
PREFIX = 'distance-nodes'


fig = plt.figure(figsize=(6.4, 4.8))
ax = fig.add_subplot(111, xlabel='Time [sec]', ylabel='Distance [m]')

pairs = [[0, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 1]]
for u, v in pairs:
    print(u, v)
    df = pd.read_csv(OUTPUT_DIR + 'boids_relay-distance-' + str(u) + '-' + str(v) + '.csv')
    time = df.time
    dist = df.distance
    ax.plot(time, dist, label=str(u) + '-' + str(v))

ax.set_xlim(0.0, 100.0)
ax.set_ylim(0.0, 60.0)
ax.grid()
ax.legend()
fig.savefig(OUTPUT_DIR + PREFIX + '.png')
