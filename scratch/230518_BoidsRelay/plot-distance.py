import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


OUTPUT_DIR = 'output/boids-relay/'
PREFIX = 'distance'

plt.figure(figsize=(6.4, 4.8))
pairs = [[0, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 1]]
for u, v in pairs:
    print(u, v)
    df = pd.read_csv(OUTPUT_DIR + 'boids-relay_distance_' + str(u) + '_' + str(v) + '.csv')
    time = df.time
    dist = df.distance
    plt.plot(time, dist, label=str(u) + '-' + str(v))
plt.xlim(0.0, 100.0)
plt.ylim(0.0, 60.0)
plt.xlabel('time [sec]')
plt.ylabel('distance [m]')
plt.grid()
plt.legend()
plt.savefig(OUTPUT_DIR + PREFIX + '.png')
