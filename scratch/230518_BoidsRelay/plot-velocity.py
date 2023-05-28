import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'
PREFIX = 'velocity-' + sys.argv[1] + '-' + sys.argv[2] + '-' + sys.argv[3] + '-' + sys.argv[4] + '-' + sys.argv[5]


fig = plt.figure(figsize=(6.4, 4.8))
ax = fig.add_subplot(111, xlabel='Time [sec]', ylabel='Distance [m]')

for v in np.arange(2, 7):
    # print(v)
    df = pd.read_csv(OUTPUT_DIR + 'boids_relay-velocity-' + str(v) + '.csv')
    time = df.time
    dist = df.velocity
    ax.plot(time, dist, label='node ' + str(v))

ax.set_xlim(0.0, 100.0)
ax.set_ylim(0.0, 20.0)
ax.set_title('Velocity ($W_S: ' + sys.argv[1] + ', W_A: ' + sys.argv[2] +
             ', W_C: ' + sys.argv[3] + ', Dist: ' + sys.argv[4] + '$)')
ax.grid()
ax.legend()
fig.savefig(OUTPUT_DIR + PREFIX + '.png')
