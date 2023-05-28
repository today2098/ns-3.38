import sys
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'
PREFIX = 'total_rx-' + sys.argv[1] + '-' + sys.argv[2] + '-' + sys.argv[3] + '-' + sys.argv[4] + '-' + sys.argv[5]


fig = plt.figure(figsize=(6.4, 4.8))
ax = fig.add_subplot(111, xlabel='Time [sec]', ylabel='Total Rx [bytes]')

for v in np.arange(2, 7):
    # print(v)
    df = pd.read_csv(OUTPUT_DIR + 'boids_relay-total_rx.csv')
    time = df.time
    rx = df.total_rx
    ax.plot(time, rx)

ax.set_xlim(0.0, 100.0)
ax.set_ylim(0, 4000000)
ax.set_title('Total Rx ($W_S: ' + sys.argv[1] + ', W_A: ' + sys.argv[2] +
             ', W_C: ' + sys.argv[3] + ', Dist: ' + sys.argv[4] + '$)')
ax.grid()
fig.savefig(OUTPUT_DIR + PREFIX + '.png')
