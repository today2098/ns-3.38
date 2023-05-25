# 開始後100秒間のノードの動きを10倍速でプロットする．

# import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'
PREFIX = 'mobility'

W = 8.0  # プロット幅．
INTERVAL = 100  # [msec/frame]
FRAMES = 100
N = 8  # ノード数．


data = []
for i in range(N):
    data.append(pd.read_csv(OUTPUT_DIR + 'boids_relay-position-' + str(i) + '.csv'))


# 開始time[sec]におけるノードiの座標を取得する．
def get_data(i, time):
    df = data[i]
    x = df.iloc[time].x
    y = df.iloc[time].y
    z = df.iloc[time].z
    color = 'tab:blue'  # Boids.
    label = 'Boids'
    if i < 2:
        color = 'tab:orange'  # BS.
        label = 'BS'
    elif i == 7:
        color = 'tab:red'  # Enemy.
        label = 'Enemy'
    elif z >= 35:
        color = 'tab:green'
    return x, y, z, color, label


def legend_without_duplicate_labels(figure):
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    figure.legend(by_label.values(), by_label.keys())


# Plot by 2D (xy).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111)


def frame_xy(i):
    ax.cla()
    for j in range(N):
        x, y, _, color, label = get_data(j, i)
        ax.scatter(x, y, color=color, label=label)
        print('plot by xy', i, j, x, y, color, label)
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')
    legend_without_duplicate_labels(ax)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_xy, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xy.gif')


# Plot by 2D (xz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111)


def frame_xz(i):
    ax.cla()
    for j in range(N):
        x, _, z, color, label = get_data(j, i)
        ax.scatter(x, z, color=color, label=label)
        print('plot by xz', i, j, x, z, color, label)
    ax.set_xlim(-200, 200)
    ax.set_ylim(0, 400)
    ax.set_xlabel('$x$')
    ax.set_ylabel('$z$')
    legend_without_duplicate_labels(ax)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_xz, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xz.gif')


# Plot by 3D (xyz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111, projection='3d')


def frame_xyz(i):
    ax.cla()
    for j in range(N):
        x, y, z, color, label = get_data(j, i)
        ax.scatter(x, y, z, color=color, label=label)
        print('plot by xyz', i, j, x, y, z, color, label)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_zlim(0, 100)
    ax.set_xlabel('$x$')
    ax.set_ylabel('$y$')
    ax.set_zlabel('$z$')
    legend_without_duplicate_labels(ax)
    # ax.view_init(elev=90, azim=270)  # 視点の角度を調整する．


ani = animation.FuncAnimation(fig, frame_xyz, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xyz.gif')
plt.close()
