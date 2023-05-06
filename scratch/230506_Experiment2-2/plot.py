# 開始後100秒間のノードの動きを10倍速でプロットする．

# import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


OUTPUT_DIR = 'output/ex2-2/'
PREFIX = 'plot'
N = 26  # ノード数．
W = 12  # プロット幅．
INTERVAL = 100  # [msec/frame]
FRAMES = 100

data = []
for i in range(N):
    data.append(pd.read_csv(OUTPUT_DIR + 'ex2-2_position_' + str(i) + '.csv'))


# 開始time[sec]におけるノードiの座標を取得する．
def get_data(i, time):
    df = data[i]
    x = df.iloc[time].x
    y = df.iloc[time].y
    z = df.iloc[time].z
    color = 'tab:blue'
    if i == 25:
        color = 'tab:red'
    elif z >= 35:
        color = 'tab:green'
    return x, y, z, color


# Plot by 2D (xy).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(1, 1, 1)


def frame_2d_xy(i):
    ax.cla()

    for j in range(N):
        x, y, _, color = get_data(j, i)
        ax.scatter(x, y, color=color)
        print('plot by xy', i, j, x, y, color)

    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_2d_xy, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '_2d_xy.gif')


# Plot by 2D (xz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(1, 1, 1)


def frame_2d_xz(i):
    ax.cla()

    for j in range(N):
        x, _, z, color = get_data(j, i)
        ax.scatter(x, z, color=color)
        print('plot by xz', i, j, x, z, color)

    ax.set_xlim(-200, 200)
    ax.set_ylim(0, 200)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_2d_xz, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '_2d_xz.gif')


# Plot by 3D (xyz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(1, 1, 1, projection='3d')


def frame_3d(i):
    ax.cla()

    for j in range(N):
        x, y, z, color = get_data(j, i)
        ax.scatter(x, y, z, color=color)
        print('plot by xyz', i, j, x, y, z, color)

    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_zlim(0, 50)
    # ax.view_init(elev=90, azim=270)


ani = animation.FuncAnimation(fig, frame_3d, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '_3d.gif')
# plt.show()
