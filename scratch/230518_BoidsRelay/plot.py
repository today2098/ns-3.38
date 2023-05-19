# 開始後100秒間のノードの動きを10倍速でプロットする．

# import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'
PREFIX = 'position'

W = 10  # プロット幅．
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
    if i < 2:
        color = 'tab:orange'  # BS.
    elif i == 7:
        color = 'tab:red'  # Enemy.
    elif z >= 35:
        color = 'tab:green'
    return x, y, z, color


# Plot by 2D (xy).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111)


def frame_xy(i):
    ax.cla()
    for j in range(N):
        x, y, _, color = get_data(j, i)
        ax.scatter(x, y, color=color)
        print('plot by xy', i, j, x, y, color)
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_xy, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xy.gif')


# Plot by 2D (xz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111)


def frame_xz(i):
    ax.cla()
    for j in range(N):
        x, _, z, color = get_data(j, i)
        ax.scatter(x, z, color=color)
        print('plot by xz', i, j, x, z, color)
    ax.set_xlim(-200, 200)
    ax.set_ylim(0, 400)
    ax.grid()


ani = animation.FuncAnimation(fig, frame_xz, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xz.gif')


# Plot by 3D (xyz).
fig = plt.figure(figsize=(W, W))
ax = fig.add_subplot(111, projection='3d')


def frame_xyz(i):
    ax.cla()
    for j in range(N):
        x, y, z, color = get_data(j, i)
        ax.scatter(x, y, z, color=color)
        print('plot by xyz', i, j, x, y, z, color)
    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_zlim(0, 100)
    # ax.view_init(elev=90, azim=270)  # 視点の角度を調整する．


ani = animation.FuncAnimation(fig, frame_xyz, interval=INTERVAL, frames=FRAMES)
ani.save(OUTPUT_DIR + PREFIX + '-xyz.gif')
plt.close()
