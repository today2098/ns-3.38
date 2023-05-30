# import sys
# import numpy as np
# import matplotlib.pyplot as plt
import pandas as pd


OUTPUT_DIR = 'output/boids_relay/'


pd.set_option('display.max_rows', 500)

df = pd.read_csv(OUTPUT_DIR + 'boids_relay.csv', header=None, names=['ws', 'wa', 'wc', 'dist', 'enemy', 'ploss'])
print(df.sort_values(by=['ploss']))

res = df.corr()
print(res)
