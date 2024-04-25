#!/usr/bin/python3

import argparse
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import re
from typing import List
from collections import defaultdict

parser = argparse.ArgumentParser()
parser.add_argument("log_files", nargs="+")
args = parser.parse_args()

dfs = {}

for log_file in args.log_files:
  with open(log_file, 'r') as f:
    lines = f.readlines()
    lines = [re.search(r"#([A-Za-z0-9_\/:]+):\s+([0-9]+\.[0-9]+)ms", l) for l in lines]
    lines = [l for l in lines if l]
    lines = [(m[1], float(m[2])) for m in lines]
    df = pd.DataFrame(lines, columns=["tag", 'dt'])

  len_before_filter = len(df)
  df = df[df['dt'] < 1e5]


  if len(df) != len_before_filter:
    print(f"[WARN] Filtered out {len_before_filter - len(df)} corrupt entries")

  df["dt"]  -= 100

  print(df.groupby("tag").describe([.5, .9, .99, .999]))

  dfs[log_file] = df

tags = list(list(dfs.values())[0]["tag"].unique())

fig, ax = plt.subplots(1, 1, dpi=240, num="Topic Delay")
from cycler import cycler
colors = [plt.cm.Spectral(i) for i in np.linspace(0, 1, len(tags))]
ax.set_prop_cycle(cycler('color', colors))


legends = {}
min_ = 1000000
max_ = 0

for name, df  in dfs.items():
  for tag in tags:
    tag_df = df[df["tag"] == tag]["dt"]
    min_ = min(min_, tag_df.min())
    max_ = max(max_, tag_df.max())

    _, _, tag_patches = ax.hist(tag_df, bins=300, range=(-100, 200), alpha=.7)
    legends[tag] = tag_patches[0]

for tag in tags:
  ax.legend(list(legends.values()), list(legends.keys()))
  #ax.set_title(f"Delay of {tag}")

  ax.set_xlim((min_, max_))
  ax.set_xlabel("Latency [ms]")
  ax.set_ylabel("Frequency [# messages]")

fig.tight_layout()
plt.show()