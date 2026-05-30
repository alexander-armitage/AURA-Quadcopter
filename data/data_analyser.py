import pandas as pd
import matplotlib.pyplot as plt

# ============================================================
# FILE
# ============================================================

FILENAME = "DATA_6.TXT"

# ============================================================
# COLUMN NAMES
# ============================================================

columns = [
    "ch1", "ch2", "ch3", "ch4", "ch5", "ch6",
    "esc0", "esc1", "esc2", "esc3",
    "pid_x", "pid_y", "pid_z",
    "set_x", "set_y", "set_z",
    "ori_x", "ori_y", "ori_z",
    "gyro_x", "gyro_y", "gyro_z", "dt",
    "voltage", "current", "power"
]

# ============================================================
# LOAD DATA
# ============================================================

df = pd.read_csv(
    FILENAME,
    skiprows=1,
    names=columns
)

# ============================================================
# WHAT TO PLOT
# ============================================================

GRAPH1 = [
    #"pid_x",
    #"ori_x",
    "gyro_x",
    "set_x",
    #"ch1",
    #"ch2",
    #"ch3",
    #"ch4",
]

GRAPH2 = [
    #"ori_y",
    #"pid_y",
    "gyro_y",
    "set_y",
    #"ch1",
    #"ch2",
    #"ch3",
    #"ch4",
]

GRAPH3 = [
    #"pid_z",
    #"ori_z",
    "gyro_z",
    "set_z",
]

# ============================================================
# PLOT
# ============================================================

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

graph_lists = [GRAPH1, GRAPH2, GRAPH3]

for ax, signals in zip(axes, graph_lists):

    for signal in signals:

        if signal not in df.columns:
            print(f"Warning: '{signal}' not found")
            continue

        #if signal.startswith("set"):
        #    ax.plot(df[signal][100:]*180/3.1415, label=signal)
        #else:
        ax.plot(df[signal], label=signal)

    ax.legend()
    ax.grid(True)

plt.xlabel("Sample")
plt.tight_layout()
plt.xlim(left=0,right=650)
plt.show()
