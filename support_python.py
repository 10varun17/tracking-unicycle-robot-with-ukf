import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

df_true = pd.read_csv("robot_pose.csv")
df_noisy = pd.read_csv("sensor_pose.csv")
df_est = pd.read_csv("ukf_pose.csv")

fig, ax = plt.subplots(1,1, figsize=(16,12))
custom_lines = [Line2D([0], [0], color="green", lw=4),
                Line2D([0], [0], color="red", lw=4),
                Line2D([0], [0], color="blue", lw=4)]

df_true.plot("x", "y", kind="line", lw=1.5, c= "green", ax=ax)
df_noisy.plot("x", "y", kind="scatter", s=1.5, c="red", marker="x", ax=ax)
df_est.plot("x", "y", kind="line", lw="1", c="blue", ax=ax)
ax.set_title("Ground Truth vs. Measurements vs. Estimated States")
ax.legend(custom_lines, ["Ground Truth", "Measurements", "Estimated States"])
plt.savefig("ukf_results.png")
plt.show()