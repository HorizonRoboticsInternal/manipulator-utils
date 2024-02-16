import dill
import numpy as np
import matplotlib.pyplot as plt
from wx250s_kinematics import WX250sParams


# Load all necessary files
with open("data/joint_trajectories.pkl", "rb") as f:
    target = dill.load(f)

with open("data/sim_trajectories.pkl", "rb") as f:
    sim_res = dill.load(f)

with open("data/hw_trajectories.pkl", "rb") as f:
    hw_res = dill.load(f)


# Each side of the triangle trajectory
target = np.concatenate((target[0], target[1], target[2]))
sim_res = np.concatenate((sim_res[0], sim_res[1], sim_res[2]))
hw_res = np.concatenate((hw_res[0], hw_res[1], hw_res[2]))

moving_time = 4.0
num_trajs = 3
time_1 = np.linspace(0, moving_time * num_trajs, hw_res.shape[0])
time_2 = np.linspace(0, moving_time * num_trajs, sim_res.shape[0])
time_3 = np.linspace(0, moving_time * num_trajs, target.shape[0])


# Plot all angles by their joint type
fig, ax = plt.subplots(2, 3, figsize=(16, 8))
joint_names = WX250sParams().joint_names
indices = [(0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2)]
for j, (x, y) in enumerate(indices):
    ax[x, y].set_title(joint_names[j])
    ax[x, y].plot(time_1, hw_res[:, j], label="hardware")
    ax[x, y].plot(time_2, sim_res[:, j], label="sim")
    ax[x, y].plot(time_3, target[:, j], label="target")
plt.legend(fontsize='x-large')
plt.tight_layout()
plt.show()
