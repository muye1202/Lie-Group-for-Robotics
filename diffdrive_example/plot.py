import os
import sys
import json
import numpy as np
from matplotlib import pyplot as plt


if __name__ == "__main__":

  file_path = "/home/muye/repos/lie_group_lib/diffdrive_example/"
  with open(file_path + 'robot_pos.json', 'r') as f:
    traj = json.load(f)

  traj_arr = np.array(traj)
  x, y = traj_arr[:, 0], traj_arr[:, 1]
  plt.plot(x, y, "-b", label="simulated trajectory")
  plt.show()