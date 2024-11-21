from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

data = pd.read_csv("/home/irol/project_hj/src/intention/resource/241121_01.csv")


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


# Scatter plot
ax.plot(data["Pose.x"], data["Pose.y"], data["Pose.z"])


# Labels
ax.set_xlabel("Pose.x")
ax.set_ylabel("Pose.y")
ax.set_zlabel("Pose.z")


plt.show()