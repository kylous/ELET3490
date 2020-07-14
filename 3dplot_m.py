from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import csv

# importing csv data
mapping = np.loadtxt("3dplot.csv", delimiter = ",")

# setting X, Y and Z parameters
x, y = np.meshgrid(mapping[:,0], mapping[:,1])
z = np.tile(mapping[:,2], (len(mapping[:,2]),1))

# 1.1. plot a 3d graph (surface plot)
fig = plt.figure(1)
ax = fig.gca(projection='3d')
ax.plot_surface(x, y, z)
ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
ax.set_zlabel('Depth')

# 1.2. plot a 3d graph (surface plot) with a colormap and colorbar
fig = plt.figure(2)
ax = fig.gca(projection='3d')
surf = ax.plot_surface(x, y, z, cmap='jet')
plt.colorbar(surf)
ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
ax.set_zlabel('Depth')

# 2. heatmap with colorbar
fig = plt.figure(3)
ax = fig.gca()
plt.contourf(x, y, z, cmap='jet')
cbar = plt.colorbar()
ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
cbar.set_label('Depth')

# 3. direct view of Depth values in a grid
fig = plt.figure(4)
ax = fig.gca()
plt.imshow(z, cmap='jet', interpolation='nearest')
cbar = plt.colorbar()
ax.set_xlabel('Latitude Index')
ax.set_ylabel('Longitude Index')
cbar.set_label('Depth')

# show
plt.show()