from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import scipy as sp
import scipy.interpolate
import open3d as o3d
from  scipy.spatial import Delaunay


pcd = o3d.io.read_point_cloud('sphere.ply')
points_array = np.asarray(pcd.points)
x = points_array[:,0]
y = points_array[:,1]
z = points_array[:,2]
fig = plt.figure(figsize=(10,6))
ax = axes3d.Axes3D(fig)
ax.scatter3D(x, y, z, c='r')


triangle_mesh = Delaunay(points_array)


fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.plot_trisurf(x, y, z, triangles=triangle_mesh.simplices)


plt.show()



