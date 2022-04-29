from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import scipy as sp
import scipy.interpolate
import open3d as o3d
from geomdl import BSpline
from geomdl import utilities
from geomdl import exchange
from geomdl.visualization import VisMPL
from plyfile import PlyData, PlyElement
import pandas as pd
import random

# pcd = o3d.io.read_point_cloud('sphere.ply')
# plydata = PlyData.read('sphere.ply')
# points_array = np.asarray(pcd.points)
# x = points_array[:,0]
# y = points_array[:,1]
# z = points_array[:,2]
# points = np.vstack((x,y,z)).transpose()
# points2 = points.astype(int)
# n_samples = 400

# indices = np.arange(0, n_samples, dtype=float) + 0.5
# phi = np.arccos(1 - 2 * indices / n_samples)
# theta = np.pi * (1 + 5**0.5) * indices

# x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi);
# points  = np.vstack( (x, y, z)).T
#print(points2)
x = [0,2,0,-2,-2,2]
y = [0,3,0,-3,3,-3]
z = [20,30,40,30,30,30]
fig = plt.figure(figsize=(10,6))
ax = axes3d.Axes3D(fig)
ax.scatter3D(x, y, z, c='r')
plt.show()
# import scipy as sp
# import scipy.interpolate
# spline = sp.interpolate.bisplrep(x,y,z)

# x_grid = np.linspace(0, 132651, 1000*len(x))
# y_grid = np.linspace(0, 132651, 1000*len(y))
# B1, B2 = np.meshgrid(x_grid, y_grid, indexing='xy')
# Z = np.zeros((x.size, z.size))

# Z = sp.interpolate.splev(B1,B2, spline)
# fig = plt.figure(figsize=(10,6))
# ax = axes3d.Axes3D(fig)
# ax.plot_wireframe(B1, B2, Z)
# ax.plot_surface(B1, B2, Z,alpha=0.2)
# plot.show()

surf = BSpline.Surface()
surf.degree_u = 1
surf.degree_v = 1
control_points = [[0,0,20],
                  [-2,-3,30],
                  [2,3,30],
                  [-2,3,30],
                  [2,-3,30],
                  [0,0,20],
                  [0,0,40],
                  [-2,-3,30],
                  [2,3,30],
                  [-2,3,30],
                  [2,-3,30],
                  [0,0,40]]


surf.set_ctrlpts(control_points, 6, 2)
surf.knotvector_u = utilities.generate_knot_vector(surf.degree_u, 6)
surf.knotvector_v = utilities.generate_knot_vector(surf.degree_v, 2) 

 # Evaluate surface
surf.evaluate()
vis_comp = VisMPL.VisSurface()
surf.vis = vis_comp
surf.render()
 # Save surface as a .obj file
exchange.export_obj(surf, "bezier_surf.obj")

# x_grid = np.linspace(0, 0.5, len(x))
# y_grid = np.linspace(0, 0.5, len(y))
# B1, B2 = np.meshgrid(x_grid, y_grid, indexing='xy')
# Z = np.zeros((x.size, z.size))

# spline = sp.interpolate.Rbf(x,y,z,function='gaussian',smooth=2, episilon=3)

# Z = spline(B1,B2)
# fig = plt.figure(figsize=(10,6))
# ax = axes3d.Axes3D(fig)
# ax.plot_wireframe(B1, B2, Z)
# ax.plot_surface(B1, B2, Z,alpha=0.2)
# ax.scatter3D(x, y, z, c='r')
# #ax.scatter3D(x, y, z, c='r')
# # ax.plot_surface(B1*10, B2*10, Z,alpha=0.2)
# # ax.plot_surface(B1+20, B2+20, Z,alpha=0.2)
# plt.show()